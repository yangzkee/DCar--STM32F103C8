#include <deque>
#include <stdio.h>
#include <vector>

#include "DFCom.h"

HardwareSerial Serial;

static uint32_t g_now_ms;
static int g_failures;

uint32_t millis()
{
    return g_now_ms++;
}

void delay(unsigned long ms)
{
    g_now_ms += (uint32_t)ms;
}

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            printf("FAIL %s:%d: %s\n", __func__, __LINE__, #condition);         \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

class FakeSerial : public HardwareSerial {
public:
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    long baud = 0;
    bool failWrite = false;

    void begin(long value) override
    {
        baud = value;
    }

    int available() override
    {
        return (int)rx.size();
    }

    int read() override
    {
        if (rx.empty()) return -1;
        uint8_t value = rx.front();
        rx.pop_front();
        return value;
    }

    size_t write(const uint8_t *data, size_t size) override
    {
        if (failWrite) return 0;
        tx.insert(tx.end(), data, data + size);
        return size;
    }

    void clear()
    {
        rx.clear();
        tx.clear();
        failWrite = false;
    }
};

static std::vector<uint8_t> progressFrame(uint8_t cmd, uint8_t progress,
                                          uint8_t notice)
{
    std::vector<uint8_t> frame;
    uint16_t sum = 0;

    frame.push_back(0xDF);
    frame.push_back(0x97);
    frame.push_back(0x01);
    frame.push_back(0x6F);
    frame.push_back(cmd);
    frame.push_back(2);
    frame.push_back(progress);
    frame.push_back(notice);
    frame.push_back(0xFD);
    for (size_t i = 0; i < frame.size(); ++i) sum += frame[i];
    frame.push_back((uint8_t)(sum & 0xFF));
    frame.push_back((uint8_t)(sum >> 8));
    return frame;
}

static void inject(FakeSerial &serial, const std::vector<uint8_t> &frame)
{
    serial.rx.insert(serial.rx.end(), frame.begin(), frame.end());
}

static unsigned countCommand(const FakeSerial &serial, uint8_t cmd)
{
    unsigned count = 0;
    size_t pos = 0;

    while (pos + 8 < serial.tx.size()) {
        if (serial.tx[pos] != 0xDF) {
            ++pos;
            continue;
        }
        uint8_t len = serial.tx[pos + 5];
        size_t frameSize = (size_t)len + 9;
        if (pos + frameSize > serial.tx.size()) break;
        if (serial.tx[pos + 4] == cmd) ++count;
        pos += frameSize;
    }
    return count;
}

static void start(FakeSerial &serial)
{
    g_now_ms = 0;
    serial.clear();
    DFCom.begin(115200, serial);
}

static void test_begin_establishes_stopped_session()
{
    FakeSerial serial;
    start(serial);

    CHECK(serial.baud == 115200);
    CHECK(countCommand(serial, DFCOM_CMD_VEL) == 2);
    CHECK(g_now_ms >= 1000);
}

static void test_fast_terminal_survives_wait_entry()
{
    FakeSerial serial;
    start(serial);
    serial.clear();

    DFCom.moveLinear(50, 0, 30, 2);
    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, 0x00));
    DFCom.update();

    CHECK(DFCom.waitDone(DFCOM_CMD_LINEAR, 20));
}

static void test_predecessor_terminal_does_not_pollute_successor()
{
    FakeSerial serial;
    start(serial);
    serial.clear();

    DFCom.moveLinear(50, 0, 30, 2);
    DFCom.moveLinear(-50, 0, 30, 2);

    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, DFCOM_CMD_LINEAR));
    DFCom.update();
    CHECK(DFCom.progress(DFCOM_CMD_LINEAR) == 0);

    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0x80, 0x00));
    DFCom.update();
    CHECK(DFCom.progress(DFCOM_CMD_LINEAR) == 0x80);

    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, 0x00));
    DFCom.update();
    CHECK(DFCom.waitDone(DFCOM_CMD_LINEAR, 20));
}

static void test_timeout_is_a_maximum_execution_window()
{
    FakeSerial serial;
    start(serial);
    serial.clear();

    DFCom.moveRot(15, 60);
    uint32_t before = g_now_ms;
    CHECK(!DFCom.waitDone(DFCOM_CMD_ROT, 20));
    CHECK(g_now_ms - before >= 20);

    DFCom.moveLinear(10, 0, 20, 2);
    inject(serial, progressFrame(DFCOM_CMD_ROT, 0xFF, DFCOM_CMD_LINEAR));
    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, 0x00));
    CHECK(DFCom.waitDone(DFCOM_CMD_LINEAR, 20));
}

static void test_lost_terminal_never_causes_false_early_done()
{
    FakeSerial serial;
    start(serial);
    serial.clear();

    DFCom.moveLinear(50, 0, 30, 2);   // A
    DFCom.moveLinear(-50, 0, 30, 2);  // B; A terminal is lost
    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, 0x00));
    CHECK(!DFCom.waitDone(DFCOM_CMD_LINEAR, 20));

    DFCom.moveLinear(50, 0, 30, 2);   // C force-reuses A
    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, 0x00));
    CHECK(!DFCom.waitDone(DFCOM_CMD_LINEAR, 20));

    DFCom.moveRot(15, 60);            // different command restores routing
    inject(serial, progressFrame(DFCOM_CMD_ROT, 0xFF, 0x00));
    CHECK(DFCom.waitDone(DFCOM_CMD_ROT, 20));
}

static void test_velocity_preempts_without_creating_wait_slot()
{
    FakeSerial serial;
    start(serial);
    serial.clear();

    DFCom.moveLinear(50, 0, 30, 2);
    DFCom.moveVel(0, 0, 0);
    CHECK(!DFCom.waitDone(DFCOM_CMD_LINEAR, 20));

    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, DFCOM_CMD_VEL));
    DFCom.update();
    DFCom.moveLinear(10, 0, 20, 2);
    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, 0x00));
    CHECK(DFCom.waitDone(DFCOM_CMD_LINEAR, 20));
}

static void test_failed_send_does_not_bind_previous_task()
{
    FakeSerial serial;
    start(serial);
    serial.clear();

    DFCom.moveLinear(10, 0, 20, 2);
    inject(serial, progressFrame(DFCOM_CMD_LINEAR, 0xFF, 0x00));
    DFCom.update();

    serial.failWrite = true;
    DFCom.moveLinear(-10, 0, 20, 2);
    uint32_t before = g_now_ms;
    CHECK(!DFCom.waitDone());
    CHECK(g_now_ms - before < 20);

    serial.failWrite = false;
    DFCom.moveRot(15, 0);
    CHECK(!DFCom.waitDone());
}

static void test_500hz_subscription_uses_protocol_code_50()
{
    FakeSerial serial;
    uint16_t frequencyHz = 500;
    start(serial);
    serial.clear();

    DFCom.subscribeOdom(frequencyHz);

    CHECK(serial.tx.size() == 11);
    CHECK(serial.tx[4] == 0x80);
    CHECK(serial.tx[5] == 2);
    CHECK(serial.tx[6] == 0x01);
    CHECK(serial.tx[7] == 50);
}

int main()
{
    test_begin_establishes_stopped_session();
    test_fast_terminal_survives_wait_entry();
    test_predecessor_terminal_does_not_pollute_successor();
    test_timeout_is_a_maximum_execution_window();
    test_lost_terminal_never_causes_false_early_done();
    test_velocity_preempts_without_creating_wait_slot();
    test_failed_send_does_not_bind_previous_task();
    test_500hz_subscription_uses_protocol_code_50();

    if (g_failures != 0) {
        printf("%d Arduino DFCom test(s) failed\n", g_failures);
        return 1;
    }

    printf("all Arduino DFCom tests passed\n");
    return 0;
}

#include <stdio.h>
#include <vector>

#include "Arduino.h"
#include "../../DFCom_Arduino/SingleFile/DcarON_Square_AllInOne/DcarON_Square_AllInOne.ino"

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

static void inject(const std::vector<uint8_t> &frame)
{
    Serial.testRx.insert(Serial.testRx.end(), frame.begin(), frame.end());
}

static unsigned countCommand(uint8_t cmd)
{
    unsigned count = 0;
    size_t pos = 0;

    while (pos + 8 < Serial.testTx.size()) {
        if (Serial.testTx[pos] != 0xDF) {
            ++pos;
            continue;
        }
        uint8_t len = Serial.testTx[pos + 5];
        size_t frameSize = (size_t)len + 9;
        if (pos + frameSize > Serial.testTx.size()) break;
        if (Serial.testTx[pos + 4] == cmd) ++count;
        pos += frameSize;
    }
    return count;
}

int main()
{
    g_now_ms = 0;
    Serial.testClear();
    setup();

    CHECK(Serial.testBaud == 115200);
    CHECK(countCommand(0x62) == 2);
    CHECK(g_now_ms >= 1000);

    Serial.testClear();
    dcarMoveLinear(50, 0, 30, 2);
    inject(progressFrame(0x64, 0xFF, 0x00));
    dcarUpdate();
    CHECK(dcarWaitDone(20));

    dcarMoveLinear(50, 0, 30, 2);
    dcarMoveLinear(-50, 0, 30, 2);
    inject(progressFrame(0x64, 0xFF, 0x64));
    dcarUpdate();
    CHECK(dfCurrentSlot <= 1);
    CHECK(mvSlots[dfCurrentSlot].progress == 0);

    inject(progressFrame(0x64, 0xFF, 0x00));
    dcarUpdate();
    CHECK(dcarWaitDone(20));

    Serial.testClear();
    dcarSubscribeOdom(500);
    CHECK(Serial.testTx.size() == 11);
    CHECK(Serial.testTx[7] == 50);

    if (g_failures != 0) {
        printf("%d single-file test(s) failed\n", g_failures);
        return 1;
    }

    printf("all Arduino single-file tests passed\n");
    return 0;
}

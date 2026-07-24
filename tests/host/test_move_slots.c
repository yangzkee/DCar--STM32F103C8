#include <stdio.h>
#include <string.h>
#include "DFCom.h"

/* Transport-to-tracker hook; intentionally absent from the public header. */
void DFCom_MoveCommandSent(u8 cmd_code);

volatile u32 g_local_tick_ms = 0;

static unsigned g_delay_calls;
static void (*g_delay_hook)(void);
static unsigned g_dma_calls;
static unsigned g_dma_failures_remaining;
static u8 g_last_tx_frame[100];
static u8 g_last_tx_size;
static int g_failures;

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            printf("FAIL %s:%d: %s\n", __func__, __LINE__, #condition);         \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

void delay_ms(u32 ms)
{
    g_local_tick_ms += ms;
    g_delay_calls++;
    if (g_delay_hook != 0) g_delay_hook();
}

void delay_us(u32 us)
{
    (void)us;
}

u8 USART1_Send_By_DMA(u8 *data, u8 size)
{
    g_dma_calls++;
    if (g_dma_failures_remaining > 0) {
        g_dma_failures_remaining--;
        return 1;
    }

    memcpy(g_last_tx_frame, data, size);
    g_last_tx_size = size;
    return 0;
}

static u16 build_progress_frame(u8 *out, u8 cmd, u8 progress, u8 notice)
{
    u16 sum = 0;
    u8 i;

    out[0] = DFCOM_FRAME_HEADER;
    out[1] = DFCOM_PC_ID;
    out[2] = DFCOM_ROBOT_ID;
    out[3] = 0x6F;
    out[4] = cmd;
    out[5] = 2;
    out[6] = progress;
    out[7] = notice;
    out[8] = DFCOM_FRAME_TAIL;

    for (i = 0; i < 9; i++) sum += out[i];
    out[9]  = (u8)(sum & 0xFF);
    out[10] = (u8)(sum >> 8);
    return 11;
}

static void feed_progress(u8 cmd, u8 progress, u8 notice)
{
    u8 frame[11];
    build_progress_frame(frame, cmd, progress, notice);
    DFCom_RxParse(frame, sizeof(frame));
}

static void reset_started(void)
{
    g_local_tick_ms = 0;
    g_delay_calls = 0;
    g_delay_hook = 0;
    g_dma_calls = 0;
    g_dma_failures_remaining = 0;
    g_last_tx_size = 0;
    memset(g_last_tx_frame, 0, sizeof(g_last_tx_frame));
    DFCom_MoveSessionQuarantine();
    DFCom_MoveSessionStart();
}

static void check_slot_zero(u8 slot)
{
    CHECK(slot < 2);
    CHECK(g_move_status[slot].cmd_code == 0);
    CHECK(g_move_status[slot].progress == 0);
    CHECK(g_move_status[slot].last_notice == 0);
    CHECK(g_move_status[slot].fresh == 0);
}

static void test_startup_quarantine(void)
{
    u8 i;

    reset_started();
    DFCom_MoveSessionQuarantine();
    Cmd_Move_Vel(0, 0, 0);
    CHECK(g_dma_calls == 1);
    CHECK(g_last_tx_size == 21);
    CHECK(g_last_tx_frame[4] == CMD_VEL);
    for (i = 6; i < 18; i++) CHECK(g_last_tx_frame[i] == 0);

    feed_progress(CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE);

    check_slot_zero(0);
    check_slot_zero(1);
    CHECK(WaitMoveDone(CMD_LINEAR, 100) == MOVE_WAIT_TIMEOUT);
}

static void test_fast_terminal_before_wait(void)
{
    reset_started();
    Cmd_Move_Linear(50, 0, 30, 2);
    CHECK(g_dma_calls == 1);
    CHECK(g_last_tx_size == 22);
    CHECK(g_last_tx_frame[3] == 0x02);
    CHECK(g_last_tx_frame[4] == CMD_LINEAR);
    feed_progress(CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE);

    CHECK(WaitMoveDone(CMD_LINEAR, 100) == MOVE_WAIT_DONE);
    CHECK(g_local_tick_ms == 0);
}

static void inject_linear_done_on_second_delay(void)
{
    if (g_delay_calls == 2) {
        g_delay_hook = 0;
        feed_progress(CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE);
    }
}

static void test_terminal_during_wait(void)
{
    reset_started();
    DFCom_MoveCommandSent(CMD_LINEAR);
    g_delay_hook = inject_linear_done_on_second_delay;

    CHECK(WaitMoveDone(CMD_LINEAR, 100) == MOVE_WAIT_DONE);
    CHECK(g_local_tick_ms == 20);
}

static void test_timeout_is_maximum_window(void)
{
    reset_started();
    DFCom_MoveCommandSent(CMD_LINEAR);

    CHECK(WaitMoveDone(CMD_LINEAR, 25) == MOVE_WAIT_TIMEOUT);
    CHECK(g_local_tick_ms == 30);
}

static void test_same_type_old_terminal_is_swallowed(void)
{
    u8 frames[33];
    u16 used = 0;

    reset_started();
    DFCom_MoveCommandSent(CMD_LINEAR);  /* slot A */
    feed_progress(CMD_LINEAR, 0x40, MOVE_NOTICE_NONE);
    DFCom_MoveCommandSent(CMD_LINEAR);  /* slot B; A becomes discard */

    CHECK(GetMoveProgress(CMD_LINEAR) == 0);
    check_slot_zero(0);

    used += build_progress_frame(&frames[used], CMD_LINEAR, 0xFF, CMD_LINEAR);
    used += build_progress_frame(&frames[used], CMD_LINEAR, 0x80, MOVE_NOTICE_NONE);
    used += build_progress_frame(&frames[used], CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE);
    DFCom_RxParse(frames, used);

    check_slot_zero(0);
    CHECK(g_move_status[1].cmd_code == CMD_LINEAR);
    CHECK(g_move_status[1].progress == 0xFF);
    CHECK(WaitMoveDone(CMD_LINEAR, 100) == MOVE_WAIT_DONE);
}

static void test_different_type_routes_without_waiting_for_old(void)
{
    reset_started();
    DFCom_MoveCommandSent(CMD_LINEAR);
    DFCom_MoveCommandSent(CMD_ROT);

    feed_progress(CMD_ROT, 0xFF, MOVE_NOTICE_NONE);
    CHECK(WaitMoveDone(CMD_ROT, 100) == MOVE_WAIT_DONE);

    feed_progress(CMD_LINEAR, 0xFF, CMD_ROT);
    check_slot_zero(0);
}

static void test_missing_terminal_forces_slot_reuse(void)
{
    reset_started();
    DFCom_MoveCommandSent(CMD_LINEAR);  /* A: terminal deliberately omitted */
    DFCom_MoveCommandSent(CMD_LINEAR);  /* B: A is discard */
    CHECK(WaitMoveDone(CMD_LINEAR, 25) == MOVE_WAIT_TIMEOUT);
    DFCom_MoveCommandSent(CMD_LINEAR);  /* A: force-reuse; B is discard */

    CHECK(g_move_status[0].cmd_code == CMD_LINEAR);
    check_slot_zero(1);

    feed_progress(CMD_LINEAR, 0xFF, CMD_LINEAR);       /* B interrupted by C */
    feed_progress(CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE); /* C natural completion */
    CHECK(WaitMoveDone(CMD_LINEAR, 100) == MOVE_WAIT_DONE);
}

/*
 * DFLink 没有线上的 task id。若 A 的终止帧真的丢失，而 B 又在同 B 码下自然
 * 完成，客户端必须选择保守：宁可让 B/C 各等满上限，也不能把模糊的旧 FF
 * 当成当前任务完成而提前跳指令。换到不同命令码后路由会立即重新明确。
 */
static void test_lost_terminal_never_causes_false_early_done(void)
{
    reset_started();
    DFCom_MoveCommandSent(CMD_LINEAR);  /* A */
    DFCom_MoveCommandSent(CMD_LINEAR);  /* B; A terminal is lost */

    feed_progress(CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE); /* B FF is conservatively A */
    CHECK(WaitMoveDone(CMD_LINEAR, 25) == MOVE_WAIT_TIMEOUT);

    DFCom_MoveCommandSent(CMD_LINEAR);  /* C force-reuses A */
    feed_progress(CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE); /* C FF is conservatively B */
    CHECK(WaitMoveDone(CMD_LINEAR, 25) == MOVE_WAIT_TIMEOUT);

    DFCom_MoveCommandSent(CMD_ROT);      /* different B restores unambiguous routing */
    feed_progress(CMD_ROT, 0xFF, MOVE_NOTICE_NONE);
    CHECK(WaitMoveDone(CMD_ROT, 100) == MOVE_WAIT_DONE);
}

static void test_current_interrupt_is_not_success(void)
{
    reset_started();
    DFCom_MoveCommandSent(CMD_ROT);
    feed_progress(CMD_ROT, 0xFF, MOVE_NOTICE_RC_TAKEOVER);

    CHECK(WaitMoveDone(CMD_ROT, 100) == MOVE_WAIT_INTERRUPTED);
    CHECK(GetMoveNotice(CMD_ROT) == MOVE_NOTICE_RC_TAKEOVER);
}

static void test_send_failure_cannot_bind_previous_task(void)
{
    reset_started();
    Cmd_Move_Linear(50, 0, 30, 2);
    feed_progress(CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE);
    g_dma_failures_remaining = 3;
    Cmd_Move_Linear(-50, 0, 30, 2);

    CHECK(WaitMoveDone(CMD_LINEAR, 100) == MOVE_WAIT_TIMEOUT);
    CHECK(g_local_tick_ms == 0);
    CHECK(g_dma_calls == 4);
}

static void test_velocity_preemption_has_no_wait_slot(void)
{
    reset_started();
    DFCom_MoveCommandSent(CMD_LINEAR);
    DFCom_MoveCommandSent(CMD_VEL);

    CHECK(WaitMoveDone(CMD_LINEAR, 100) == MOVE_WAIT_TIMEOUT);
    feed_progress(CMD_LINEAR, 0xFF, CMD_VEL);
    check_slot_zero(0);

    DFCom_MoveCommandSent(CMD_LINEAR);
    feed_progress(CMD_LINEAR, 0xFF, MOVE_NOTICE_NONE);
    CHECK(WaitMoveDone(CMD_LINEAR, 100) == MOVE_WAIT_DONE);
}

static void test_500hz_subscription_uses_protocol_code_50(void)
{
    u16 frequency_hz = 500;

    reset_started();
    Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS, frequency_hz);

    CHECK(g_last_tx_size == 11);
    CHECK(g_last_tx_frame[4] == 0x80);
    CHECK(g_last_tx_frame[5] == 2);
    CHECK(g_last_tx_frame[6] == 0x01);
    CHECK(g_last_tx_frame[7] == 50);
}

int main(void)
{
    test_startup_quarantine();
    test_fast_terminal_before_wait();
    test_terminal_during_wait();
    test_timeout_is_maximum_window();
    test_same_type_old_terminal_is_swallowed();
    test_different_type_routes_without_waiting_for_old();
    test_missing_terminal_forces_slot_reuse();
    test_lost_terminal_never_causes_false_early_done();
    test_current_interrupt_is_not_success();
    test_send_failure_cannot_bind_previous_task();
    test_velocity_preemption_has_no_wait_slot();
    test_500hz_subscription_uses_protocol_code_50();

    if (g_failures != 0) {
        printf("%d move-slot test(s) failed\n", g_failures);
        return 1;
    }

    printf("all move-slot tests passed\n");
    return 0;
}

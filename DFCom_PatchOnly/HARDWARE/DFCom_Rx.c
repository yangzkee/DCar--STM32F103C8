/*****************************************************************************
 * DFCom_Rx.c - 接收侧实现（SI 单位 / ROS REP-103 坐标系）
 * ---------------------------------------------------------------------------
 * 功能：从 USART1 IDLE 中断收到 DcarON 回传的字节流，做帧定界 + 校验和 +
 *       按 (A, B) 分发到对应解析函数，把数据写到 g_odom / g_move_status /
 *       g_dcar_state 等全局变量供应用层读取。
 *
 *
 * ★★ DF_Link 回传帧总格式（与发送帧地址顺序相反！）★★
 *
 *   字节  内容                           说明
 *   ────  ─────────────────────────      ──────────────────────────────────
 *   [0]   0xDF                           固定帧头
 *   [1]   Target_ID = 0x97               目标地址（=PC_ID，我们这一端）
 *   [2]   Robot_ID  = 0x01               源地址（小车 ID）
 *   [3]   A                              回传类（见下表）
 *   [4]   B                              子码
 *   [5]   LEN
 *   [6..LEN+5]   payload
 *   [LEN+6]      0xFD
 *   [LEN+7/8]    checksum_lo / hi
 *
 *
 * ★★ 本库会解析的 4 种回传 ★★
 *
 *   A      B        含义                                  小车端发起位置
 *   ────   ──────   ──────────────────────────────────    ─────────────────────────
 *   0x6C   0x80     Odom v4 全量帧 (51B payload)          DF_CoreState.c::DF_Send_odom
 *   0x6C   0x81     VelPos v2 简化帧 (35B payload)        DF_CoreState.c::DF_Send_vel_pose
 *   0x6F   *        运动完成回传 (ProgressBack, 2B)       DF_CoreFlag.c::DF_Flag_Back
 *                   payload = [Process(0xFF=完成), Notice]
 *   0x8C   0x40     激活/校准状态 (12B payload)           DF_CoreDate.c::DF_DcarParmState
 *                   ifIMUcali / ifFinetune / IMU 参数 / 控制 Par
 *   0x46   *        IMU 校准完成 (本库不主动用)           DF_CoreFlag.c
 *
 *
 * ★★ 坐标系约定（ROS REP-103）★★
 *
 *    +X = 前进 (forward)         g_odom.n_pos_x_m, b_vel_x_mps
 *    +Y = 左方 (left)            g_odom.n_pos_y_m, b_vel_y_mps
 *    +Yaw = CCW (逆时针)         g_odom.yaw_rad (单位 rad, 协议层 SI rad s32×10000)
 *
 *    与老协议相比：+X/+Y 含义对调 + Yaw 正负反转 + 单位 deg→rad。
 *****************************************************************************/
#include "DFCom.h"
#include "usart.h"
#include "delay.h"

/* ===========================================================================
 * 全局数据存储 —— 应用层直接读
 * ===========================================================================*/
volatile OdomData_t   g_odom         = {0};   /* Odom v4 (CMD 0x6C 0x80, 51B) */
volatile VelPosData_t g_velpos       = {0};   /* VelPos v2 (CMD 0x6C 0x81, 35B) */
volatile MoveStatus_t g_move_status[2] = {{0}};
volatile DcarState_t  g_dcar_state   = {0};

#define MOVE_SLOT_NONE  0xFF

/* 运动任务只保留“当前槽 + 旧槽丢弃器”这两个角色。 */
static volatile u8 g_move_current_slot    = MOVE_SLOT_NONE;
static volatile u8 g_move_discard_slot    = MOVE_SLOT_NONE;
static volatile u8 g_move_discard_cmd     = 0;
static volatile u8 g_move_session_active  = 0;
static volatile u8 g_move_send_failed     = 0;
static volatile u8 g_move_failed_cmd      = 0;

/* 本地 ms tick（由 TIM2 中断 +1，在 DFCom_Print.c 维护） */
extern volatile u32 g_local_tick_ms;

/* ===========================================================================
 * 小端字节合成（DF_Link 协议所有多字节字段都是小端）
 * ===========================================================================*/
static s16 read_s16(const u8 *p) {
    return (s16)((u16)p[0] | ((u16)p[1] << 8));
}

static s32 read_s32(const u8 *p) {
    return (s32)((u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24));
}

static u32 read_u32(const u8 *p) {
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static u8 move_is_bounded_cmd(u8 cmd_code)
{
    return (cmd_code >= CMD_ROT && cmd_code <= CMD_ARC);
}

static void move_clear_slot(u8 slot)
{
    if (slot > 1) return;
    g_move_status[slot].cmd_code    = 0;
    g_move_status[slot].progress    = 0;
    g_move_status[slot].last_notice = 0;
    g_move_status[slot].fresh       = 0;
}

static u8 move_slot_is_terminal(u8 slot)
{
    return (slot <= 1 &&
            g_move_status[slot].fresh &&
            g_move_status[slot].progress == 0xFF);
}

static void move_release_discard(void)
{
    if (g_move_discard_slot <= 1) {
        move_clear_slot(g_move_discard_slot);
    }
    g_move_discard_slot = MOVE_SLOT_NONE;
    g_move_discard_cmd  = 0;
}

/*
 * 当前任务离开前台时，已终止的槽直接清空；未终止的槽变成丢弃器，
 * 专门吞掉该任务随后唯一的终止帧。
 */
static void move_retire_current(void)
{
    u8 slot = g_move_current_slot;
    u8 cmd_code;

    if (slot > 1) return;
    cmd_code = g_move_status[slot].cmd_code;

    if (move_slot_is_terminal(slot)) {
        move_clear_slot(slot);
    } else {
        if (g_move_discard_slot <= 1 &&
            g_move_discard_slot != slot) {
            move_release_discard();
        }
        move_clear_slot(slot);
        g_move_discard_slot = slot;
        g_move_discard_cmd  = cmd_code;
    }
    g_move_current_slot = MOVE_SLOT_NONE;
}

static u8 move_next_slot(void)
{
    if (g_move_current_slot <= 1) return g_move_current_slot ^ 1U;
    if (g_move_discard_slot <= 1) return g_move_discard_slot ^ 1U;
    return 0;
}

static u32 move_enter_critical(void)
{
    u32 primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void move_exit_critical(u32 primask)
{
    if ((primask & 1U) == 0) __enable_irq();
}

static void move_reset_session_state(u8 active)
{
    move_clear_slot(0);
    move_clear_slot(1);
    g_move_current_slot   = MOVE_SLOT_NONE;
    g_move_discard_slot   = MOVE_SLOT_NONE;
    g_move_discard_cmd    = 0;
    g_move_send_failed    = 0;
    g_move_failed_cmd     = 0;
    g_move_session_active = active;
}

void DFCom_MoveSessionQuarantine(void)
{
    u32 primask = move_enter_critical();
    move_reset_session_state(0);
    move_exit_critical(primask);
}

void DFCom_MoveSessionStart(void)
{
    u32 primask = move_enter_critical();
    move_reset_session_state(1);
    move_exit_critical(primask);
}

/*
 * 只能在运动帧成功启动 DMA 后调用。
 * 有界任务按 A/B/A/B 交替；若目标槽还是 N-2 的旧丢弃槽，直接强制清空复用，
 * 因此丢过一次终止帧也不会让槽永久占用。
 */
void DFCom_MoveCommandSent(u8 cmd_code)
{
    u32 primask;
    u8 next;

    if (cmd_code != CMD_VEL && !move_is_bounded_cmd(cmd_code)) return;

    primask = move_enter_critical();
    g_move_send_failed = 0;
    g_move_failed_cmd  = 0;

    /* 上电停车/诊断阶段仍发送帧，但所有运动回传都处于隔离态。 */
    if (!g_move_session_active) {
        move_exit_critical(primask);
        return;
    }

    /*
     * 持续速度没有可等待的自然终止帧，但它会打断当前有界任务。
     * 所以只把旧当前槽降级为丢弃器，不为 CMD_VEL 建立新当前槽。
     */
    if (cmd_code == CMD_VEL) {
        move_retire_current();
        move_exit_critical(primask);
        return;
    }

    next = move_next_slot();

    /*
     * A→B→A：轮到一个槽再次成为当前槽时，无条件放弃它保存的 N-2 旧任务。
     * 这是“终止帧在客户端丢失”时的硬兜底，不让旧槽永久等待。
     */
    if (g_move_discard_slot == next) {
        move_release_discard();
    }

    move_retire_current();
    move_clear_slot(next);
    g_move_status[next].cmd_code = cmd_code;
    g_move_current_slot = next;
    move_exit_critical(primask);
}

void DFCom_MoveCommandSendFailed(u8 cmd_code)
{
    u32 primask;
    if (!move_is_bounded_cmd(cmd_code)) return;

    primask = move_enter_critical();
    if (g_move_session_active) {
        g_move_send_failed = 1;
        g_move_failed_cmd  = cmd_code;
    }
    move_exit_critical(primask);
}


/*============================================================================
 *  共享 scale 表 (跟小车端 DF_link.h 完全一致)
 *============================================================================*/
#define DFCOM_SCALE_ANGLE_RAD   10000.0f
#define DFCOM_SCALE_ACC_MPS2      400.0f
#define DFCOM_SCALE_GYRO_RAD     1800.0f
#define DFCOM_SCALE_VEL_MPS      5000.0f
#define DFCOM_SCALE_POS_M        1000.0f

/*============================================================================
 *  解析 1：Odom v4 数据帧 (A=0x6C, B=0x80, 51B payload)
 *============================================================================*/
static void parse_odom_v4(const u8 *payload, u8 payload_len)
{
    if (payload_len < 51) return;
    if (payload[0] != 0x04) return;  /* version 字节检查 */

    g_odom.roll_rad    = read_s16(&payload[1])  / DFCOM_SCALE_ANGLE_RAD;
    g_odom.pitch_rad   = read_s16(&payload[3])  / DFCOM_SCALE_ANGLE_RAD;
    g_odom.yaw_rad     = read_s16(&payload[5])  / DFCOM_SCALE_ANGLE_RAD;

    g_odom.acc_x_mps2  = read_s16(&payload[7])  / DFCOM_SCALE_ACC_MPS2;
    g_odom.acc_y_mps2  = read_s16(&payload[9])  / DFCOM_SCALE_ACC_MPS2;
    g_odom.acc_z_mps2  = read_s16(&payload[11]) / DFCOM_SCALE_ACC_MPS2;

    g_odom.gyro_x_rps  = read_s16(&payload[13]) / DFCOM_SCALE_GYRO_RAD;
    g_odom.gyro_y_rps  = read_s16(&payload[15]) / DFCOM_SCALE_GYRO_RAD;
    g_odom.gyro_z_rps  = read_s16(&payload[17]) / DFCOM_SCALE_GYRO_RAD;

    g_odom.b_vel_x_mps = read_s16(&payload[19]) / DFCOM_SCALE_VEL_MPS;
    g_odom.b_vel_y_mps = read_s16(&payload[21]) / DFCOM_SCALE_VEL_MPS;
    g_odom.b_vel_z_mps = read_s16(&payload[23]) / DFCOM_SCALE_VEL_MPS;

    g_odom.n_vel_x_mps = read_s16(&payload[25]) / DFCOM_SCALE_VEL_MPS;
    g_odom.n_vel_y_mps = read_s16(&payload[27]) / DFCOM_SCALE_VEL_MPS;
    g_odom.n_vel_z_mps = read_s16(&payload[29]) / DFCOM_SCALE_VEL_MPS;

    g_odom.n_pos_x_m   = read_s32(&payload[31]) / DFCOM_SCALE_POS_M;
    g_odom.n_pos_y_m   = read_s32(&payload[35]) / DFCOM_SCALE_POS_M;
    g_odom.n_pos_z_m   = read_s32(&payload[39]) / DFCOM_SCALE_POS_M;

    g_odom.car_time_int_s  = read_u32(&payload[43]);
    g_odom.car_time_dec_us = read_u32(&payload[47]);

    g_odom.local_tick_ms = g_local_tick_ms;
    g_odom.frame_count++;
    g_odom.fresh = 1;
}

/*============================================================================
 *  解析 2：VelPos v2 数据帧 (A=0x6C, B=0x81, 35B payload)
 *============================================================================*/
static void parse_velpos_v2(const u8 *payload, u8 payload_len)
{
    if (payload_len < 35) return;
    if (payload[0] != 0x02) return;  /* version 字节检查 */

    g_velpos.yaw_rad     = read_s16(&payload[1])  / DFCOM_SCALE_ANGLE_RAD;

    g_velpos.b_vel_x_mps = read_s16(&payload[3])  / DFCOM_SCALE_VEL_MPS;
    g_velpos.b_vel_y_mps = read_s16(&payload[5])  / DFCOM_SCALE_VEL_MPS;
    g_velpos.b_vel_z_mps = read_s16(&payload[7])  / DFCOM_SCALE_VEL_MPS;

    g_velpos.n_vel_x_mps = read_s16(&payload[9])  / DFCOM_SCALE_VEL_MPS;
    g_velpos.n_vel_y_mps = read_s16(&payload[11]) / DFCOM_SCALE_VEL_MPS;
    g_velpos.n_vel_z_mps = read_s16(&payload[13]) / DFCOM_SCALE_VEL_MPS;

    g_velpos.n_pos_x_m   = read_s32(&payload[15]) / DFCOM_SCALE_POS_M;
    g_velpos.n_pos_y_m   = read_s32(&payload[19]) / DFCOM_SCALE_POS_M;
    g_velpos.n_pos_z_m   = read_s32(&payload[23]) / DFCOM_SCALE_POS_M;

    g_velpos.car_time_int_s  = read_u32(&payload[27]);
    g_velpos.car_time_dec_us = read_u32(&payload[31]);

    g_velpos.local_tick_ms = g_local_tick_ms;
    g_velpos.frame_count++;
    g_velpos.fresh = 1;
}


/*============================================================================
 *  解析 2：运动完成回传 ProgressBack (A=0x6F, B=cmd)
 *  ──────────────────────────────────────────────────────────────────────────
 *  小车在执行运动指令时定期回传进度，到位/被打断时也回传"完成"。
 *  payload 2 字节：
 *
 *    [0] Process  u8   0..0xFE = 进度比例（×255/100），0xFF = 完成
 *    [1] Notice   u8   0       = 正常完成
 *                      1       = 版本不支持或参数非法，任务未启动
 *                      0x0A    = 遥控器接管
 *                      0x62~66 = 被对应的新运动指令打断
 *
 *  B 字段 = 原指令的 cmd 码：
 *    0x62 = CMD_VEL          (持续速度无回传, 一般不会到这)
 *    0x63 = CMD_ROT          (原地旋转)
 *    0x64 = CMD_LINEAR       (直线位移)
 *    0x65 = CMD_LINEAR_WITH_YAW (无头位移)
 *    0x66 = CMD_ARC          (圆弧)
 *
 *  A/B 两槽绑定的是“本地发送次序”，不是命令类型。旧槽只吞帧，不保存状态；
 *  当前槽才会写入进度。这样连续两条相同 B 的指令也不会共用状态槽。
 *============================================================================*/
static void parse_move_progress(u8 cmd_code, const u8 *payload, u8 payload_len)
{
    if (payload_len < 2) return;
    if (!g_move_session_active) return;
    if (!move_is_bounded_cmd(cmd_code)) return;

    /*
     * 同类型连续指令时，DCar 回传里没有本地槽号。按协议顺序，上一任务会先
     * 回唯一的 0xFF 终止帧；旧槽在此之前吞掉同 B 的全部帧，见到 0xFF 后释放。
     * 若这唯一终止帧真的在客户端丢失，无法证明身份的同 B 帧仍按旧帧丢弃：
     * 最坏结果是当前 Wait 等满上限，而不是错误提前完成并跳过动作。
     */
    if (g_move_discard_slot <= 1 && cmd_code == g_move_discard_cmd) {
        if (payload[0] == 0xFF) {
            move_release_discard();
        }
        return;
    }

    if (g_move_current_slot <= 1 &&
        g_move_status[g_move_current_slot].cmd_code == cmd_code) {
        g_move_status[g_move_current_slot].progress    = payload[0];
        g_move_status[g_move_current_slot].last_notice = payload[1];
        g_move_status[g_move_current_slot].fresh       = 1;
    }
}


/*============================================================================
 *  解析 3：激活/校准状态 DcarParmState (A=0x8C, B=0x40)
 *  ──────────────────────────────────────────────────────────────────────────
 *  Cmd_Query_DcarState() 触发；小车端 DF_DcarParmState() 返回。
 *  payload 12 字节：
 *
 *    [0]      ifIMUcali   u8   ★ 1 = IMU 已校准（小车可正常工作）★
 *                              0 = 未校准（需要先用上位机校准/激活！）
 *                              判定标准：flash.acc_offset.z ∈ (1, 8000)
 *    [1]      ifFinetune  u8   1 = 控制参数已微调
 *    [2..3]   gyro_z_off  s16  Gyro Z 偏置 × 100
 *    [4..5]   imu_par2    s16  IMU 参数 2 × 10000
 *    [6..7]   imu_temp    s16  IMU 温度参数
 *    [8..9]   ctrl_par1   s16  控制 Par1 × 100
 *    [10..11] ctrl_par2   s16  控制 Par2 × 100
 *
 *  → 解析到 g_dcar_state；启动时 main 用它打印诊断
 *============================================================================*/
static void parse_dcar_state(const u8 *payload, u8 payload_len)
{
    if (payload_len < 12) return;
    g_dcar_state.imu_calibrated    = payload[0];
    g_dcar_state.is_finetune       = payload[1];
    g_dcar_state.imu_offset_z_x100 = read_s16(&payload[2]);
    g_dcar_state.imu_param2_x10000 = read_s16(&payload[4]);
    g_dcar_state.imu_temp_x10000   = read_s16(&payload[6]);
    g_dcar_state.ctrl_par1_x100    = read_s16(&payload[8]);
    g_dcar_state.ctrl_par2_x100    = read_s16(&payload[10]);
    g_dcar_state.received = 1;
}


/*============================================================================
 *  协议解析入口 DFCom_RxParse
 *  ──────────────────────────────────────────────────────────────────────────
 *  USART1 IDLE 中断（usart.c::USART1_IRQHandler）每收到一组数据后调用此函数。
 *
 *  算法：
 *    1. 在 data[] 里找 0xDF 帧头
 *    2. 校验 [1]=PC_ID, [2]=Robot_ID (回传帧地址顺序)
 *    3. 读 LEN，计算 tail 位置，检查 0xFD
 *    4. 累加校验和，与帧末 u16 LE 比较
 *    5. 按 A 字段分发到对应 parse_* 函数
 *    6. 跳到下一帧（一次中断可能收到多帧粘连，循环处理）
 *============================================================================*/
void DFCom_RxParse(u8 *data, u16 size)
{
    /* ★ size 必须 u16: buffer 现在是 256B (USART1_RX_BUFF_SIZE), u8 容不下 */
    u16 pos = 0;
    u8 iter = 0;
    const u8 MAX_ITER = 20;   /* 防止恶意数据导致死循环 */

    while (pos < size && iter < MAX_ITER) {
        u16 hdr;
        u8 A, B, len;
        u16 tail_pos;
        u16 sc;
        u16 sc_recv;
        u8 sum_len;
        u8 i;
        const u8 *payload;

        iter++;

        /* 1. 找帧头 0xDF */
        hdr = pos;
        while (hdr < size && data[hdr] != DFCOM_FRAME_HEADER) hdr++;
        if (hdr >= size) break;

        /* 2. 至少要 9 字节才可能是完整帧（DF + 5 字段头 + FD + 2 校验） */
        if (hdr + 8 >= size) break;

        /* 3. 回传帧地址：[1]=Target_ID(PC_ID), [2]=Robot_ID */
        if (data[hdr + 1] != DFCOM_PC_ID || data[hdr + 2] != DFCOM_ROBOT_ID) {
            pos = hdr + 1;  /* 不是给我们的帧，跳过 */
            continue;
        }

        A   = data[hdr + 3];
        B   = data[hdr + 4];
        len = data[hdr + 5];

        /* 4. 长度合理性 */
        if (len > 60) { pos = hdr + 1; continue; }

        /* 5. 边界 + 帧尾 */
        tail_pos = hdr + 6 + len;
        if (tail_pos + 2 >= size) break;   /* 帧不完整，等下次中断 */
        if (data[tail_pos] != DFCOM_FRAME_TAIL) { pos = hdr + 1; continue; }

        /* 6. 校验和（从帧头到帧尾全部累加） */
        sc = 0;
        sum_len = len + 7;
        for (i = 0; i < sum_len; i++) sc += data[hdr + i];
        sc_recv = (u16)data[tail_pos + 1] | ((u16)data[tail_pos + 2] << 8);
        if (sc != sc_recv) { pos = hdr + 1; continue; }

        /* 7. 校验通过 → 按 A 分发 */
        payload = &data[hdr + 6];
        switch (A) {
            case 0x6C:   /* 周期数据回传 */
                if (B == 0x80) parse_odom_v4(payload, len);         /* Odom v4 全量 51B */
                else if (B == 0x81) parse_velpos_v2(payload, len);  /* VelPos v2 简化 35B */
                break;

            case 0x6F:   /* 运动完成/进度回传 */
                parse_move_progress(B, payload, len);
                break;

            case 0x8C:   /* 状态查询回传 */
                if (B == 0x40) parse_dcar_state(payload, len);
                break;

            case 0x46:   /* IMU 校准回传 —— 本库不用 */
                break;

            default:
                /* 其他类型（电池/版本/参数查询等）暂不解析 */
                break;
        }

        /* 8. 跳到下一帧 */
        pos = tail_pos + 3;
    }
}


/*============================================================================
 *  可选辅助：阻塞等待运动完成
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ 必读 ★ —— 第一个参数 (cmd_code) 必须跟你刚发的指令配对!
 *
 *  发的函数                     →  WaitMoveDone 第一参填什么
 *  -----------------------------    ---------------------------------
 *  Cmd_Move_Linear(...)         →  CMD_LINEAR           (0x64)
 *  Cmd_Move_LinearWithYaw(...)  →  CMD_LINEAR_WITH_YAW  (0x65)
 *  Cmd_Move_Rot(...)            →  CMD_ROT              (0x63)
 *  Cmd_Move_Arc(...)            →  CMD_ARC              (0x66)
 *  Cmd_Move_Vel(...)            →  ★ 不要用 WaitMoveDone ★
 *                                  (持续速度没有完成回传, 等不到)
 *
 *  填错不会去读别的任务槽，而是立即返回 MOVE_WAIT_TIMEOUT。
 *
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ 第二个参数 (timeout_ms) ★
 *
 *  timeout_ms == 0  → 永久等待；链路失联时也不会自行返回。
 *
 *  timeout_ms > 0   → 这条任务允许执行的【最长时间】。
 *                     若 DCar 提前完成，收到本任务 FF/00 后立即返回 DONE；
 *                     到达时限仍未完成，立即返回 TIMEOUT。调用者随后发送的
 *                     下一条运动命令会按协议合法打断上一条。
 *
 *  A/B 槽如何防止旧终止帧误伤下一条：
 *    - 相邻任务强制使用不同槽；
 *    - 超时任务的槽降级为“旧槽丢弃器”，只吞帧，不保存状态；
 *    - 旧任务唯一的 0xFF 终止帧只负责释放旧槽，不会完成新任务；
 *    - 到 N+2 轮回时无条件清空复用，即使客户端丢过旧终止帧也不永久占槽。
 *
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ 返回值 ★
 *
 *  返回 MOVE_WAIT_DONE        → 自然完成 (progress=0xFF + notice=0x00)
 *  返回 MOVE_WAIT_TIMEOUT     → 超时、发送失败或未绑定到最近任务
 *  返回 MOVE_WAIT_INTERRUPTED → 被打断 (progress=0xFF + notice!=0x00)
 *
 *  progress=0xFF 只是“任务终止”，notice 才决定结果：
 *    last_notice=0x00 → 自然完成 (车真的到位了)
 *    last_notice=0x01 → 任务被拒绝 (版本不支持或参数非法)
 *    last_notice=0x0A → 被遥控器接管
 *    last_notice=0x62~0x66 → 被对应的新运动指令打断
 *
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ 用法示例 ★
 *
 *    Cmd_Move_Linear(50, 0, 30, 2);       // CM 模式：前进 50cm
 *    WaitMoveDone(CMD_LINEAR, 1000);       // 最多跑 1s；提前完成就提前返回
 *    Cmd_Move_Linear(-50, 0, 30, 2);      // 到点后打断上一条，或自然接续
 *    WaitMoveDone(CMD_LINEAR, 1000);
 *============================================================================*/
u8 WaitMoveDone(u8 cmd_code, u32 timeout_ms)
{
    u8 slot;
    u8 notice;
    u8 send_failed;
    u32 primask;
    u32 start;

    /*
     * 发送成功时已经完成切槽。这里只拍下当前槽，不清任何状态；
     * 因而零距离/参数拒绝等“比 Wait 入口更快”的终止帧也不会被抹掉。
     */
    primask = move_enter_critical();
    send_failed = (g_move_send_failed && g_move_failed_cmd == cmd_code);
    if (g_move_current_slot <= 1 &&
        g_move_status[g_move_current_slot].cmd_code == cmd_code) {
        slot = g_move_current_slot;
    } else {
        slot = MOVE_SLOT_NONE;
    }
    move_exit_critical(primask);

    start = g_local_tick_ms;
    printf("[WAIT.IN]  cmd=0x%02X slot=%u tmo=%lu start_tick=%lu\r\n",
           (unsigned)cmd_code, (unsigned)slot,
           (unsigned long)timeout_ms, (unsigned long)start);

    if (send_failed) {
        printf("[WAIT.TX_FAIL] cmd=0x%02X (DMA frame was not started)\r\n",
               (unsigned)cmd_code);
        return MOVE_WAIT_TIMEOUT;
    }

    if (slot == MOVE_SLOT_NONE) {
        printf("[WAIT.NO_TASK] cmd=0x%02X (not the latest bounded command)\r\n",
               (unsigned)cmd_code);
        return MOVE_WAIT_TIMEOUT;
    }

    while (1) {
        /* 终止帧统一由 progress=0xFF 标识，notice 决定自然完成或被打断。 */
        if (g_move_status[slot].fresh &&
            g_move_status[slot].progress == 0xFF) {
            notice = g_move_status[slot].last_notice;
            if (notice == MOVE_NOTICE_NONE) {
                printf("[WAIT.DONE] cmd=0x%02X waited=%lums\r\n",
                       (unsigned)cmd_code,
                       (unsigned long)(g_local_tick_ms - start));
                return MOVE_WAIT_DONE;
            }

            printf("[WAIT.INTERRUPTED] cmd=0x%02X notice=0x%02X waited=%lums\r\n",
                   (unsigned)cmd_code,
                   (unsigned)notice,
                   (unsigned long)(g_local_tick_ms - start));
            return MOVE_WAIT_INTERRUPTED;
        }
        if (timeout_ms != 0) {                /* timeout_ms == 0 ⇒ 永久等待 */
            if ((g_local_tick_ms - start) >= timeout_ms) {
                printf("[WAIT.TIMEOUT] cmd=0x%02X waited=%lums now_tick=%lu (没等到 progress=FF)\r\n",
                       (unsigned)cmd_code,
                       (unsigned long)(g_local_tick_ms - start),
                       (unsigned long)g_local_tick_ms);
                return MOVE_WAIT_TIMEOUT;
            }
        }
        /*===== 低功耗优化建议 (高级, 当前未启用) ======================================
         * 下面这行 delay_ms(10) 是 SysTick 忙等待, 等待期间 CPU 一直在自旋,
         * 浪费功耗 + 占住 CPU 不能干别的事。
         *
         * 如果你想让 CPU 在等待期间睡眠 (省电 + 让出 CPU 给其他任务), 把
         *      delay_ms(10);
         * 替换为下面这一行:
         *      __WFI();    // Wait For Interrupt, ARM Cortex-M 标准指令
         *
         * 原理: __WFI() 让 CPU 立刻进入睡眠, 任何中断 (SysTick 1ms / USART RX)
         *       都会自动唤醒。 这样 CPU 占用从 ~100% 降到 ~0.1%, 行为完全一致。
         *
         * 注意: 进入 __WFI 前必须确保 SysTick + USART RX 中断已启用 (默认就是这样,
         *       不用改); 不要在 __disable_irq() 之后调 __WFI, 否则永远醒不来。
         *
         * 我们当前保留 delay_ms(10) 是因为这版例程优先考虑简单易懂, 用户可按需切换。
         *===========================================================================*/
        delay_ms(10);
    }
}

u8 GetMoveProgress(u8 cmd_code)
{
    u8 slot = g_move_current_slot;
    if (slot <= 1 && g_move_status[slot].cmd_code == cmd_code) {
        return g_move_status[slot].progress;
    }
    return 0;
}

u8 GetMoveNotice(u8 cmd_code)
{
    u8 slot = g_move_current_slot;
    if (slot <= 1 && g_move_status[slot].cmd_code == cmd_code) {
        return g_move_status[slot].last_notice;
    }
    return 0;
}

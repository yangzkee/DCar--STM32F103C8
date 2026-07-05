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
 *   0x6C   0x80     IMU_TIME 数据帧 (36B payload)         DF_CoreState.c::DF_IMU_TIME
 *                   含 Acc/Gyro/Yaw(rad s32)/B_Vel/N_Pos/Time
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
volatile OdomData_t   g_odom         = {0};
volatile MoveStatus_t g_move_status[8] = {{0}};
volatile DcarState_t  g_dcar_state   = {0};

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


/*============================================================================
 *  解析 1：IMU_TIME 数据帧 (A=0x6C, B=0x80)
 *  ──────────────────────────────────────────────────────────────────────────
 *  payload 36 字节，布局如下（小车端编码见 DF_CoreState.c::DF_IMU_TIME）：
 *
 *    偏移      字段      类型     缩放     含义 (ROS REP-103: +X 前, +Y 左)
 *    ────      ───────   ──────  ──────   ────────────────────────────────
 *    [0..1]    Acc_X     s16 LE   ×100    加速度 +X 前 (m/s²)
 *    [2..3]    Acc_Y     s16 LE   ×100    加速度 +Y 左
 *    [4..5]    Acc_Z     s16 LE   ×100    加速度 +Z 上
 *    [6..7]    Gyro_X    s16 LE   ×100    角速度 X (rad/s)
 *    [8..9]    Gyro_Y    s16 LE   ×100    角速度 Y
 *    [10..11]  Gyro_Z    s16 LE   ×100    角速度 Z, CCW+
 *    [12..15]  Yaw       s32 LE   ×10000  ★ Yaw (弧度 rad, 连续累计, CCW+) ★
 *                                          ↑ 协议层 SI rad, 不再 deg!
 *                                          ↑ 分辨率 1/10000 rad ≈ 0.0057°
 *    [16..17]  B_VelX    s16 LE   ×100    车体系 Vx +前 (m/s)
 *    [18..19]  B_VelY    s16 LE   ×100    车体系 Vy +左 (m/s)
 *    [20..23]  N_PosX    s32 LE   ×10000  ★ 世界系 +X 前进 位置 (米) ★
 *    [24..27]  N_PosY    s32 LE   ×10000  ★ 世界系 +Y 左方 位置 (米) ★
 *    [28..31]  Time_Int  u32 LE   --      小车端时间戳（秒部分）
 *    [32..35]  Time_Dec  u32 LE   --      小车端时间戳（微秒部分）
 *
 *  → 解析到 g_odom；frame_count 自增以便诊断"数据是否在流动"
 *============================================================================*/
static void parse_imu_time(const u8 *payload, u8 payload_len)
{
    if (payload_len < 36) return;  /* 长度不对直接丢 (新协议 36B, 旧 34B 兼容性已断) */

    g_odom.acc_x_mps2  = read_s16(&payload[0])  / 100.0f;
    g_odom.acc_y_mps2  = read_s16(&payload[2])  / 100.0f;
    g_odom.acc_z_mps2  = read_s16(&payload[4])  / 100.0f;
    g_odom.gyro_x_rps  = read_s16(&payload[6])  / 100.0f;
    g_odom.gyro_y_rps  = read_s16(&payload[8])  / 100.0f;
    g_odom.gyro_z_rps  = read_s16(&payload[10]) / 100.0f;

    /* Yaw: rad s32×10000 (4B), 原来是 deg s16×100 (2B) */
    g_odom.yaw_rad     = read_s32(&payload[12]) / 10000.0f;
    g_odom.b_vel_x_mps = read_s16(&payload[16]) / 100.0f;
    g_odom.b_vel_y_mps = read_s16(&payload[18]) / 100.0f;

    g_odom.n_pos_x_m   = read_s32(&payload[20]) / 10000.0f;
    g_odom.n_pos_y_m   = read_s32(&payload[24]) / 10000.0f;

    g_odom.car_time_int_s  = read_u32(&payload[28]);
    g_odom.car_time_dec_us = read_u32(&payload[32]);

    g_odom.local_tick_ms = g_local_tick_ms;
    g_odom.frame_count++;
    g_odom.fresh = 1;
}


/*============================================================================
 *  解析 2：运动完成回传 ProgressBack (A=0x6F, B=cmd)
 *  ──────────────────────────────────────────────────────────────────────────
 *  小车在执行运动指令时定期回传进度，到位/被打断时也回传"完成"。
 *  payload 2 字节：
 *
 *    [0] Process  u8   0..0xFE = 进度比例（×255/100），0xFF = 完成
 *    [1] Notice   u8   0       = 正常完成
 *                      其他    = 被打断（值 = 中断 type，比如新指令的 cmd）
 *
 *  B 字段 = 原指令的 cmd 码：
 *    0x62 = CMD_VEL          (持续速度无回传, 一般不会到这)
 *    0x63 = CMD_ROT          (原地旋转)
 *    0x64 = CMD_LINEAR       (直线位移)
 *    0x65 = CMD_LINEAR_WITH_YAW (无头位移)
 *    0x66 = CMD_ARC          (圆弧)
 *
 *  应用层用 WaitMoveDone(cmd) 等到 g_move_status[idx].progress == 0xFF
 *
 *  g_move_status 用 (cmd & 0x07) 索引: 0x62..0x66 → 2..6, 都不冲突。
 *============================================================================*/
static void parse_move_progress(u8 cmd_code, const u8 *payload, u8 payload_len)
{
    u8 idx;
    if (payload_len < 2) return;
    idx = cmd_code & 0x07;
    g_move_status[idx].progress    = payload[0];
    g_move_status[idx].last_notice = payload[1];
    g_move_status[idx].fresh       = 1;
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
                if (B == 0x80) parse_imu_time(payload, len);
                /* 其他 0x6C 子码（B_Vel/N_Vel/B_Pos/N_Pos 等）信息全在 IMU_TIME 里
                 * 都有了，不重复解析 */
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
 *  Cmd_Move_Rotate(...)         →  CMD_ROT              (0x63)
 *  Cmd_Move_Arc(...)            →  CMD_ARC              (0x66)
 *  Cmd_Move_Velocity(...)       →  ★ 不要用 WaitMoveDone ★
 *                                  (持续速度没有完成回传, 等不到)
 *
 *  填错就等错槽位 (g_move_status[] 是按 cmd&0x07 索引的), 永远等不到
 *  → 主程序会卡死在这一行。
 *
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ 第二个参数 (timeout_ms) ★
 *
 *  timeout_ms == 0  → ★★★ 永久等待 (推荐!) ★★★
 *                     不设超时, 一直等到 MCU 真正回传 0xFF/0x00 (自然完成)
 *                     这是最安全的用法, 杜绝 "雪崩吃单" bug
 *                     代价: 万一小车失联/卡死, 这一行也卡死, 不会自己往下走
 *
 *  timeout_ms > 0   → 设了超时
 *                     ⚠️ 注意: 超时时间必须 > 小车实际跑这段的物理时间!
 *                     ⚠️ 例如 0.5m @ 0.3m/s 梯形加减速 ≈ 5.5 秒, 超时设 5000ms
 *                        就会刚好踩在临界, 触发 "雪崩":
 *                          客户端超时 return 0 → 立刻发下一条 →
 *                          MCU 端 Motion_* 被新指令打断, 立刻回 0xFF/0x64 →
 *                          下一次 WaitMoveDone 入口清零和打断回传时序竞争 →
 *                          客户端被骗 "完成", 又发下一条 →
 *                          雪崩, 每条只跑几毫秒就被打断, 一秒疯发 3~5 条!
 *                     ⚠️ 安全的超时值: 物理时间 × 2~3 倍, 留足余量。
 *                        或者干脆传 0, 别用超时。
 *
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ 返回值 ★
 *
 *  返回 1  →  ★ 自然完成 ★ (progress=0xFF + last_notice=0x00)
 *  返回 0  →  超时 (timeout_ms>0 时才会发生)
 *
 *  ⚠️ 重要: 这一版修过了 "被打断也算完成" 的 bug
 *  从前 progress=0xFF 就直接 return 1, 但 0xFF 有两个含义:
 *    last_notice=0x00 → 自然完成 (车真的到位了)
 *    last_notice=非零 → 被打断 (新指令进来 / RC 接管 / 撞到东西)
 *  老代码不区分, 把 "被打断" 当 "完成", 直接造成雪崩。
 *  这一版加上 last_notice==0 判定后, 只有真正自然完成才 return 1。
 *
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ 用法示例 ★
 *
 *    Cmd_Move_Linear(0.5f, 0, 0.3f);    // SI 模式: 前进 0.5m, 速度 0.3m/s
 *    WaitMoveDone(CMD_LINEAR, 0);       // 永久等, 推荐
 *
 *    Cmd_Move_Arc(0.3f, 1.5708f, 0.2f); // 半径 0.3m, +90°, 0.2m/s
 *    WaitMoveDone(CMD_ARC, 0);          // 永久等
 *
 *    Cmd_Move_Rotate(3.14159f, 1.0f);   // 转 180°, 角速度 1 rad/s
 *    WaitMoveDone(CMD_ROT, 0);
 *
 *  示例 (带超时, 必须保守):
 *    Cmd_Move_Linear(0.5f, 0, 0.3f);
 *    if (WaitMoveDone(CMD_LINEAR, 15000) == 0) {     // 15s 兜底
 *        printf("timeout, sending stop\r\n");
 *        Cmd_Move_Velocity(0, 0, 0);                  // 主动停车
 *        delay_ms(1000);                              // 让 MCU 喘口气
 *    }
 *============================================================================*/
u8 WaitMoveDone(u8 cmd_code, u32 timeout_ms)
{
    u8 idx = cmd_code & 0x07;
    u32 start;
    /* 入口清掉对应槽位的残留, 防止上一轮回传被误读 */
    g_move_status[idx].progress    = 0;
    g_move_status[idx].last_notice = 0;
    g_move_status[idx].fresh       = 0;

    start = g_local_tick_ms;
    /* ★★★ 诊断 printf (2026-05-26 加, 定位 "1 秒超时不退出" 问题) ★★★
     * 跑一次抓 USART2 log, 看 IN / DONE / TIMEOUT 三种事件能不能配对。
     * 验证完根因后这三行可以删掉。 */
    printf("[WAIT.IN]  cmd=0x%02X tmo=%lu start_tick=%lu\r\n",
           (unsigned)cmd_code, (unsigned long)timeout_ms, (unsigned long)start);

    while (1) {
        /* ★ 关键修复 ★
         *   原来只看 progress==0xFF, 把 "被打断的完成" (notice!=0)
         *   也误判为 "自然完成", 是雪崩的根因。
         *   现在必须 progress==0xFF AND last_notice==0 才算真正完成。
         *   被打断时 (notice!=0) 继续等 → 客户端不会被骗着疯发下一条。
         */
        if (g_move_status[idx].fresh &&
            g_move_status[idx].progress    == 0xFF &&
            g_move_status[idx].last_notice == 0x00) {
            printf("[WAIT.DONE] cmd=0x%02X waited=%lums\r\n",
                   (unsigned)cmd_code,
                   (unsigned long)(g_local_tick_ms - start));
            return 1;   /* ✓ 自然完成 */
        }
        if (timeout_ms != 0) {                /* timeout_ms == 0 ⇒ 永久等待 */
            if ((g_local_tick_ms - start) > timeout_ms) {
                printf("[WAIT.TIMEOUT] cmd=0x%02X waited=%lums now_tick=%lu (没等到 progress=FF/notice=00)\r\n",
                       (unsigned)cmd_code,
                       (unsigned long)(g_local_tick_ms - start),
                       (unsigned long)g_local_tick_ms);
                return 0;
            }
        }
        delay_ms(10);
    }
}

u8 GetMoveProgress(u8 cmd_code)
{
    u8 idx = cmd_code & 0x07;
    return g_move_status[idx].progress;
}

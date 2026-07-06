/*****************************************************************************
 * DFCom_Tx.c - 发送侧实现（SI 单位 / ROS REP-103 坐标系）
 * ---------------------------------------------------------------------------
 * 这个文件负责把"用户调用的 Cmd_Move_* 函数"打包成 DcarON DF_Link 协议帧，
 * 通过 USART1 DMA 发送到小车。
 *
 * 所有 Cmd_* 函数发完立即 return —— 不阻塞、不等回传。
 * 要等回传请用 WaitMoveDone() 辅助函数（在 DFCom_Rx.c 提供）。
 *
 *
 *  ╔════════════════════════════════════════════════════════════════════╗
 *  ║                                                                    ║
 *  ║   ★ 协议版本：SI + ROS REP-103（重要！）★                         ║
 *  ║                                                                    ║
 *  ║   所有数值字段都是 SI 单位，与老版本"工程量"完全不兼容。           ║
 *  ║                                                                    ║
 *  ║   单位:                                                            ║
 *  ║     位置 px/py        → 米 (m)                                     ║
 *  ║     速度 speed/vx/vy  → 米/秒 (m/s)                                ║
 *  ║     角度 dyaw         → 弧度 (rad)        ★ 不是 度!              ║
 *  ║     角速度 omega/vz   → 弧度/秒 (rad/s)                            ║
 *  ║                                                                    ║
 *  ║   缩放（所有 SI 值通过 ×10000 编码为 int32 LE 上线）：             ║
 *  ║     0.5 米    → 5000   (s32)                                       ║
 *  ║     0.3 m/s   → 3000   (s32)                                       ║
 *  ║     M_PI/2    → 15707  (s32) ≈ 1.5708 rad ≈ 90°                   ║
 *  ║                                                                    ║
 *  ║   坐标系（ROS REP-103）:                                           ║
 *  ║     +X = 前方 (forward)                                            ║
 *  ║     +Y = 左方 (left)                                               ║
 *  ║     +Yaw = CCW (逆时针, 从上往下看)                                ║
 *  ║                                                                    ║
 *  ║   速度范围参考:                                                    ║
 *  ║     典型使用 0.1 ~ 0.5 m/s                                         ║
 *  ║     omega_max 推荐 0.5 ~ 2.0 rad/s                                ║
 *  ║                                                                    ║
 *  ╚════════════════════════════════════════════════════════════════════╝
 *****************************************************************************/
#include "DFCom.h"
#include "usart.h"
#include "delay.h"

/* 字节拆分宏（标准库 BYTE0/1/2/3） */
#define BYTE0(v) ((u8)((v) & 0xFF))
#define BYTE1(v) ((u8)(((v) >> 8) & 0xFF))
#define BYTE2(v) ((u8)(((v) >> 16) & 0xFF))
#define BYTE3(v) ((u8)(((v) >> 24) & 0xFF))

/* 缩放因子（新协议：所有 SI 量都 ×10000 编 s32 LE）
 *   位置 / 速度 / 角度 / 角速度全用这一个缩放                      */
#define SCALE_SI     10000   /* SI 单位 → s32 (÷10000 还原) */

/* ===========================================================================
 *  单位模式全局变量（参见 DFCom.h 顶部 DFCom_UnitMode_e）
 *  ---------------------------------------------------------------------------
 *  默认 = DFCOM_UNIT_CM（厘米 + 度），适合学生 / 初学者。
 *  想用 SI 原生单位（米 + 弧度），在 main.c 把它设成 DFCOM_UNIT_M。
 *  本文件下面 5 个 Cmd_Move_* 函数顶部会按本变量做单位转换，
 *  协议层永远只发 SI（米 / rad / ×10000 s32）。
 * ===========================================================================*/
u8 g_dfcom_unit_mode = DFCOM_UNIT_CM;   /* 默认厘米模式 */

/*============================================================================
 *  通用打帧工具：build_header + finalize_frame + send_frame
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ DF_Link 发送帧总格式 ★
 *
 *   字节  内容                           说明
 *   ────  ────────────────────────────   ────────────────────────────────────
 *   [0]   0xDF                           固定帧头
 *   [1]   ROBOT_ID = 0x01                小车 ID（默认 1，可在 flash 改）
 *   [2]   PC_ID = 0x97                   上位机 ID（小车端不校验此字节）
 *   [3]   A                              指令类 (class)
 *   [4]   B                              子指令 (cmd)
 *   [5]   LEN                            payload 长度（不含头/尾/校验）
 *   [6..LEN+5]   payload                 数据域，字段顺序见每个指令注释
 *   [LEN+6]      0xFD                    固定帧尾
 *   [LEN+7/8]    checksum_lo / hi        u16 LE，从 [0] 累加到 [LEN+6]
 *============================================================================*/

static u8 build_header(u8 *buf, u8 A, u8 B)
{
    buf[0] = DFCOM_FRAME_HEADER;
    buf[1] = DFCOM_ROBOT_ID;
    buf[2] = DFCOM_PC_ID;
    buf[3] = A;
    buf[4] = B;
    /* buf[5] = LEN 由调用方填 */
    return 5;
}

static u8 finalize_frame(u8 *buf, u8 data_len)
{
    u8 cnt = 6 + data_len;
    u16 sc;
    u8 sum_len;
    u8 i;
    buf[cnt++] = DFCOM_FRAME_TAIL;
    /* 校验和：从帧头到帧尾全部累加 */
    sc = 0;
    sum_len = data_len + 7;   /* header 6 + tail 1 */
    for (i = 0; i < sum_len; i++) sc += buf[i];
    buf[cnt++] = BYTE0(sc);
    buf[cnt++] = BYTE1(sc);
    return cnt;
}

static void send_frame(u8 *buf, u8 len)
{
    /* DMA 发送，最多重试 3 次（上一帧 DMA 还在跑时立刻发会失败） */
    u8 retry = 0;
    while (USART1_Send_By_DMA(buf, len) != 0 && retry < 3) {
        retry++;
        delay_us(200);
    }
}

/*----------------------------------------------------------------------------
 * 发送 Motion 指令前清掉对应 cmd 在 g_move_status 里的残留状态
 * ---------------------------------------------------------------------------
 * 为什么要清:
 *   WaitMoveDone 通过 g_move_status[cmd & 0x07] 判断完成。
 *   如果上一条 Motion_* 被打断, MCU 会发 progress=0xFF / notice!=0 回传,
 *   这条回传会停留在 g_move_status 里。
 *   下一条 Cmd_Move_* 发出后, 如果不清零, 客户端可能在 WaitMoveDone
 *   入口清零和新回传到达之间出现时序竞争, 把上一条的残留误读成本条完成。
 *   所以发送侧主动清零, 双保险。
 *
 * 只有有完成回传的命令需要清 (LINEAR / LINEAR_WITH_YAW / ROT / ARC)。
 * VEL (持续速度) 没有完成回传, 不需要清。
 *--------------------------------------------------------------------------*/
static void clear_move_slot(u8 cmd_code)
{
    u8 idx = cmd_code & 0x07;
    g_move_status[idx].progress    = 0;
    g_move_status[idx].last_notice = 0;
    g_move_status[idx].fresh       = 0;
}

/* 把 4 个字节小端写入 buf[off..off+3]，给 s32 LE 用 (协议统一整数编码哲学) */
static void put_s32_le(u8 *buf, s32 v)
{
    buf[0] = BYTE0(v);
    buf[1] = BYTE1(v);
    buf[2] = BYTE2(v);
    buf[3] = BYTE3(v);
}


/*============================================================================
 *  指令 1：直线位移 Cmd_Move_Linear  (推荐：直线移动用这个)
 *  ──────────────────────────────────────────────────────────────────────────
 *  Class/Cmd: A = 0x02 (DataRecivControClass), B = 0x64 (Motion_Linear)
 *
 *  ★ 新协议合并：把老的 Cmd_Move_Pos / Cmd_Move_VelPos 合二为一 ★
 *    小车端默认走 TRAPEZOID（梯形加减速）规划。原老版本 0x65 已停用。
 *
 *  参数（按 g_dfcom_unit_mode 解释）：
 *    CM 模式 (默认): px/py/speed 单位 = 厘米 / 厘米 / 厘米每秒
 *    M  模式:         px/py/speed 单位 = 米   / 米   / 米每秒 (SI)
 *
 *  参数语义：
 *    px:     +X 方向（前方）目标位移
 *    py:     +Y 方向（左方）目标位移
 *    speed:  最大速度
 *
 *  示例 (CM 模式, 默认):
 *    Cmd_Move_Linear(50, 0,   30)   →  前进 50 cm, 峰值 30 cm/s (~0.3 m/s)
 *    Cmd_Move_Linear(0,  50,  30)   →  左移 50 cm
 *    Cmd_Move_Linear(30, 30,  20)   →  斜向 (前左各 30 cm), 20 cm/s
 *
 *  示例 (M 模式):
 *    Cmd_Move_Linear(0.5, 0,   0.3) →  前进 0.5 m, 0.3 m/s
 *    Cmd_Move_Linear(0,   0.5, 0.3) →  左移 0.5 m
 *
 *  完整帧布局（共 21 字节，注意协议层永远走 SI ×10000）：
 *
 *    偏移   字段       类型     缩放    含义
 *    ────   ────────   ──────  ──────  ───────────────────────────────────
 *    [0]    0xDF
 *    [1]    Robot_ID = 0x01
 *    [2]    PC_ID    = 0x97
 *    [3]    A = 0x02
 *    [4]    B = 0x64
 *    [5]    LEN = 12
 *    [6..9]   px       s32 LE   ×10000   px=0.5 m  → 5000 → 88 13 00 00
 *    [10..13] py       s32 LE   ×10000
 *    [14..17] speed    s32 LE   ×10000   speed=0.3 → 3000 → B8 0B 00 00
 *    [18]   0xFD
 *    [19..20] checksum u16 LE
 *
 *  完成回传：A=0x6F, B=0x64, payload=[Process(0~0xFE 或 0xFF), Notice]
 *============================================================================*/
void Cmd_Move_Linear(float px, float py, float speed_mps, u8 profile)
{
    /* ★ profile: 0=匀速 (起停硬, 短距精到位)
     *            1=加减速 (★ 推荐, 梯形 ramp 平滑) */
    u8 buf[32];
    u8 cnt;
    s32 ipx, ipy, ispd;

    if (g_dfcom_unit_mode == DFCOM_UNIT_CM) {
        px        *= 0.01f;
        py        *= 0.01f;
        speed_mps *= 0.01f;
    } else if (g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        px        *= 0.001f;
        py        *= 0.001f;
        speed_mps *= 0.001f;
    }

    if (speed_mps <= 0.0f) return;
    if (profile > 1) profile = 1;   /* 兜底, 非 0/1 → 默认加减速 */

    cnt = build_header(buf, 0x02, CMD_LINEAR);

    ipx  = (s32)(px        * SCALE_SI);
    ipy  = (s32)(py        * SCALE_SI);
    ispd = (s32)(speed_mps * SCALE_SI);

    buf[cnt++] = 13;                          /* LEN: 12B (Px Py Sp) + 1B profile */
    put_s32_le(&buf[cnt], ipx);  cnt += 4;
    put_s32_le(&buf[cnt], ipy);  cnt += 4;
    put_s32_le(&buf[cnt], ispd); cnt += 4;
    buf[cnt++] = profile;                     /* ★ byte[12] = Profile (0/1) */

    cnt = finalize_frame(buf, 13);
    clear_move_slot(CMD_LINEAR);
    send_frame(buf, cnt);
}


/*============================================================================
 *  指令 2：平移+yaw 无头位移 Cmd_Move_LinearWithYaw
 *  ──────────────────────────────────────────────────────────────────────────
 *  Class/Cmd: A = 0x02, B = 0x65 (Motion_LinearWithYaw)
 *
 *  含义：在 (px, py) 平移的同时整车转动 dyaw（"无头" / "全向带转向" 模式）。
 *        小车端会规划出"边走边转"的运动，到达目标点时刚好转完。
 *
 *  参数（按 g_dfcom_unit_mode 解释）：
 *    CM 模式 (默认): px/py = cm,  dyaw = deg,  speed = cm/s
 *    M  模式:         px/py = m,   dyaw = rad,  speed = m/s (SI)
 *
 *  参数语义：
 *    px:    +X 位移
 *    py:    +Y 位移
 *    dyaw:  yaw 增量 (相对当前姿态, CCW+)
 *    speed: 平移速度
 *
 *  示例 (CM 模式, 默认):
 *    Cmd_Move_LinearWithYaw(50, 0, 90, 30)
 *      → 前进 50 cm 同时整车左转 90°, 30 cm/s
 *
 *  示例 (M 模式):
 *    Cmd_Move_LinearWithYaw(0.5, 0, 1.5708f, 0.3)
 *      → 前进 0.5 m 同时整车左转 π/2 rad, 0.3 m/s
 *
 *  完整帧布局（共 25 字节，协议层 SI ×10000）：
 *    [6..9]    px        s32 LE  ×10000
 *    [10..13]  py        s32 LE  ×10000
 *    [14..17]  dyaw_rad  s32 LE  ×10000
 *    [18..21]  speed     s32 LE  ×10000
 *    LEN = 16
 *
 *  完成回传：A=0x6F, B=0x65
 *============================================================================*/
void Cmd_Move_LinearWithYaw(float px, float py, float dyaw_rad, float speed_mps, u8 profile)
{
    /* ★ profile: 0=匀速, 1=加减速 (★ 推荐) */
    u8 buf[32];
    u8 cnt;
    s32 ipx, ipy, idyaw, ispd;

    if (g_dfcom_unit_mode == DFCOM_UNIT_CM) {
        px        *= 0.01f;
        py        *= 0.01f;
        dyaw_rad  *= 0.01745329f;
        speed_mps *= 0.01f;
    } else if (g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        px        *= 0.001f;
        py        *= 0.001f;
        dyaw_rad  *= 0.01745329f;
        speed_mps *= 0.001f;
    }

    if (speed_mps <= 0.0f) return;
    if (profile > 1) profile = 1;

    cnt = build_header(buf, 0x02, CMD_LINEAR_WITH_YAW);

    ipx   = (s32)(px        * SCALE_SI);
    ipy   = (s32)(py        * SCALE_SI);
    idyaw = (s32)(dyaw_rad  * SCALE_SI);
    ispd  = (s32)(speed_mps * SCALE_SI);

    buf[cnt++] = 17;                          /* LEN: 16B + 1B profile */
    put_s32_le(&buf[cnt], ipx);   cnt += 4;
    put_s32_le(&buf[cnt], ipy);   cnt += 4;
    put_s32_le(&buf[cnt], idyaw); cnt += 4;
    put_s32_le(&buf[cnt], ispd);  cnt += 4;
    buf[cnt++] = profile;                     /* ★ byte[16] = Profile */

    cnt = finalize_frame(buf, 17);
    clear_move_slot(CMD_LINEAR_WITH_YAW);
    send_frame(buf, cnt);
}


/*============================================================================
 *  指令 3：原地旋转 Cmd_Move_Rot
 *  ──────────────────────────────────────────────────────────────────────────
 *  Class/Cmd: A = 0x02, B = 0x63 (Motion_Rotate)
 *
 *  ★★ 重要：相对旋转 + ROS CCW+ ★★
 *
 *    dyaw 是【相对于小车当前姿态】的旋转增量（不是绝对角）。
 *
 *    正负规则 (ROS REP-103):
 *      dyaw > 0  → CCW  逆时针旋转 (从上往下看, 也就是车头向左转)
 *      dyaw < 0  → CW   顺时针旋转 (车头向右转)
 *
 *  参数（按 g_dfcom_unit_mode 解释）：
 *    CM 模式 (默认): dyaw = 度 (deg),    omega_max = 度/秒 (deg/s)
 *    M  模式:         dyaw = 弧度 (rad), omega_max = 弧度/秒 (rad/s) (SI)
 *
 *  示例 (CM 模式, 默认):
 *    当前 yaw=0 → Cmd_Move_Rot( 90, 60)   → 转到 +90° (左转), 60 deg/s
 *    当前 yaw=0 → Cmd_Move_Rot(-45, 60)   → 转到 -45° (右转)
 *
 *  示例 (M 模式):
 *    当前 yaw=0 → Cmd_Move_Rot( 1.5708f, 1.0f)  → 转到 +π/2 (左转)
 *    当前 yaw=0 → Cmd_Move_Rot(-0.7854f, 1.0f)  → 转到 -π/4 (右转)
 *
 *  完整帧布局（共 17 字节，协议层 SI rad ×10000）：
 *
 *    偏移      字段           类型     缩放     含义
 *    ────      ───────────    ──────  ──────   ────────────────────────────
 *    [6..9]    dyaw_rad       s32 LE  ×10000   yaw 增量 (rad), CCW+
 *    [10..13]  omega_max      s32 LE  ×10000   最大角速度 (rad/s), >0
 *    LEN = 8
 *
 *  完成回传：A=0x6F, B=0x63
 *============================================================================*/
void Cmd_Move_Rot(float dyaw_rad, float omega_max_rad_s)
{
    u8 buf[32];
    u8 cnt;
    s32 idyaw, iomega;

    /* ★ 单位转换: 原地旋转只有角度参数, MM 跟 CM 一样用 deg
     *   CM / MM 模式: deg + deg/s → rad + rad/s
     *   M  模式:       rad + rad/s, 不转换 (协议原生) */
    if (g_dfcom_unit_mode == DFCOM_UNIT_CM ||
        g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        dyaw_rad        *= 0.01745329f;
        omega_max_rad_s *= 0.01745329f;
    }

    if (omega_max_rad_s <= 0.0f) return;

    cnt = build_header(buf, 0x02, CMD_ROT);

    idyaw  = (s32)(dyaw_rad        * SCALE_SI);
    iomega = (s32)(omega_max_rad_s * SCALE_SI);

    buf[cnt++] = 8;
    put_s32_le(&buf[cnt], idyaw);  cnt += 4;
    put_s32_le(&buf[cnt], iomega); cnt += 4;

    cnt = finalize_frame(buf, 8);
    clear_move_slot(CMD_ROT);   /* 清残留, 防止 WaitMoveDone 误判完成 */
    send_frame(buf, cnt);
}


/*============================================================================
 *  指令 4：持续速度 Cmd_Move_Vel（遥控连续指令，无完成回传）
 *  ──────────────────────────────────────────────────────────────────────────
 *  Class/Cmd: A = 0x02, B = 0x62 (Motion_Velocity)
 *
 *  含义：让小车按 (vx, vy) 持续移动，按 vz 持续旋转。
 *        ★ 不会自动停！想停车要主动发 Cmd_Move_Vel(0, 0, 0)
 *        ★ 此指令小车收到立即响应，没有完成回传
 *
 *  参数（按 g_dfcom_unit_mode 解释）：
 *    CM 模式 (默认): vx/vy = 厘米/秒 (cm/s),  vz = 度/秒 (deg/s)
 *    M  模式:         vx/vy = 米/秒 (m/s),     vz = 弧度/秒 (rad/s) (SI)
 *
 *  方向：
 *    vx: +X 速度，+ = 前进
 *    vy: +Y 速度，+ = 左方
 *    vz: yaw 角速度，+ = CCW 左转 (逆时针)
 *
 *  示例 (CM 模式, 默认):
 *    Cmd_Move_Vel(30, 0,  0)  →  前进 30 cm/s
 *    Cmd_Move_Vel( 0, 0, 60)  →  原地左转 60 deg/s
 *    Cmd_Move_Vel( 0, 0,  0)  →  ★ 主动停车
 *
 *  示例 (M 模式):
 *    Cmd_Move_Vel(0.3f, 0,    0)    →  前进 0.3 m/s
 *    Cmd_Move_Vel(0,    0,    1.0f) →  原地左转 1 rad/s
 *
 *  典型用法：手柄/遥控 50Hz 推杆量；或 g_odom 闭环控速。
 *
 *  完整帧布局（共 21 字节，协议层 SI ×10000）：
 *    [6..9]    vx        s32 LE  ×10000   +X 速度 (m/s)
 *    [10..13]  vy        s32 LE  ×10000   +Y 速度 (m/s)
 *    [14..17]  vz_rad_s  s32 LE  ×10000   yaw 角速度 (rad/s)
 *    LEN = 12
 *============================================================================*/
void Cmd_Move_Vel(float vx_mps, float vy_mps, float vz_rad_s)
{
    u8 buf[32];
    u8 cnt;
    s32 ivx, ivy, ivz;

    /* ★ 单位转换: 根据 g_dfcom_unit_mode 把入参转成 SI
     *   CM 模式: cm/s + deg/s → m/s + rad/s
     *   MM 模式: mm/s + deg/s → m/s + rad/s
     *   M  模式: m/s + rad/s, 不转换 */
    if (g_dfcom_unit_mode == DFCOM_UNIT_CM) {
        vx_mps   *= 0.01f;
        vy_mps   *= 0.01f;
        vz_rad_s *= 0.01745329f;
    } else if (g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        vx_mps   *= 0.001f;
        vy_mps   *= 0.001f;
        vz_rad_s *= 0.01745329f;
    }

    cnt = build_header(buf, 0x02, CMD_VEL);

    ivx = (s32)(vx_mps   * SCALE_SI);
    ivy = (s32)(vy_mps   * SCALE_SI);
    ivz = (s32)(vz_rad_s * SCALE_SI);

    buf[cnt++] = 12;
    put_s32_le(&buf[cnt], ivx); cnt += 4;
    put_s32_le(&buf[cnt], ivy); cnt += 4;
    put_s32_le(&buf[cnt], ivz); cnt += 4;

    cnt = finalize_frame(buf, 12);
    send_frame(buf, cnt);
}


/*============================================================================
 *  指令 5：圆弧 Cmd_Move_Arc
 *  ──────────────────────────────────────────────────────────────────────────
 *  Class/Cmd: A = 0x02, B = 0x66 (Motion_Arc)
 *
 *  含义：小车沿圆弧路径运动，圆心在车体侧方 radius 距离处。
 *        典型用途：直线 → 圆弧 → 直线 平滑过渡，不用原地转向。
 *
 *  参数（按 g_dfcom_unit_mode 解释）：
 *    CM 模式 (默认): radius = cm, dyaw = deg, speed = cm/s
 *    M  模式:         radius = m,  dyaw = rad, speed = m/s (SI)
 *
 *  参数语义：
 *    radius:  圆弧半径, 仅取正值。小车端内部 fabsf() 取绝对值, 负号被忽略
 *    dyaw:    圆心角, CCW+。转向只看这个符号:
 *               dyaw > 0 → CCW 逆时针 (圆心在车体 +Y 左侧, 左转)
 *               dyaw < 0 → CW  顺时针 (圆心在车体 -Y 右侧, 右转)
 *    speed:   线速度, 正数
 *
 *  示例 (CM 模式, 默认):
 *    Cmd_Move_Arc(30, 90, 20)
 *      → 半径 30 cm 圆弧, 转 90° (左转), 20 cm/s
 *    Cmd_Move_Arc(30, 360, 20)
 *      → 整圆 (360°)
 *
 *  示例 (M 模式):
 *    Cmd_Move_Arc(0.3f, 1.5708f, 0.2f)
 *      → 半径 0.3 m 圆弧, 转 π/2 rad (左转), 0.2 m/s
 *
 *  参数范围 (M 模式):
 *    • radius: 物理上限别太小 (< 0.1 m 实际跑不出来)
 *    • dyaw:   |dyaw| < M_PI (3.14)，不要正好 ±M_PI
 *    • speed:  0.05 ~ 0.5 之间比较好
 *
 *  完整帧布局（共 21 字节，协议层 SI ×10000）：
 *    [6..9]    radius_m  s32 LE  ×10000   圆弧半径 (m)
 *    [10..13]  dyaw_rad  s32 LE  ×10000   圆心角 (rad), CCW+
 *    [14..17]  speed     s32 LE  ×10000   线速度 (m/s), >0
 *    LEN = 12
 *
 *  完成回传：A=0x6F, B=0x66
 *============================================================================*/
void Cmd_Move_Arc(float radius_m, float dyaw_rad, float speed_mps, u8 profile)
{
    /* ★ profile: 0=匀速, 1=加减速 (★ 推荐) */
    u8 buf[32];
    u8 cnt;
    s32 ir, idyaw, ispd;

    if (g_dfcom_unit_mode == DFCOM_UNIT_CM) {
        radius_m  *= 0.01f;
        dyaw_rad  *= 0.01745329f;
        speed_mps *= 0.01f;
    } else if (g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        radius_m  *= 0.001f;
        dyaw_rad  *= 0.01745329f;
        speed_mps *= 0.001f;
    }

    if (speed_mps <= 0.0f) return;
    if (profile > 1) profile = 1;

    cnt = build_header(buf, 0x02, CMD_ARC);

    ir    = (s32)(radius_m  * SCALE_SI);
    idyaw = (s32)(dyaw_rad  * SCALE_SI);
    ispd  = (s32)(speed_mps * SCALE_SI);

    buf[cnt++] = 13;                          /* LEN: 12B + 1B profile */
    put_s32_le(&buf[cnt], ir);    cnt += 4;
    put_s32_le(&buf[cnt], idyaw); cnt += 4;
    put_s32_le(&buf[cnt], ispd);  cnt += 4;
    buf[cnt++] = profile;                     /* ★ byte[12] = Profile */

    cnt = finalize_frame(buf, 13);
    clear_move_slot(CMD_ARC);
    send_frame(buf, cnt);
}


static u8 dfcom_freq_to_code(u8 freq_hz)
{
    if      (freq_hz == 10)  return 1;
    else if (freq_hz == 50)  return 5;
    else if (freq_hz == 100) return 10;
    else if (freq_hz == 200) return 20;
    else if (freq_hz == 250) return 25;
    else if (freq_hz == 500) return 50;
    else                     return 1;   /* 默认 10Hz */
}

static u8 dfcom_mode_to_visit_type(u8 mode)
{
    if      (mode == ODOM_MODE_CONTINUOUS) return 0x01;
    else if (mode == ODOM_MODE_ONESHOT)    return 0x02;
    else                                   return 0x02;   /* STOP → ONESHOT */
}

static void send_periodic_subscribe(u8 cmd, u8 mode, u8 freq_hz)
{
    u8 buf[15];
    u8 cnt;

    cnt = build_header(buf, 0x04, cmd);
    buf[cnt++] = 2;
    buf[cnt++] = dfcom_mode_to_visit_type(mode);
    buf[cnt++] = dfcom_freq_to_code(freq_hz);
    cnt = finalize_frame(buf, 2);
    send_frame(buf, cnt);
}

/*============================================================================
 *  指令 6：周期数据订阅
 *  ──────────────────────────────────────────────────────────────────────────
 *  周期访问入口: A = 0x04 (DataRecivViitClass), B = 0x80 (Time_V)
 *
 *  含义：
 *    Cmd_Subscribe_Odom   订阅 Odom v4 全量包（Yaw + 里程计 + IMU 原始）
 *    Cmd_Subscribe_VelPos 订阅 VelPos v2 简化包（Yaw + 位置 + 速度）
 *
 *  重要:
 *    当前 DCar 底盘固件的"订阅命令入口"仍是 0x04/0x80。
 *    VelPos v2 是回传帧 0x6C/0x81, 不是订阅命令 0x04/0x81。
 *    如果发送 0x04/0x81, 部分底盘不会开启回传, 表现为一直 no data。
 *    所以 Cmd_Subscribe_VelPos() 也通过 0x04/0x80 打开周期访问,
 *    但应用层默认只读取和打印 g_velpos。
 *
 *  ★ 这条指令的协议层格式未变（订阅机制本身不动），但回传里的 N_PosX/Y
 *    现在的含义是 ROS 坐标系：+X 前进、+Y 左方 ★
 *
 *  完整帧布局（共 11 字节）：
 *    [6]   type     u8    Vistt_type:
 *                         0x01 = 持续模式（小车自己 N Hz 推，最常用）
 *                         0x02 = 一次模式（小车发一帧后自动清零）
 *                         0x00 = 错误（小车会拒绝整个订阅！）
 *    [7]   freq     u8    频率枚举（不是 Hz 数字！）：
 *                         1=10Hz, 5=50Hz, 10=100Hz, 20=200Hz,
 *                         25=250Hz, 50=500Hz, 100=1000Hz
 *                         其他值小车也会拒绝。
 *    LEN = 2
 *
 *  ★ 重要：持续模式只要发一次！反复发会重置小车状态机。★
 *
 *  ★ 怎么"停止"持续推送？（官方文档原话）★
 *    "若已开启连续访问，只需发送单次访问指令即可自动关闭连续回传功能"
 *    所以本函数对 ODOM_MODE_STOP 内部转成 ONESHOT (0x02) 发出去——小车收到后
 *    把 Visit_Type 改成 0x02，发完下一帧自动清零，从此不再推。
 *============================================================================*/
void Cmd_Subscribe_Odom(u8 mode, u8 freq_hz)
{
    send_periodic_subscribe(0x80, mode, freq_hz);
}

void Cmd_Subscribe_VelPos(u8 mode, u8 freq_hz)
{
    send_periodic_subscribe(0x80, mode, freq_hz);
}


/*============================================================================
 *  指令 7：查询小车激活/校准状态 Cmd_Query_DcarState
 *  ──────────────────────────────────────────────────────────────────────────
 *  Class/Cmd: A = 0x0B (RobAttribViitClass), B = 0x40 (ActivateState_V = 64)
 *
 *  含义：问小车 "你校准好了吗 / 参数 OK 吗 / IMU 健康吗"。
 *        小车回传 A=0x8C, B=0x40, LEN=12，含 IMU 校准标志等。
 *
 *  发送帧（共 9 字节，无 payload）：
 *    [0..5]  0xDF 0x01 0x97 0x0B 0x40 0x00       LEN = 0
 *    [6]     0xFD
 *    [7..8]  checksum LE
 *
 *  回传帧 payload (12 字节):
 *    [0]    ifIMUcali   u8    1 = IMU 校准好，0 = 未校准（不能用！）
 *    [1]    ifFinetune  u8    1 = 控制参数已微调
 *    [2..3] gyro_z_off  s16   gyro Z 偏置 × 100
 *    [4..5] imu_par2    s16
 *    [6..7] imu_temp    s16
 *    [8..9] ctrl_par1   s16   控制 Par1 × 100
 *    [10..11] ctrl_par2 s16   控制 Par2 × 100
 *============================================================================*/
void Cmd_Query_DcarState(void)
{
    u8 buf[12];
    u8 cnt = build_header(buf, 0x0B, 0x40);
    buf[cnt++] = 0;                  /* LEN = 0，无 payload */
    cnt = finalize_frame(buf, 0);
    send_frame(buf, cnt);
}

/******************************************************************************
 * DFCom.h - Differ Tech 小车通信库（精简版，面向教学）
 * ---------------------------------------------------------------------------
 * 适用：STM32F103C8T6 + DcarON 系列小车（DF-Link 协议 / SI 单位 / ROS REP-103）
 * 官网：https://differ-tech.pages.dev/
 *
 * 库结构（拆成三个文件，互不依赖）：
 *   DFCom_Tx.c    — 发送侧（控制指令）
 *   DFCom_Rx.c    — 接收侧（自动收+解析 ODOM 数据）
 *   DFCom_Print.c — 终端打印（10Hz 定时器自动把 ODOM 打到 USART2）
 *
 * 硬件接线（F103C8T6）：
 *   USART1 (PA9/PA10)  @460800  → 接 DcarON 小车（必接）
 *   USART2 (PA2/PA3)   @115200  → 接 USB-TTL 看终端（必接，看 ODOM 数据用）
 *
 * 简单玩法 vs 专业玩法：
 *   简单：只调 Cmd_Move_Linear(...) 然后 delay_ms 就好
 *   专业：调 Cmd_Move_Linear(...) 后调 WaitMoveDone(...) 等小车真到位再下一条
 *   高手：自己读 g_odom.yaw_rad / g_odom.n_pos_x_m 做闭环
 *
 *
 *  ╔══════════════════════════════════════════════════════════════════════╗
 *  ║                                                                      ║
 *  ║   ★★★ 协议版本：SI + ROS REP-103（与老版本不兼容！）★★★             ║
 *  ║                                                                      ║
 *  ║   单位（国际标准）：                                                  ║
 *  ║       位置   → 米 (m)                                                ║
 *  ║       速度   → 米/秒 (m/s)                                           ║
 *  ║       角度   → 弧度 (rad)         ← 协议层全 rad，不再 deg!         ║
 *  ║       角速度 → 弧度/秒 (rad/s)                                       ║
 *  ║                                                                      ║
 *  ║   坐标系（ROS REP-103 标准）：                                       ║
 *  ║       +X = 小车前方 (forward)                                        ║
 *  ║       +Y = 小车左方 (left)                                           ║
 *  ║       +Z = 上 (up)                                                   ║
 *  ║       +Yaw = 逆时针 CCW（从上往下看，右手定则绕 +Z）                 ║
 *  ║                                                                      ║
 *  ║   速度范围参考（与车型/电机有关）：                                  ║
 *  ║       典型使用 0.1 ~ 0.5 m/s                                         ║
 *  ║       物理上限：取决于车型（标准版 520 电机大约 0.8~1.0 m/s）        ║
 *  ║       角速度建议：原地旋转 omega_max 0.5~2.0 rad/s                  ║
 *  ║                                                                      ║
 *  ╚══════════════════════════════════════════════════════════════════════╝
 ******************************************************************************/
#ifndef __DFCOM_H
#define __DFCOM_H

#include "sys.h"

/* ===========================================================================
 * 1. 协议常量
 * ===========================================================================*/
#define DFCOM_FRAME_HEADER   0xDF       /* 帧头 */
#define DFCOM_FRAME_TAIL     0xFD       /* 帧尾 */
#define DFCOM_ROBOT_ID       0x01       /* 小车 ID（默认 1） */
#define DFCOM_PC_ID          0x97       /* 上位机 ID（本 STM32 端） */

/* 指令码（cmd_code）—— 给 WaitMoveDone / GetMoveProgress 用
 * ★ 新协议：cmd 编号与含义全部刷新，老编号不再支持！★ */
#define CMD_VEL              0x62       /* Motion_Velocity   - 持续速度（无完成回传） */
#define CMD_ROT              0x63       /* Motion_Rotate     - 原地旋转（按 yaw 增量） */
#define CMD_LINEAR           0x64       /* Motion_Linear     - 直线位移（默认梯形加减速） */
#define CMD_LINEAR_WITH_YAW  0x65       /* Motion_LinearWithYaw - 平移+yaw（无头） */
#define CMD_ARC              0x66       /* Motion_Arc        - 圆弧 */

/* WaitMoveDone 返回值 */
#define MOVE_WAIT_TIMEOUT      0        /* 等待超时 */
#define MOVE_WAIT_DONE         1        /* 自然到位: progress=0xFF, notice=0x00 */
#define MOVE_WAIT_INTERRUPTED  2        /* 被打断:   progress=0xFF, notice!=0x00 */

/* 完成帧 notice：新运动指令打断时，notice 就是对应的 CMD_xxx */
#define MOVE_NOTICE_NONE         0x00
#define MOVE_NOTICE_REJECTED     0x01   /* 版本不支持或参数非法，任务未启动 */
#define MOVE_NOTICE_RC_TAKEOVER  0x0A   /* 遥控器接管 */

/* ODOM 订阅模式（Cmd_Subscribe_Odom 第一个参数） */
#define ODOM_MODE_CONTINUOUS 0x01       /* ★ 持续发送（最常用，发一次后小车自己 N Hz 推） */
#define ODOM_MODE_ONESHOT    0x02       /* 只发一次（用来"拍一帧"快照） */
#define ODOM_MODE_STOP       0x00       /* 停止发送（内部转成 ONESHOT 实现：发一次后小车自动停）*/

/* ODOM 订阅频率（Cmd_Subscribe_Odom 第二个参数，单位 Hz）
 * 小车端只接受这 7 个固定频率，其他值会被默默 fallback 到 10Hz: */
#define ODOM_FREQ_10HZ       10         /* 推荐：肉眼能看清屏幕滚动 */
#define ODOM_FREQ_50HZ       50         /* 做闭环用 */
#define ODOM_FREQ_100HZ      100
#define ODOM_FREQ_200HZ      200
#define ODOM_FREQ_250HZ      250
#define ODOM_FREQ_500HZ      500

/* ===========================================================================
 * 2. 接收到的 ODOM 数据（应用直接读 g_odom）
 *
 *  ★ 坐标系（ROS REP-103）★
 *    g_odom.n_pos_x_m  = 世界系 +X 方向位移（开机起累计），+X = 前进方向
 *    g_odom.n_pos_y_m  = 世界系 +Y 方向位移（开机起累计），+Y = 左方
 *    g_odom.b_vel_x_mps = 车体系 Vx（前进为正）
 *    g_odom.b_vel_y_mps = 车体系 Vy（左方为正）
 *    g_odom.yaw_rad    = 当前航向角，单位弧度 (rad)，CCW（逆时针）为正，范围 [-pi, pi]
 *      ↑ 协议层 IMU_TIME 回传 SI rad（s32/10000），与发送层一致。
 *      ↑ 想看度数：DFCOM_RAD2DEG(g_odom.yaw_rad) 或 *57.29578f
 * ===========================================================================*/

/* rad → deg 转换辅助宏（学生看不懂弧度时用一下） */
#define DFCOM_RAD2DEG(rad) ((rad) * 57.29578f)
#define DFCOM_DEG2RAD(deg) ((deg) * 0.01745329f)

/* ===========================================================================
 * OdomData_t — 全量 Odom v4 数据 (高级用户, 含 IMU 姿态 + 原始数据)
 *   对应小车回传帧: CMD 0x6C 0x80, 51B payload, version=0x04
 *   坐标系: ROS REP-103 FLU (+X 前, +Y 左, +Z 上, CCW+ 逆时针)
 * ===========================================================================*/
typedef struct {
    /* 姿态 (rad) */
    float roll_rad;         /* ROS roll */
    float pitch_rad;        /* ROS pitch */
    float yaw_rad;          /* ROS yaw (CCW+ 逆时针为正, wrap 到 [-pi, pi])
                             * 想看度数: DFCOM_RAD2DEG(g_odom.yaw_rad) */

    /* 加速度 (m/s², body 系 specific force, 静止 acc_z ≈ +9.81) */
    float acc_x_mps2;
    float acc_y_mps2;
    float acc_z_mps2;

    /* 角速度 (rad/s, body 系) */
    float gyro_x_rps;
    float gyro_y_rps;
    float gyro_z_rps;       /* +Z 轴角速度, CCW+ */

    /* 速度 (m/s, body 系 base_link) */
    float b_vel_x_mps;      /* Vx 前进速度 */
    float b_vel_y_mps;      /* Vy 左方速度 */
    float b_vel_z_mps;      /* Vz (2D 车 = 0) */

    /* 速度 (m/s, world 系 map) */
    float n_vel_x_mps;
    float n_vel_y_mps;
    float n_vel_z_mps;

    /* 位置 (m, world 系 map, 从开机起累积) */
    float n_pos_x_m;        /* +X 前进方向 位移 */
    float n_pos_y_m;        /* +Y 左方     位移 */
    float n_pos_z_m;        /* +Z 上       位移 (2D 车 = 0) */

    /* 小车端时间戳 */
    u32   car_time_int_s;   /* 秒 */
    u32   car_time_dec_us;  /* 微秒 */

    /* 客户端侧元数据 */
    u32   local_tick_ms;
    u32   frame_count;
    u8    fresh;
} OdomData_t;

extern volatile OdomData_t g_odom;

/* ===========================================================================
 * VelPosData_t — 简化版 VelPos v2 数据 (初学者, 只有 yaw + 速度 + 位置)
 *   对应小车回传帧: CMD 0x6C 0x81, 35B payload, version=0x02
 *   不含 roll/pitch/acc/gyro, 适合低带宽 / 不需 IMU 解算的应用
 * ===========================================================================*/
typedef struct {
    float yaw_rad;          /* rad, ROS CCW+ */

    /* 速度 body */
    float b_vel_x_mps;
    float b_vel_y_mps;
    float b_vel_z_mps;

    /* 速度 world */
    float n_vel_x_mps;
    float n_vel_y_mps;
    float n_vel_z_mps;

    /* 位置 world */
    float n_pos_x_m;
    float n_pos_y_m;
    float n_pos_z_m;

    u32   car_time_int_s;
    u32   car_time_dec_us;

    u32   local_tick_ms;
    u32   frame_count;
    u8    fresh;
} VelPosData_t;

extern volatile VelPosData_t g_velpos;

/* ===========================================================================
 * 3. 运动状态（应用通常不用直接读，给 WaitMoveDone 内部用）
 * ===========================================================================*/
typedef struct {
    u8 cmd_code;     /* 当前槽绑定的命令码；0 = 空槽 */
    u8 progress;     /* 0~0xFE = 进度%×255, 0xFF = 完成 */
    u8 last_notice;  /* 最近一次回传的 notice 字节（0=正常完成, 其他=中断码） */
    u8 fresh;        /* 1 = 有新进度回传 */
} MoveStatus_t;

/* 两个槽按发送顺序 A/B/A/B 交替；相邻任务永远不会共用一个槽。 */
extern volatile MoveStatus_t g_move_status[2];

/* ---------------------------------------------------------------------------
 * 本地毫秒计数 (TIM2 1ms 中断维护) — WaitMoveDone / Startup_Diagnose 等用
 * --------------------------------------------------------------------------*/
extern volatile u32 g_local_tick_ms;

/* ===========================================================================
 * 4. 小车状态（启动诊断用）
 *    通过 Cmd_Query_DcarState() 主动查询，回传 A=0x8C B=64 (0x40)
 *    收到回传后 g_dcar_state.received = 1，应用就能读到 imu_calibrated 等
 * ===========================================================================*/
typedef struct {
    u8  received;          /* 1 = 已收到一次回传（说明链路活的） */
    u8  imu_calibrated;    /* 1 = IMU 已校准，0 = 未校准/未激活 */
    u8  is_finetune;       /* 1 = 控制参数已微调 */
    s16 imu_offset_z_x100; /* gyro Z 偏置 × 100 */
    s16 imu_param2_x10000; /* IMU 参数 2 × 10000 */
    s16 imu_temp_x10000;   /* IMU 温度 */
    s16 ctrl_par1_x100;    /* 控制 Par1 × 100 */
    s16 ctrl_par2_x100;    /* 控制 Par2 × 100 */
} DcarState_t;

extern volatile DcarState_t g_dcar_state;

/* ===========================================================================
 * 单位模式 (g_dfcom_unit_mode)
 * ---------------------------------------------------------------------------
 * 默认: DFCOM_UNIT_CM (厘米 + 度, 学生友好)
 *
 * 调用 Cmd_Move_* 时入参的单位会根据这个全局变量自动转换:
 *   - 模式 = MM:       入参为 毫米 / 毫米每秒 / 度 / 度每秒 (精度高, 适合机械臂/精确定位)
 *   - 模式 = CM (默认): 入参为 厘米 / 厘米每秒 / 度 / 度每秒 (适合教学/普通场景)
 *   - 模式 = M:         入参为 米   / 米每秒    / 弧度 / 弧度每秒 (协议原生 SI)
 *
 * 内部转换后, 协议层永远是 SI (米 + rad), 小车端不需要知道这件事。
 *
 * 接收侧 g_odom / g_velpos 字段不受这个开关影响, 永远是 SI 原生
 * (想看 cm/mm/deg, 自己 *100 / *1000 / DFCOM_RAD2DEG() 转一下)。
 * ===========================================================================*/
typedef enum {
    DFCOM_UNIT_CM = 0,   /* ★ 默认: 厘米 + 度 (适合学生 / 初学者) */
    DFCOM_UNIT_M  = 1,   /* 米 + 弧度 (SI, 协议原生, 数值更小) */
    DFCOM_UNIT_MM = 2    /* 毫米 + 度 (精确定位场景, 数值更直观) */
} DFCom_UnitMode_e;

extern u8 g_dfcom_unit_mode;   /* 默认 = DFCOM_UNIT_CM */

/* ===========================================================================
 * 4. API
 * ===========================================================================*/

/* ---- 初始化 ---- */
/* 没有专门的 DFCom_Init —— 见 main.c::System_Init，分三步：
 *   uart_init(460800);       // USART1 接小车
 *   usart2_init(115200);     // USART2 接 USB-TTL 看 printf
 *   Odom_PrintTimer_Init();  // TIM2 维护 tick + 10Hz 自动打印 ODOM
 */

/* ---- Tx 发送侧（DFCom_Tx.c）：发完立即返回，0 阻塞 ---- */
/*  ────────────────────────────────────────────────────────────────────────
 *  坐标系（永远是 ROS REP-103, 与 g_dfcom_unit_mode 无关）：
 *    +X = 前方 (forward)      ← 想"前进"就 px > 0 / vx > 0
 *    +Y = 左方 (left)         ← 想"左移"就 py > 0 / vy > 0
 *    +Yaw = CCW 逆时针         ← 想"左转"就 dyaw > 0
 *
 *  ★ 入参单位由 g_dfcom_unit_mode 决定 ★
 *
 *    CM 模式 (默认, DFCOM_UNIT_CM):
 *      位置 px / py        → 厘米 (cm)            例: 50 = 0.5 米
 *      速度 speed/vx/vy    → 厘米/秒 (cm/s)       例: 30 ≈ 0.3 m/s
 *      角度 dyaw           → 度 (deg)             例: 90 ≈ M_PI/2 rad
 *      角速度 omega/vz     → 度/秒 (deg/s)        例: 60 ≈ 1.05 rad/s
 *
 *    M 模式 (DFCOM_UNIT_M, SI 协议原生):
 *      位置 px / py        → 米 (m)               例: 0.5 m
 *      速度 speed/vx/vy    → 米/秒 (m/s)          例: 0.3 m/s
 *      角度 dyaw           → 弧度 (rad)           例: 1.5708 ≈ 90°
 *      角速度 omega/vz     → 弧度/秒 (rad/s)      例: 1.0 ≈ 57°/s
 *
 *  参数语义约定（与单位模式无关）：
 *    Cmd_Move_Linear:        (px, py) = 目标位移矢量, speed = 标量速度
 *    Cmd_Move_LinearWithYaw: 在平移同时整体转 dyaw（无头模式）
 *    Cmd_Move_Rot:           原地旋转, dyaw 是【相对当前姿态的增量】，CCW+
 *    Cmd_Move_Vel:           持续速度（不会自动停！要主动 (0,0,0) 停车）
 *    Cmd_Move_Arc:           radius>0 半径; 转向只看 dyaw 符号 (CCW+ 左转, CW- 右转)
 *  ────────────────────────────────────────────────────────────────────────*/
/* ★ profile 参数: 0=匀速, 1=梯形加减速, 2=高精度距离闭环 */
void Cmd_Move_Linear        (float px, float py, float speed_mps, u8 profile);                  /* 0x64 直线位移 */
void Cmd_Move_LinearWithYaw (float px, float py, float dyaw_rad, float speed_mps, u8 profile);  /* 0x65 平移+yaw（无头） */
void Cmd_Move_Rot           (float dyaw_rad, float omega_max_rad_s);                /* 0x63 原地旋转（增量） */
void Cmd_Move_Vel           (float vx_mps, float vy_mps, float vz_rad_s);           /* 0x62 持续速度（无完成回传） */
void Cmd_Move_Arc           (float radius_m, float dyaw_rad, float speed_mps, u8 profile);      /* 0x66 圆弧 */

/* ODOM/VelPos 订阅：发一次小车就持续发送
 *   mode: ODOM_MODE_CONTINUOUS / ODOM_MODE_ONESHOT / ODOM_MODE_STOP
 *   freq_hz: 10/50/100/200/250/500（小车协议枚举，其他自动 fallback 到 10）
 * 教学常用：Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ);
 */
void Cmd_Subscribe_Odom   (u8 mode, u16 freq_hz);
void Cmd_Subscribe_VelPos (u8 mode, u16 freq_hz);

/* 查询小车激活/校准状态（启动诊断用）
 * 发完后等 100~500ms，读 g_dcar_state.received/imu_calibrated
 */
void Cmd_Query_DcarState(void);

/* ---- Rx 接收侧（DFCom_Rx.c）：在 USART1 IDLE 中断里自动调用 ---- */
void DFCom_RxParse(u8 *data, u16 size);  /* 协议解析入口（中断里调） */

/* ---- 启动阶段的运动回传隔离 ----
 * Quarantine: 丢弃全部 A=0x6F，供上电停车/诊断阶段隔离旧回传。
 * Start:      清空 A/B 两槽后开始接收业务运动回传。
 */
void DFCom_MoveSessionQuarantine(void);
void DFCom_MoveSessionStart(void);

/* ---- 可选辅助 ---- */

/* 等待最近一条运动指令完成（阻塞；ODOM 的定时打印仍由中断继续）
 *   cmd_code:    CMD_LINEAR / CMD_LINEAR_WITH_YAW / CMD_ROT / CMD_ARC
 *   timeout_ms:  0 = 无限等；>0 = 允许该任务执行的最长时间
 * 返回: MOVE_WAIT_DONE / MOVE_WAIT_TIMEOUT / MOVE_WAIT_INTERRUPTED
 * 被打断时可用 GetMoveNotice(cmd_code) 读取原因码
 *
 * ★ 给学生的提示：★
 *   - 想"发完就跑下一条"：不用这个函数（删掉就行）
 *   - 在 timeout 内到位会立刻返回；到点仍未完成就返回 TIMEOUT，
 *     随后发送的下一条运动命令会按协议合法打断上一条
 *   - CMD_VEL (持续速度) 没有完成回传，别用 WaitMoveDone 等它
 */
u8 WaitMoveDone(u8 cmd_code, u32 timeout_ms);

/* 进度查询（不阻塞，返回 0~254 进度 / 255 完成 / 0=未开始） */
u8 GetMoveProgress(u8 cmd_code);

/* 最近一次完成帧 notice（0=完成, 1=拒绝, 0x0A=RC接管, CMD_xxx=新指令打断） */
u8 GetMoveNotice(u8 cmd_code);

/* ---- Print（DFCom_Print.c）：TIM2 中断自动调用 ---- */
void VelPos_Print(void);            /* VelPos v2 简化版 (默认调) */
void Odom_Print(void);              /* Odom v4 全量版 (进阶用, 默认 TIM2 里注释) */
void Odom_PrintTimer_Init(void);    /* 启动 10Hz 自动打印 */

#endif /* __DFCOM_H */

/******************************************************************************
 *
 *   ╔══════════════════════════════════════════════════════════════════════╗
 *   ║                                                                      ║
 *   ║       DcarON DFCom 客户端例程 (精简版) - STM32F103C8T6 裸机          ║
 *   ║       协议版本: SI + ROS REP-103 (m / m/s / rad / +X 前 +Y 左)       ║
 *   ║                                                                      ║
 *   ║              官方网站：https://differ-tech.pages.dev/                ║
 *   ║                                                                      ║
 *   ╚══════════════════════════════════════════════════════════════════════╝
 *
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  ▌ 一、硬件接线
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *  STM32F103C8T6 板子 ←→ 接线对象
 *
 *  ┌──────────────────────────────────────┬─────────────────────────────────┐
 *  │ STM32 引脚                            │ 接到                            │
 *  ├──────────────────────────────────────┼─────────────────────────────────┤
 *  │ USART1_TX  (PA9)                     │ DcarON 小车 USART1_RX           │
 *  │ USART1_RX  (PA10)                    │ DcarON 小车 USART1_TX           │
 *  │ GND                                  │ DcarON GND（共地，必接）         │
 *  ├──────────────────────────────────────┼─────────────────────────────────┤
 *  │ USART2_TX  (PA2)                     │ USB-TTL 模块 RX                 │
 *  │ USART2_RX  (PA3)                     │ USB-TTL 模块 TX（可不接）       │
 *  │ GND                                  │ USB-TTL GND（共地，必接）       │
 *  └──────────────────────────────────────┴─────────────────────────────────┘
 *
 *  USB-TTL 模块（CH340/CP2102/FT232 都行）插电脑 USB，电脑识别成 COMx 口。
 *
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  ▌ 二、串口参数
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *   USART1 (接 DcarON): 460800 bps, 8N1, 无流控
 *   USART2 (接电脑   ): 115200 bps, 8N1, 无流控
 *
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  ▌ 三、怎么看 printf 输出
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *  1. 把 USB-TTL 模块插电脑
 *  2. 设备管理器里看分配到哪个 COM 口（比如 COM6）
 *  3. 打开串口工具（推荐 MobaXterm / Tera Term / SecureCRT / 串口助手）
 *  4. 选择 COMx，波特率 115200，8N1，无流控
 *  5. 点 "打开/连接"
 *  6. 给 STM32 上电，应该看到下面这样的输出：
 *
 *       ============================================
 *         DcarON DFCom Client - STM32F103C8T6
 *         https://differ-tech.pages.dev/
 *       ============================================
 *       [INIT] USART1 = 460800 (to DcarON)
 *       [INIT] USART2 = 115200 (this terminal)
 *       [INIT] Subscribe ODOM @ 10Hz...
 *       [INIT] Query DcarState...
 *       [INIT] DcarState: imu_calibrated=1, finetune=1, par1=1.00 par2=1.00
 *       [INIT] ODOM data flowing ✓ (frame_count=15 in 1500ms)
 *       [INIT] ✓ All OK, sending commands...
 *
 *       [ODOM] Yaw=  0.12 X(fwd)=  0.000 Y(left)=  0.000 ... fr=15
 *       [ODOM] Yaw=  0.13 X(fwd)=  0.024 Y(left)=  0.000 ... fr=16
 *       ...
 *
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  ▌ 四、Keil 工程必须勾的选项
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *   Project → Options → Target → Code Generation → ☑ Use MicroLIB
 *
 *  不勾的话 printf 不工作（fputc 重定向用到 MicroLIB），整个例程会卡在第一个
 *  printf 那里——看不到任何输出。
 *
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  ▌ 五、★ 单位约定（默认 厘米 + 度, 可切换 SI）★
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *   全局变量 g_dfcom_unit_mode 控制 Cmd_Move_* 入参的单位:
 *
 *      DFCOM_UNIT_CM (默认, 学生友好):
 *          位置   → 厘米 (cm)         例: 50 = 0.5 米
 *          速度   → 厘米/秒 (cm/s)    例: 30 ≈ 0.3 m/s
 *          角度   → 度 (deg)          例: 90 ≈ M_PI/2 rad
 *          角速度 → 度/秒 (deg/s)     例: 60 ≈ 1.05 rad/s
 *
 *      DFCOM_UNIT_M (SI 协议原生):
 *          位置   → 米 (m)            例: 0.5  = 50 cm
 *          速度   → 米/秒 (m/s)       例: 0.3  ≈ 30 cm/s
 *          角度   → 弧度 (rad)        例: 1.5708 ≈ 90°
 *          角速度 → 弧度/秒 (rad/s)
 *
 *   协议层永远是 SI，小车端不知道这个开关存在。
 *   接收侧 g_odom 字段也永远是 SI 原生 (不受开关影响)。
 *
 *   坐标系 (ROS REP-103, 永远不变):
 *      +X = 前方 (forward) ← 想"前进"就 px > 0 或 vx > 0
 *      +Y = 左方 (left)    ← 想"左移"就 py > 0 或 vy > 0
 *      +Yaw = CCW 逆时针    ← 想"左转"就 dyaw > 0 (右手定则绕 +Z)
 *
 *   速度范围参考（与车型有关）:
 *      CM 模式: 典型 10 ~ 50 cm/s; omega_max 30 ~ 120 deg/s
 *      M  模式: 典型 0.1 ~ 0.5 m/s; omega_max 0.5 ~ 2.0 rad/s
 *      物理上限取决于车型 (标准版 520 电机大约 80 ~ 100 cm/s)
 *
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  ▌ 六、三个等级的玩法 (示例用默认 CM 模式)
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *  🟢 入门 (最简单):
 *      Cmd_Move_Linear(0, 50, 30, 2); // +Y 左移 50 cm, 30 cm/s, profile=2(高精度)
 *      delay_ms(3000);                // 粗暴等 3 秒
 *
 *  🟡 进阶 (等小车真到位):
 *      Cmd_Move_Linear(50, 0, 30, 2); // 第 4 参 profile: 2=高精度距离闭环
 *      WaitMoveDone(CMD_LINEAR, 1000); // 最多执行 1s；提前到位就提前返回
 *                                      //   传 0 才表示永久等待
 *                                      //   ─── 第一参 cmd_code 必须跟刚发的函数对应:
 *                                      //       Cmd_Move_Linear         → CMD_LINEAR
 *                                      //       Cmd_Move_LinearWithYaw  → CMD_LINEAR_WITH_YAW
 *                                      //       Cmd_Move_Rot            → CMD_ROT
 *                                      //       Cmd_Move_Arc            → CMD_ARC
 *                                      //       Cmd_Move_Vel            → 不要用 WaitMoveDone
 *
 *  🔴 高手 (闭环, 读 g_odom 永远是 SI 米/弧度):
 *      while (g_odom.n_pos_x_m < 0.5f) {     // 0.5 m = 50 cm
 *          Cmd_Move_Vel(30, 0, 0);           // CM 模式: 30 cm/s
 *          delay_ms(50);
 *      }
 *      Cmd_Move_Vel(0, 0, 0);                // 主动停车
 *
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  ▌ 七、常见报错与排查
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *  ❌ 终端啥都不输出
 *     → USB-TTL 接错 PA2/PA3？波特率不是 115200？Keil 没勾 Use MicroLIB？
 *
 *  ❌ 看到 INIT 输出但 [INIT] ✗ NO TELEMETRY data
 *     → PA9/PA10 接小车的线没接？小车没上电？小车 USART1 不是 460800？
 *
 *  ❌ 看到 [INIT] ✗ DcarState query timeout
 *     → 同上，PA9/PA10 接线有问题；或小车上电时间不够
 *
 *  ❌ 看到 imu_calibrated=0
 *     → 小车未激活或 IMU 未校准 → 用 DcarON 上位机软件先激活/校准
 *
 *  ❌ 小车收到指令但不动
 *     → 小车 OLED 是不是显示"未激活/未校准"？激活了才能动
 *     → DcarON 安全锁：未激活时即使收到指令电机也不转
 *
 *  ❌ 小车动的方向不对（前进变后退之类）
 *     → 新协议是 +X 前 / +Y 左！如果是从老例程改来的，记得交换 px/py
 *
 ******************************************************************************/

#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "DFCom.h"
#include "system_stm32f10x.h"

/* M_PI 在 Keil C90 默认未定义，自己声明 */
#ifndef M_PI
#define M_PI   3.14159265358979323846f
#endif

/* ===========================================================================
 * 系统初始化
 * ===========================================================================*/
static void System_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(460800);          /* USART1 → DcarON */
    usart2_init(115200);        /* USART2 → printf 终端 */
    Odom_PrintTimer_Init();     /* TIM2: 维护本地 ms tick + 10Hz 自动打 ODOM */
    /* ★ 单位模式默认 CM (厘米 + 度)
     * 如需切换, 看 main() 里的 "★★ 单位切换 ★★" 那一段 */
}

/* ===========================================================================
 * 启动诊断 —— 帮学生快速发现接线/激活/校准问题
 * ===========================================================================*/
/* ★ 启动诊断函数 — 可注释掉
 * ──────────────────────────────────────────────────────────────────────
 * 这是给学生用的"自检流程", 帮你快速发现接线/激活/通信问题。
 * 跑完产生几行 [INIT] 日志, 让你确认链路 OK 再开始发指令。
 *
 * 如果你已经确认接线正常 / 小车已激活, 这一步是可选的:
 *   在 main() 里把 Startup_Diagnose() 这一行注释掉即可, 直接进 while(1) 发指令。
 *
 * 等待时间已尽可能压缩:
 *   - 订阅后等 200ms (1~2 帧 10Hz 推回来) 确认链路
 *   - DcarState 查询超时 200ms (实际通常 5~30ms 就回来)
 *   - 总耗时 ~500ms, 比之前 (~3 秒) 短得多
 */
static void Startup_Diagnose(void)
{
    u32 t0;
    u32 fc_odom_before,   fc_odom_after;
    u32 fc_velpos_before, fc_velpos_after;
    u32 diff_odom, diff_velpos, diff_total;

    printf("\r\n");
    printf("============================================\r\n");
    printf("  DcarON DFCom Client - STM32F103C8T6\r\n");
    printf("  Protocol: SI + ROS REP-103 (+X fwd, +Y left)\r\n");
    printf("  https://differ-tech.pages.dev/\r\n");
    printf("============================================\r\n");
    printf("[INIT] USART1 = 460800 (to DcarON)\r\n");
    printf("[INIT] USART2 = 115200 (this terminal)\r\n");
    printf("[CLK] SystemCoreClock=%lu\r\n", (unsigned long)SystemCoreClock);
    printf("[CLK] RCC->CFGR=0x%08lX RCC->CR=0x%08lX\r\n",
           (unsigned long)RCC->CFGR,
           (unsigned long)RCC->CR);
    printf("[CLK] SWS=0x%02lX HSERDY=%lu PLLRDY=%lu\r\n",
           (unsigned long)((RCC->CFGR & RCC_CFGR_SWS) >> 2),
           (unsigned long)(((RCC->CR & RCC_CR_HSERDY) != 0) ? 1 : 0),
           (unsigned long)(((RCC->CR & RCC_CR_PLLRDY) != 0) ? 1 : 0));

    /* 步骤 1：独立订阅 Odom + VelPos（持续 10Hz） */
    printf("[INIT] Subscribe ODOM + VelPos @ 10Hz...\r\n");
    Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ);
    Cmd_Subscribe_VelPos(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ);
    delay_ms(150);   /* 等 ~1.5 帧, 让链路稳定 */

    /* 步骤 2：查询小车状态（IMU 校准 / 控制参数）*/
    printf("[INIT] Query DcarState...\r\n");
    g_dcar_state.received = 0;
    Cmd_Query_DcarState();

    /* 等回传, 最多 200ms (实际通常 5~30ms 回来) */
    t0 = g_local_tick_ms;
    while (!g_dcar_state.received && (g_local_tick_ms - t0) < 200) {
        delay_ms(5);
    }

    if (g_dcar_state.received) {
        printf("[INIT] DcarState: imu_calibrated=%d, finetune=%d, par1=%.2f par2=%.2f\r\n",
               (int)g_dcar_state.imu_calibrated,
               (int)g_dcar_state.is_finetune,
               (double)(g_dcar_state.ctrl_par1_x100 / 100.0f),
               (double)(g_dcar_state.ctrl_par2_x100 / 100.0f));

        if (g_dcar_state.imu_calibrated == 0) {
            printf("[INIT] ✗ IMU NOT CALIBRATED!\r\n");
            printf("[INIT]   小车 IMU 没校准 -- 用 DcarON 上位机软件先激活/校准！\r\n");
            printf("[INIT]   未激活的小车会拒绝运动（安全锁），电机不会转。\r\n");
        }
    } else {
        printf("[INIT] ✗ DcarState query timeout!\r\n");
        printf("[INIT]   小车没回应 -- 检查:\r\n");
        printf("[INIT]    1. PA9/PA10 与小车 USART1 接线（注意 TX/RX 交叉）\r\n");
        printf("[INIT]    2. STM32 GND 与小车 GND 是否共地\r\n");
        printf("[INIT]    3. 小车有没有上电、是不是 460800 bps\r\n");
    }

    /* 步骤 3：等 200ms 看 telemetry 流是否在动
     * Odom v4 和 VelPos v2 是两条独立订阅线；本示例两条都订阅，所以两个计数器都看。 */
    printf("[INIT] Waiting 200ms to check telemetry flow...\r\n");
    fc_odom_before   = g_odom.frame_count;
    fc_velpos_before = g_velpos.frame_count;
    delay_ms(200);   /* 10Hz × 0.2s ≈ 2 帧, 足够确认链路 (原来 1500ms 太久了) */
    fc_odom_after   = g_odom.frame_count;
    fc_velpos_after = g_velpos.frame_count;
    diff_odom   = fc_odom_after   - fc_odom_before;
    diff_velpos = fc_velpos_after - fc_velpos_before;
    diff_total  = diff_odom + diff_velpos;

    if (diff_total > 0) {
        printf("[INIT] Telemetry flowing ✓ (Odom=%lu + VelPos=%lu in 200ms)\r\n",
               (unsigned long)diff_odom, (unsigned long)diff_velpos);
        printf("[INIT] ✓ All OK, sending commands...\r\n");
    } else {
        printf("[INIT] ✗ NO TELEMETRY data (neither Odom v4 nor VelPos v2)!\r\n");
        printf("[INIT]   小车没在推数据 -- 可能原因:\r\n");
        printf("[INIT]    1. 接线问题 (同上)\r\n");
        printf("[INIT]    2. 小车未激活 (上位机软件激活后再试)\r\n");
        printf("[INIT]    3. 小车 USART1 不是 460800\r\n");
    }

    printf("\r\n");
}


/* ===========================================================================
 * 主程序
 * ===========================================================================*/
int main(void)
{
    /* ─── 1. 初始化（串口、定时器）─────────────────── */
    System_Init();

    /*
     * 上电安全握手：
     *   1) 立刻发一次零速度，先尽力停车；
     *   2) DCar 由整车供电，等待其真正启动稳定后再发一次零速度；
     *   3) 这段时间所有 A=0x6F 运动回传都隔离，不允许污染首个业务任务。
     */
    DFCom_MoveSessionQuarantine();
    Cmd_Move_Vel(0, 0, 0);
    delay_ms(1000);   /* DCar 从整车取电，启动到 ready 大约需要 500ms~1s */
    Cmd_Move_Vel(0, 0, 0);

    /* ===========================================================
     * ★★ 单位切换 (一行就能改, 想用什么单位取消注释那一行就行) ★★
     * -----------------------------------------------------------
     *   Cmd_Move_* 函数入参的单位由 g_dfcom_unit_mode 决定:
     *
     *     DFCOM_UNIT_CM (★ 默认, 不改就是这个)
     *         cm / cm/s / deg / deg/s
     *         例: Cmd_Move_Linear(50, 0, 30, 2)   → 前进 50 cm, 30 cm/s
     *
     *     DFCOM_UNIT_MM (毫米, 精确定位 / 机械臂场景)
     *         mm / mm/s / deg / deg/s
     *         例: Cmd_Move_Linear(500, 0, 300, 2) → 前进 500 mm, 300 mm/s
     *
     *     DFCOM_UNIT_M (协议原生 SI)
     *         m / m/s / rad / rad/s
     *         例: Cmd_Move_Linear(0.5f, 0, 0.3f, 2) → 前进 0.5 m, 0.3 m/s
     *
     *   ⚠ 接收侧 g_odom / g_velpos 字段永远 SI (m / m/s / rad),
     *      不受这个开关影响 (要看 cm/mm 自己 *100 / *1000)。
     * -----------------------------------------------------------*/
    // g_dfcom_unit_mode = DFCOM_UNIT_MM;   /* 毫米 + 度 */
    // g_dfcom_unit_mode = DFCOM_UNIT_M;    /* 米 + 弧度 (SI) */
    /* (不写就是默认 DFCOM_UNIT_CM, 厘米 + 度) */

    /* ─── 2. 启动诊断 —— 打印链路 / 激活状态 ──────── */
    /* ★ 如果已经确认接线 + 激活都 OK, 可以把下面这行注释掉, 启动更快 */
    Startup_Diagnose();

    /* 即使用户关闭 Startup_Diagnose，也给第二次停车的旧终止帧留出隔离窗口。 */
    delay_ms(50);
    /* 诊断期旧回传已经隔离；从两个全零槽开始接收正式运动任务。 */
    DFCom_MoveSessionStart();
    printf("[INIT] Motion session ready (A/B slots cleared)\r\n");

    /* ★★★ ODOM 自动打印位置说明 ★★★
     * --------------------------------------------------------------
     *  TIM2 在 System_Init() 里被初始化, 之后每 100ms (10Hz) 进入
     *  TIM2_IRQHandler (DFCom_Print.c:72) 自动调用:
     *
     *    VelPos_Print()   ← 默认开 (简化版, 只 yaw+vel+pos)
     *    // Odom_Print()  ← 默认注释 (全量进阶版, 含 roll/pitch/acc/gyro)
     *
     *  ⚠ 想换打印内容: 改 DFCom_Print.c:80~82 那块的注释 (取消 Odom_Print()
     *                  的注释 / 注释掉 VelPos_Print 都行, 也可以两个同时开)
     *  ⚠ 想关掉自动打印: 设 g_print_odom_enable = 0 (DFCom_Print.c:29)
     *  ⚠ 想手动调用一次: 在你自己代码里直接调 VelPos_Print() 或 Odom_Print()
     * --------------------------------------------------------------*/

    /* ─── 3. 主循环：发指令 + TIM2 自动打 ODOM ────── */
    while (1)
    {
        /* ────────────────────────────────────────────
         * 当前回归序列：
         *   1) +X 直线最多 1s
         *   2) -X 直线最多 1s（同类型连续指令，专门验证旧 FF 隔离）
         *   3) 左转 15°，最多 1s
         *   4) 右转 15°，最多 1s（再次验证同类型连续指令）
         *
         * 入门玩法：注释掉 WaitMoveDone，加 delay_ms(3000)
         * 进阶玩法：用 WaitMoveDone 等小车真到位（当前就是）
         *
         * 默认 CM 模式：速度统一 30 cm/s (约 0.3 m/s)
         * 想改 SI 模式：System_Init 里打开 g_dfcom_unit_mode = DFCOM_UNIT_M
         *               然后把下面数值改成 0.5f / 0.3f 等
         *
         * ★ WaitMoveDone 第二参是任务允许执行的最长时间
         *   1000 = 最多执行 1 秒；1 秒内提前完成就立即进入下一条，
         *   1 秒仍未完成就返回 TIMEOUT，下一条运动命令会合法打断它。
         *   0 才表示永久等待。A/B 交替槽会吞掉上一任务随后到达的终止帧，
         *   不会把旧任务的 FF 错认成新任务完成。
         *
         * ★ WaitMoveDone 第一参 = CMD_xx, 必须跟刚发的指令对应!
         *   Cmd_Move_Linear         → CMD_LINEAR           (0x64)
         *   Cmd_Move_LinearWithYaw  → CMD_LINEAR_WITH_YAW  (0x65)
         *   Cmd_Move_Rot            → CMD_ROT              (0x63)
         *   Cmd_Move_Arc            → CMD_ARC              (0x66)
         *   Cmd_Move_Vel            → ★ 别用 WaitMoveDone (没完成回传)
         * ────────────────────────────────────────────*/
        /* ★ Cmd_Move_Linear 第 4 参 = profile:
         *   0 = 匀速 (起停硬, 短距精确到位)
         *   1 = 加减速 (★ 推荐, 平滑梯形 ramp)
         */
        Cmd_Move_Linear( 50,   0,  30, 2);  WaitMoveDone(CMD_LINEAR, 1000);  /* +X 前进 50cm, Profile 2 */
        Cmd_Move_Linear(-50,   0,  30, 2);  WaitMoveDone(CMD_LINEAR, 1000);  /* -X 后退 50cm */
        Cmd_Move_Rot( 15, 60);  WaitMoveDone(CMD_ROT, 1000);/* 原地左转 15°（CCW 逆时针，60 deg/s）*/
        Cmd_Move_Rot( -15, 60);  WaitMoveDone(CMD_ROT, 1000);/* 原地右转 15°（CW 顺时针，60 deg/s）*/

        /* 想画整圆？取消注释下面这行（半径 30cm, 圆心角 360°, 20 cm/s, 加减速）：*/
        // Cmd_Move_Arc(30, 360, 20, 1);  WaitMoveDone(CMD_ARC, 0);

        /* 想原地左转 90°（CCW 逆时针，60 deg/s）？*/
        // Cmd_Move_Rot( 90, 60);  WaitMoveDone(CMD_ROT, 0);

        /* 想原地右转 90°（CW 顺时针，dyaw 取负）？*/
        // Cmd_Move_Rot(-90, 60);  WaitMoveDone(CMD_ROT, 0);
    }
}

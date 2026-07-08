/******************************************************************************
 * DFCom_Print.c - 终端打印 + 本地 tick
 * ---------------------------------------------------------------------------
 * 提供两个东西：
 *   1. g_local_tick_ms：本地毫秒计数（TIM2 1ms 中断维护，给 WaitMoveDone 等用）
 *   2. Odom_Print()：把 g_odom 打到 USART2（printf 已重定向到 USART2）
 *      TIM2 计数到 100ms 时自动调用一次（10Hz 刷屏）
 *
 * TIM2 中断：72MHz / 7200 = 10kHz 计数器，1ms 一次中断
 *   - 每次中断 g_local_tick_ms++
 *   - 每 100 次中断（100ms） Odom_Print() 一次
 *
 * 学生在 USART2（USB-TTL 接 PC）终端看到的输出（坐标系 ROS REP-103）：
 *   X(fwd) = +X 前进方向 (m)        Y(left) = +Y 左方 (m)
 *   Yaw 协议层是弧度 (rad)，但终端打印转成度 (deg) 学生看着方便，CCW+ 逆时针为正
 *
 *   [ODOM] Yaw=  0.12 X(fwd)= 0.000 Y(left)= 0.000 Vx(fwd)= 0.00 Vy(left)= 0.00 fr=10
 *   [ODOM] Yaw=  0.15 X(fwd)= 0.024 Y(left)= 0.000 Vx(fwd)= 0.25 Vy(left)= 0.00 fr=11
 ******************************************************************************/
#include "DFCom.h"
#include "stm32f10x.h"
#include "usart.h"
#include "stdio.h"

/* ---- 本地 tick (ms) —— DFCom_Rx.c 用来给 ODOM 数据打时间戳 ---- */
volatile u32 g_local_tick_ms = 0;

/* ---- 控制是否自动打印（学生可以临时关掉） ---- */
volatile u8 g_print_odom_enable = 1;

/* ===========================================================================
 * TIM2 初始化：1ms 中断
 *   APB1 = 36MHz, TIM2 计数频率 = 72MHz / Prescaler
 *   设 Prescaler = 7200 - 1, Period = 10 - 1 → 10kHz/10 = 1kHz = 1ms
 *   (或者 Prescaler=72-1, Period=1000-1，也是 1ms，原理一样)
 * ===========================================================================*/
void Odom_PrintTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 72MHz / 72 = 1MHz, 1MHz / 1000 = 1kHz = 1ms 中断 */
    TIM_TimeBaseStructure.TIM_Period        = 1000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* NVIC 配置 —— 优先级比 USART1（0,2）低一些 */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

/* ===========================================================================
 * TIM2 中断服务 —— 每 1ms 一次
 *   1. g_local_tick_ms++ (给 WaitMoveDone 等用)
 *   2. 每 100ms (= 10Hz) 调一次 Odom_Print
 * ===========================================================================*/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        g_local_tick_ms++;

        /* 10Hz 打印 */
        if (g_print_odom_enable && (g_local_tick_ms % 100) == 0) {
            VelPos_Print();   /* 简化版默认打印 */
            // Odom_Print();  /* 全量版按需打开 */
        }
    }
}

/* ===========================================================================
 * 打印一行 VelPos v2 简化数据到 USART2（printf 已重定向到 USART2）
 * ===========================================================================*/
void VelPos_Print(void)
{
    VelPosData_t snap;
    snap.yaw_rad      = g_velpos.yaw_rad;
    snap.n_pos_x_m    = g_velpos.n_pos_x_m;
    snap.n_pos_y_m    = g_velpos.n_pos_y_m;
    snap.b_vel_x_mps  = g_velpos.b_vel_x_mps;
    snap.b_vel_y_mps  = g_velpos.b_vel_y_mps;
    snap.frame_count  = g_velpos.frame_count;

    if (snap.frame_count == 0) {
        printf("[VELPOS] (no data yet, send Cmd_Subscribe_VelPos?)\r\n");
        return;
    }

    printf("[VELPOS] Yaw=%7.2f X(N,fwd)=%7.3f Y(N,left)=%7.3f Vx(B,fwd)=%6.2f Vy(B,left)=%6.2f fr=%lu\r\n",
           (double)DFCOM_RAD2DEG(snap.yaw_rad),
           (double)snap.n_pos_x_m,
           (double)snap.n_pos_y_m,
           (double)snap.b_vel_x_mps,
           (double)snap.b_vel_y_mps,
           (unsigned long)snap.frame_count);
}

/* ===========================================================================
 * 打印一行 ODOM 数据到 USART2（printf 已重定向到 USART2）
 *   只在收到过至少一帧数据时才打印（避免开机刷一堆 0）
 * ===========================================================================*/
void Odom_Print(void)
{
    /* 取一份"瞬时快照"，避免中断期间被改写产生撕裂 */
    OdomData_t snap;
    /* volatile 复制：逐字段读，因为 g_odom 是 volatile */
    snap.yaw_rad      = g_odom.yaw_rad;
    snap.n_pos_x_m    = g_odom.n_pos_x_m;
    snap.n_pos_y_m    = g_odom.n_pos_y_m;
    snap.b_vel_x_mps  = g_odom.b_vel_x_mps;
    snap.b_vel_y_mps  = g_odom.b_vel_y_mps;
    snap.gyro_z_rps   = g_odom.gyro_z_rps;
    snap.frame_count  = g_odom.frame_count;

    if (snap.frame_count == 0) {
        printf("[ODOM] (no data yet, send Cmd_Subscribe_Odom?)\r\n");
        return;
    }

    /* 格式（ROS REP-103, +X 前 / +Y 左 / Yaw CCW+, deg）：
     *   [ODOM] Yaw=<度> X(fwd)=<m> Y(left)=<m> Vx(fwd)=<m/s> Vy(left)=<m/s> Gz=<rad/s> fr=<帧号>
     * 注意: 协议层 yaw 是 rad, 这里转 deg 仅为终端打印好看 (学生不一定懂弧度);
     *       内部闭环建议直接用 g_odom.yaw_rad 跟 dyaw_rad 对齐 (单位一致免转换). */
    printf("[ODOM] Yaw=%7.2f X(fwd)=%7.3f Y(left)=%7.3f Vx(fwd)=%6.2f Vy(left)=%6.2f Gz=%6.2f fr=%lu\r\n",
           (double)DFCOM_RAD2DEG(snap.yaw_rad),
           (double)snap.n_pos_x_m,
           (double)snap.n_pos_y_m,
           (double)snap.b_vel_x_mps,
           (double)snap.b_vel_y_mps,
           (double)snap.gyro_z_rps,
           (unsigned long)snap.frame_count);
}

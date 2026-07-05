/******************************************************************************
 * usart.c - USART1 / USART2 配置
 * ---------------------------------------------------------------------------
 *   USART1 (PA9/PA10) @ 460800
 *     - 与 DcarON 小车通信（DMA RX + DMA TX）
 *     - IDLE 中断里调 DFCom_RxParse 解析回传帧
 *
 *   USART2 (PA2/PA3)  @ 115200
 *     - printf 重定向到这里
 *     - 接 USB-TTL 接电脑串口工具看 ODOM 数据
 *
 *   USART3 / 无线模块 部分已删除（如需要可参考原 V0401 例程）
 ******************************************************************************/
#include "sys.h"
#include "usart.h"
#include "DFCom.h"

/* ===========================================================================
 * USART1 缓冲区
 * ===========================================================================*/
u8 USART1_RX_BUFF[USART1_RX_BUFF_SIZE];   /* 256B, 装得下双帧 + 余量, 见 usart.h */
u8 USART1_TX_BUFF[100];
u8 USART1_cnt;

/* ===========================================================================
 * printf 重定向 → USART2（看终端数据）
 *   注意：标准库下需要 #pragma import(__use_no_semihosting)
 *   Keil 项目里也要勾选 "Use MicroLIB" 才能正常工作
 * ===========================================================================*/
#if 1
#pragma import(__use_no_semihosting)
struct __FILE {
    int handle;
};
FILE __stdout;
void _sys_exit(int x) { x = x; }

int fputc(int ch, FILE *f)
{
    /* USART2 阻塞发送（printf 由 TIM2 中断调用，频率低，可以阻塞） */
    while ((USART2->SR & 0x40) == 0);  /* 等 TC（发送完成）标志 */
    USART2->DR = (u8)ch;
    return ch;
}
#endif

/* ===========================================================================
 * USART1 接收状态变量（保留兼容，新代码用 DMA IDLE）
 * ===========================================================================*/
#if EN_USART1_RX
u8  USART_RX_BUF[USART_REC_LEN];
u16 USART_RX_STA = 0;
#endif

/* ===========================================================================
 * USART1 初始化 (与 DcarON 通信)
 *   bound: 推荐 460800（DcarON 默认）
 * ===========================================================================*/
void uart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* PA9  - USART1_TX  - 复用推挽 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10 - USART1_RX  - 浮空输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* NVIC */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART 参数 */
    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    USART1_DMA_Rx_Configuration();
    USART_DMA_Tx_Configuration();
}

/* ===========================================================================
 * USART2 初始化 (printf 终端)
 *   bound: 推荐 115200
 * ===========================================================================*/
void usart2_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* USART2 挂在 APB1, GPIOA 在 APB2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* PA2 - USART2_TX  - 复用推挽 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 - USART2_RX  - 浮空输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART 参数 */
    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    /* 不用 USART2 接收，只 TX 输出 printf。如以后要 RX，加 NVIC + IRQHandler 即可 */
    USART_Cmd(USART2, ENABLE);
}

/* ===========================================================================
 * USART1 DMA RX 配置（DMA1_Channel5）
 * ===========================================================================*/
void USART1_DMA_Rx_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)USART1_RX_BUFF;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize         = USART1_RX_BUFF_SIZE;  /* 256, 见 usart.h */
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_ClearFlag(DMA1_FLAG_GL5);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);
}

/* ===========================================================================
 * USART1 DMA TX 配置（DMA1_Channel4）
 * ===========================================================================*/
void USART_DMA_Tx_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (USART1_BASE + 0x04);
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)USART1_TX_BUFF;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_BufferSize         = 100;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel4, DISABLE);
}

/* ===========================================================================
 * USART1 中断 (IDLE) - 收到 DcarON 一帧数据后触发
 *   1. 取 DMA 已收字节数
 *   2. 调 DFCom_RxParse 解析
 *   3. 重置 DMA
 * ===========================================================================*/
void USART1_IRQHandler(void)
{
    u16 size;   /* ★ buffer 256 字节, 必须 u16; u8 会在 size>=256 时溢出 */
    if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE) != RESET) {
        USART_ReceiveData(USART1);  /* 清 IDLE 标志（必须先读 DR） */

        DMA_Cmd(DMA1_Channel5, DISABLE);
        size = USART1_RX_BUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);

        if (size > 0) {
            /* ★ 协议解析（DFCom_Rx.c 提供） */
            DFCom_RxParse(USART1_RX_BUFF, size);

            /* 清空已用部分 */
            u16 j;
            for (j = 0; j < size; j++) USART1_RX_BUFF[j] = 0;
        }

        USART_ClearFlag(USART1, USART_FLAG_IDLE);
        DMA_SetCurrDataCounter(DMA1_Channel5, USART1_RX_BUFF_SIZE);
        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
}

/* ===========================================================================
 * USART1 DMA 发送（DFCom_Tx 用）
 *   返回: 0=成功, 1=失败/超时
 * ===========================================================================*/
u8 USART1_Send_By_DMA(u8 *data, u8 size)
{
    if (data == 0 || size == 0 || size > 100) return 1;

    /* 等上一次 DMA 发完（最多 10ms） */
    u32 timeout = 10000;
    while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET && timeout--);
    if (timeout == 0) {
        DMA_Cmd(DMA1_Channel4, DISABLE);
        return 1;
    }

    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);

    u8 i;
    for (i = 0; i < size; i++) USART1_TX_BUFF[i] = data[i];

    DMA1_Channel4->CMAR = (uint32_t)USART1_TX_BUFF;
    DMA_SetCurrDataCounter(DMA1_Channel4, size);
    DMA_Cmd(DMA1_Channel4, ENABLE);

    return 0;
}

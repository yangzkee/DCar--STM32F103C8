#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"

/* ===========================================================================
 * USART 配置说明
 *   USART1 (PA9/PA10) @ 460800 → 接 DcarON
 *   USART2 (PA2/PA3)  @ 115200 → 接 USB-TTL 看 printf 输出
 * ===========================================================================*/

#define USART_REC_LEN   200
#define EN_USART1_RX    1

extern u8  USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;

/* USART1（与 DcarON 通信） */
void uart_init(u32 bound);
void USART1_DMA_Rx_Configuration(void);
void USART_DMA_Tx_Configuration(void);
u8   USART1_Send_By_DMA(u8 *data, u8 size);
void USART1_IRQHandler(void);

/* ★ USART1 RX buffer 容量
 * ──────────────────────────────────────────────────────────────────────
 * 同时订阅 Odom + VelPos 时, 小车一个 tick 可能回传两帧
 * (Odom v4 60B + VelPos v2 44B = 104B 背靠背)。
 * 旧的 100 字节装不下, 后续帧尾部被 DMA 截断 → CRC 失败 → 整帧丢。
 * 现在扩到 256 字节, 能容下 4~5 帧粘连 + 余量。
 * 配套: usart.c 里 DMA_BufferSize, DMA_SetCurrDataCounter, size 类型都改 u16。
 * 配套: DFCom_Rx.c DFCom_RxParse(u8*, u16 size) 参数类型也改 u16。
 */
#define USART1_RX_BUFF_SIZE 256
extern u8 USART1_RX_BUFF[USART1_RX_BUFF_SIZE];
extern u8 USART1_TX_BUFF[100];
extern u8 USART1_cnt;

/* USART2（printf 终端） */
void usart2_init(u32 bound);

#endif

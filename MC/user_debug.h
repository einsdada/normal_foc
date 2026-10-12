#ifndef __USER_DEBUG_H
#define __USER_DEBUG_H

#include "stdint.h"


void Debug_Usart_Init(void);


/* --------------------------------- 调试TX --------------------------------- */

#define TX_BUF_LEN 	192     /* 发送缓冲：16 通道 CSV 一行最坏 ~113 字节（16 个数 + 15 逗号 + 换行），
                               CFG1/CFG2 那两行更长（~135）——192 是按最长的那两行留的余量
                               （uart_printf 用 vsnprintf 格式化，超长会被截断 → 破坏整行） */

// 函数声明
uint8_t uart_printf(const char *format, ...);   /* 返回 1 = 已交给 DMA 发送，0 = 上一次还没发完（丢弃本次） */
void uart_send_justfloat(float *data, uint16_t count);

/* --------------------------------- 调试RX --------------------------------- */
/* 接收命令后期开发：届时启用 HAL_UART_Receive_IT，此处理放单字节接收缓存
   （如 extern uint8_t rx_data;） */

#endif

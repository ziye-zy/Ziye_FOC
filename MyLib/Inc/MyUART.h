#ifndef __MYUART_H
#define __MYUART_H

#include "stm32g4xx_hal.h"
#include "usart.h"
#include <stdio.h>

/**
 * @brief   GCC环境下重定向 _write 函数到串口
 * @param  file  文件描述符（未使用）
 * @param  ptr   要发送的数据指针
 * @param  len   要发送的数据长度
 * @retval 成功发送的字节数
 */
int _write(int file, char *ptr, int len)
{
    // 通过HAL库函数将数据通过串口发出
    // HAL_MAX_DELAY 表示无限等待，直到发送完成
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 2);
    return len;
}


#endif
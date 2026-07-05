#include "uart.h"

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "clock.h"

#define UART_INST UART_0_INST

#define UART_TX_INTERVAL 100  // 发送间隔 (ms)
#define UART_TX_FIFO_SIZE 8
#define UART_TX_BUFFER_SIZE 64

char uart_tx_buffer[UART_TX_BUFFER_SIZE];
uint8_t uart_tx_index = 0;
uint8_t uart_tx_len = 0;
uint32_t last_uart_tx_time = 0;

extern float yaw,xun;
extern float TargetAngle,TargetSpeed,TargetLine;
extern volatile int16_t Speed_A, Speed_B;

void updateUARTData(void){
    if((tick_ms - last_uart_tx_time) >= UART_TX_INTERVAL && uart_tx_index == uart_tx_len){
        //格式化数据
        //uart_tx_len = snprintf(uart_tx_buffer, UART_TX_BUFFER_SIZE,
        //                       "%.1f,%.1f,%d,%d,%.1f\n",
        //                      yaw, TargetAngle, Speed_A, Speed_B, TargetSpeed);
        // uart_tx_len = snprintf(uart_tx_buffer, UART_TX_BUFFER_SIZE,
        //                        "%d,%d,%.1f\n",
        //                        Speed_A, Speed_B, TargetSpeed);
        uart_tx_len = snprintf(uart_tx_buffer, UART_TX_BUFFER_SIZE,
                               "%.1f,%.1f \n",
                               TargetLine, xun);
        uart_tx_index = 0;
        last_uart_tx_time = tick_ms;
    }
}

void uartNonBlockingSend(void) {
    // 如果还有数据未发送
    while (uart_tx_index < uart_tx_len) {
        if (DL_UART_transmitDataCheck(UART_INST, uart_tx_buffer[uart_tx_index])) {
            uart_tx_index++;
        } else {
            // FIFO满，退出等待下次轮询
            break;
        }
    }
}

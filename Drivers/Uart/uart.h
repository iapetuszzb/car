#ifndef _UART_H
#define _UART_H

#include <stdbool.h>
#include <stdint.h>

void Bluetooth_Init(void);
void Bluetooth_Task(void);
void Bluetooth_SendString(const char *text);
void Bluetooth_UART_IRQHandler(void);
uint32_t Bluetooth_GetRxByteCount(void);
uint32_t Bluetooth_GetTxByteCount(void);
uint32_t Bluetooth_GetCommandCount(void);
uint32_t Bluetooth_GetDroppedCommandCount(void);
uint32_t Bluetooth_GetRxErrorCount(void);
void Bluetooth_GetRecentAscii(char *buffer, uint32_t buffer_len);
bool Bluetooth_IsConnectedRecent(void);
bool Bluetooth_MotionIsActive(void);
bool Bluetooth_MotionControlTick10ms(int16_t encoder_delta_a,
                                     int16_t encoder_delta_b);

void updateUARTData(void);
void uartNonBlockingSend(void);

#endif

#ifndef _BUTTON_H
#define _BUTTON_H

#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *ButtonPort;      // GPIO端口
    uint32_t ButtonPin;         // GPIO引脚
    uint8_t ButtonEvent;        // 事件类型：0未按，1短按，2长按

    // 以下为内部变量（状态判断用）
    uint8_t isPressed;          // 当前是否处于按下状态
    uint8_t lastPinState;       // 上一次读取电平状态
    unsigned long pressTick;    // 按下时间
} button_t;

// 初始化
void Button_Init(button_t *button, GPIO_Regs *PORT, uint32_t PIN);

// 获取事件值（短按/长按等）
uint8_t Button_ReadState(button_t *button);

// 基于轮询获取状态（建议10ms周期调用）
void Button_Update(button_t *button);

//基于中断获取状态
void Button_UpdateFromInterrupt(button_t *button, uint8_t pinState);

#endif

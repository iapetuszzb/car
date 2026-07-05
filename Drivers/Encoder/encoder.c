#include "encoder.h"

void ReadEncoder(GPIO_Regs *INVC_PORT, uint32_t INVC_PIN,
                 GPIO_Regs *GPIO_PORT, uint32_t GPIO_PIN,
                 volatile int *EncoderCount) {
    // 获取中断状态
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(INVC_PORT, INVC_PIN);

    if (status & INVC_PIN) {
        // 判断B相信号电平
        if (DL_GPIO_readPins(GPIO_PORT, GPIO_PIN)) {
            (*EncoderCount)++;  // 顺时针
        } else {
            (*EncoderCount)--;  // 逆时针
        }
        // 清除中断标志
        DL_GPIO_clearInterruptStatus(INVC_PORT, INVC_PIN);
    }
}

int ReadSpeed(volatile int *EncoderCount) {
    int Speed = *EncoderCount;
    *EncoderCount = 0;
    return Speed;
}
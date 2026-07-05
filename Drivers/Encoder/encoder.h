#ifndef _ENCODER_H
#define _ENCODER_H

#include "ti_msp_dl_config.h"

void ReadEncoder(GPIO_Regs *INVC_PORT, uint32_t INVC_PIN,
                 GPIO_Regs *GPIO_PORT, uint32_t GPIO_PIN,
                 volatile int *EncoderCount);
int ReadSpeed(volatile int *EncoderCount);

#endif
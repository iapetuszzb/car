/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"
#include "main.h"

#include "clock.h"
#include "interrupt.h"

#include "mpu6500.h"
#include "motor.h"
#include "pid.h"
#include "uart.h"
#include "menu.h"
#include "oled_software_i2c.h"
#include "stepper_gimbal.h"
#include "k210_face.h"
#include <stdbool.h>

#define STEPPER_GIMBAL_RUN_TEST_ON_BOOT 0

bool start = false;
uint8_t Circle_Count, turncount, paramSelcet;
float TargetSpeed, TargetLine, basespeed;
pid_t pidMotorA, pidMotorB, pidLine;
float P, I, D;

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    OLED_Init();
    (void)MPU6500_Init();
    Motor_Init();
    StepperGimbal_Init();
    K210Face_Init();

#if STEPPER_GIMBAL_RUN_TEST_ON_BOOT
    StepperGimbal_TestSmallMove();
#endif

    DL_TimerG_startCounter(TIMER_0_INST);
    Interrupt_Init();

    pid_init(&pidMotorA, DELTA_PID, 18, 35, 0);
    pid_init(&pidMotorB, DELTA_PID, 18, 35, 0);

    P = 900.0f;
    I = 0;
    D = 300.0f;
    pid_init(&pidLine, POSITION_PID, P, I, D);

    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    Load(0, 0);

    basespeed = 10.0f;
    TargetSpeed = basespeed;
    TargetLine = 4.5f;
    Circle_Count = 1;
    turncount = 0;

    while (1) {
        K210Face_Task();
        menu();
    }
}

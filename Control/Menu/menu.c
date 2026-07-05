#include "menu.h"
#include "oled_software_i2c.h"
#include "clock.h"
#include "pid.h"
#include "mpu6500.h"
#include "k210_face.h"
#include "stepper_gimbal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

extern float P, I, D;
extern uint8_t Circle_Count, turncount;
extern bool start;
extern pid_t pidLine;

void menu(void)
{
    char oled_Buffer[20];
    static unsigned long last_update_ms = 0;
    unsigned long now_ms = 0;

    (void)mspm0_get_clock_ms(&now_ms);
    if ((unsigned long)(now_ms - last_update_ms) < 100UL) {
        return;
    }
    last_update_ms = now_ms;

    (void)Read_MPU6500();

    snprintf(oled_Buffer, sizeof(oled_Buffer), "turn=%u S=%u   ",
             (unsigned int)turncount, start ? 1U : 0U);
    OLED_ShowString(0, 0, (uint8_t *)oled_Buffer, 16);

    snprintf(oled_Buffer, sizeof(oled_Buffer), "n=%u          ",
             (unsigned int)Circle_Count);
    OLED_ShowString(0, 2, (uint8_t *)oled_Buffer, 16);

    snprintf(oled_Buffer, sizeof(oled_Buffer), "R=%lu B=%02X ",
             (unsigned long)K210Face_GetRxByteCount(),
             K210Face_GetLastByte());
    OLED_ShowString(0, 4, (uint8_t *)oled_Buffer, 16);

    snprintf(oled_Buffer, sizeof(oled_Buffer), "L=%lu F=%lu E=%lu",
             (unsigned long)K210Face_GetRxLineCount(),
             (unsigned long)K210Face_GetValidCount(),
             (unsigned long)K210Face_GetParseErrorCount());
    OLED_ShowString(0, 6, (uint8_t *)oled_Buffer, 8);

    snprintf(oled_Buffer, sizeof(oled_Buffer), "EX%ld EY%ld A%ld",
             (long)K210Face_GetLastErrorX(),
             (long)K210Face_GetLastErrorY(),
             (long)StepperGimbal_GetPositionSteps(STEPPER_GIMBAL_MOTOR_A));
    OLED_ShowString(0, 7, (uint8_t *)oled_Buffer, 8);

    pid_init(&pidLine, POSITION_PID, P, I, D);
}

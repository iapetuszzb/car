#include "menu.h"
#include "oled_software_i2c.h"
#include "clock.h"
#include "pid.h"
#include "mpu6500.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

extern float P, I, D;
extern pid_t pidLine;

static void format_yaw(char *buffer, uint32_t buffer_len, float value)
{
    long yaw_milli;
    long yaw_int;
    long yaw_frac;

    if (value >= 0.0f) {
        yaw_milli = (long)(value * 1000.0f + 0.5f);
    } else {
        yaw_milli = (long)(value * 1000.0f - 0.5f);
    }

    yaw_int = yaw_milli / 1000L;
    yaw_frac = yaw_milli % 1000L;
    if (yaw_frac < 0) {
        yaw_frac = -yaw_frac;
    }

    (void)snprintf(buffer, buffer_len, "yaw:%ld.%03ld   ",
                   yaw_int, yaw_frac);
}

void menu(void)
{
    char oled_Buffer[20];
    static bool oled_cleared = false;
    static unsigned long last_update_ms = 0;
    unsigned long now_ms = 0;
    uint8_t init_status;
    uint8_t read_status;

    if (!oled_cleared) {
        OLED_Clear();
        oled_cleared = true;
    }

    (void)mspm0_get_clock_ms(&now_ms);
    if ((unsigned long)(now_ms - last_update_ms) < 100UL) {
        return;
    }
    last_update_ms = now_ms;

    init_status = MPU6500_GetInitStatus();
    read_status = MPU6500_GetLastReadStatus();

    if (init_status != 0U) {
        (void)snprintf(oled_Buffer, sizeof(oled_Buffer), "mpu err:%u   ",
                       (unsigned int)init_status);
    } else if ((MPU6500_GetUpdateCount() == 0U) && (read_status != 0U)) {
        (void)snprintf(oled_Buffer, sizeof(oled_Buffer), "mpu wait:%u  ",
                       (unsigned int)read_status);
    } else {
        format_yaw(oled_Buffer, sizeof(oled_Buffer), yaw);
    }
    OLED_ShowString(0, 0, (uint8_t *)oled_Buffer, 16);

    pid_init(&pidLine, POSITION_PID, P, I, D);
}

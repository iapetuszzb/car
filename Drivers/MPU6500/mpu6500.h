#ifndef _MPU6500_H_
#define _MPU6500_H_

#include <stdint.h>

extern short gyro[3], accel[3];
extern float pitch, roll, yaw;

uint8_t MPU6500_Init(void);
uint8_t MPU6500_ReadID(void);
uint8_t Read_MPU6500(void);
float MPU6500_GetYawRateDps(void);
uint8_t MPU6500_GetInitStatus(void);
uint8_t MPU6500_GetLastReadStatus(void);
uint32_t MPU6500_GetUpdateCount(void);

#endif  /* #ifndef _MPU6500_H_ */

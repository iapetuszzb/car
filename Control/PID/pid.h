#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include <stdbool.h>

/*
*位置式(角度环)
*P 快速性->到达目标值的速度
*I 准确性->消除静态误差
*D 稳定性->抑制震荡
*/

/*
*增量式(速度环)
*P 稳定性->抑制震荡
*I 快速性->到达目标值的速度
*D 准确性->消除静态误差
*/

enum
{
  POSITION_PID = 0, //位置式
  DELTA_PID,        //增量式
};

typedef struct
{
	float target;	
	float now;
	float error[3];		
	float p,i,d;
	float pout, dout, iout;
	float out;   
	
	uint32_t pid_mode;  //pid模式

}pid_t;


//void pid_control_angle(float TargetAngle, float TargetSpeed, bool enable_angle_control);
void pid_control_line(float TargetLine, float TargetSpeed);
void pid_turn_only(void);
float aWheel_pid_control(pid_t* motor, float target, float feedback);

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void PID_Reset(pid_t *pid);
void pid_cal(pid_t *pid);
void PID_Limit(pid_t *pid);

#endif

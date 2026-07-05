#ifndef _MOTOR_H
#define _MOTOR_H

//分频系数  2
//PSC      1
//PWM Period count 40000
//上下限(-40000,40000)

void Motor_Init(void);
int abs(int x);
void Load(int motoA, int motoB);
void Limit(int *MotoA, int *MotoB);

#endif
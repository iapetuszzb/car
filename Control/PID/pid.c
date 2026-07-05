#include "pid.h"
#include "motor.h"
#include "IR_Module.h"

extern pid_t pidMotorA, pidMotorB, pidLine;
extern volatile int16_t Speed_A, Speed_B;

#define MAX_DUTY 24000
#define MIN_DUTY -24000
#define BASE_SPEED_TO_PWM 1500
#define MIN_RUN_PWM 12000
#define START_BOOST_PWM 12000
#define START_BOOST_TICKS 30
#define LINE_TURN_PWM_LIMIT 5000


static int clamp_int(int value, int min, int max)
{
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
    pid->pid_mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;
}

void PID_Reset(pid_t *pid)
{
    pid->target = 0;
    pid->now = 0;
    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->error[2] = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

void pid_control_line(float TargetLine, float TargetSpeed)
{
    int base_pwm;
    int turn_pwm;
    int right_pwm;
    int left_pwm;

    pidLine.target = TargetLine;
    pidLine.now = getLine();
    pid_cal(&pidLine);
    PID_Limit(&pidLine);

    base_pwm = (int)(TargetSpeed * BASE_SPEED_TO_PWM);
    base_pwm = clamp_int(base_pwm, MIN_RUN_PWM, MAX_DUTY - LINE_TURN_PWM_LIMIT);
    turn_pwm = clamp_int((int)pidLine.out, -LINE_TURN_PWM_LIMIT, LINE_TURN_PWM_LIMIT);

    /* Motor A is right wheel, motor B is left wheel. */
    right_pwm = base_pwm + turn_pwm;
    left_pwm = base_pwm - turn_pwm;
    Load(right_pwm, left_pwm);

    pidMotorA.target = right_pwm;
    pidMotorB.target = left_pwm;
    pidMotorA.now = Speed_A;
    pidMotorB.now = Speed_B;
}

void pid_turn_only(void)
{
    pidMotorA.target = 0.8f;
    pidMotorB.target = 2.6f;
    pidMotorA.now = Speed_A;
    pidMotorB.now = Speed_B;
    pid_cal(&pidMotorA);
    pid_cal(&pidMotorB);
    PID_Limit(&pidMotorA);
    PID_Limit(&pidMotorB);
    Load((int)pidMotorA.out, (int)pidMotorB.out);
}

float aWheel_pid_control(pid_t *motor, float target, float feedback)
{
    motor->target = target;
    motor->now = feedback;
    pid_cal(motor);
    PID_Limit(motor);
    return motor->out;
}

void pid_cal(pid_t *pid)
{
    pid->error[0] = pid->target - pid->now;

    if(pid->pid_mode == DELTA_PID) {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
    } else if(pid->pid_mode == POSITION_PID) {
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

void PID_Limit(pid_t *pid)
{
    if(pid->out >= MAX_DUTY) pid->out = MAX_DUTY;
    if(pid->out <= MIN_DUTY) pid->out = MIN_DUTY;
}

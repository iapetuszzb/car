#include "pid.h"
#include "motor.h"
#include "IR_Module.h"

extern pid_t pidMotorA, pidMotorB, pidLine;
extern volatile int16_t Speed_A, Speed_B;

#define MAX_DUTY 32000
#define MIN_DUTY -32000
#define BASE_SPEED_TO_PWM 1500
#define LINE_TURN_PWM_LIMIT 18000
#define LINE_EXTREME_ERROR_THRESHOLD 2.25f
#define LINE_EXTREME_TURN_BOOST 5000
#define LINE_STOP_BLACK_COUNT 5U
#define LINE_STOP_CONFIRM_TICKS 2U
#define LINE_STOP_RAMP_PWM_PER_TICK 2500

static bool corner_turn_active = false;
static ir_outer_direction_t corner_mark = IR_OUTER_NONE;
static bool line_stop_armed = false;
static bool line_stop_active = false;
static bool line_stop_complete = false;
static uint8_t line_stop_confirm_ticks = 0U;
static int last_right_pwm = 0;
static int last_left_pwm = 0;
static int stop_right_pwm = 0;
static int stop_left_pwm = 0;

static int clamp_int(int value, int min, int max)
{
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int ramp_pwm_toward_zero(int pwm)
{
    if(pwm > LINE_STOP_RAMP_PWM_PER_TICK) {
        return pwm - LINE_STOP_RAMP_PWM_PER_TICK;
    }
    if(pwm < -LINE_STOP_RAMP_PWM_PER_TICK) {
        return pwm + LINE_STOP_RAMP_PWM_PER_TICK;
    }
    return 0;
}

static void command_line_motors(int right_pwm, int left_pwm)
{
    right_pwm = clamp_int(right_pwm, MIN_DUTY, MAX_DUTY);
    left_pwm = clamp_int(left_pwm, MIN_DUTY, MAX_DUTY);
    Load(right_pwm, left_pwm);

    last_right_pwm = right_pwm;
    last_left_pwm = left_pwm;
    pidMotorA.target = right_pwm;
    pidMotorB.target = left_pwm;
    pidMotorA.now = Speed_A;
    pidMotorB.now = Speed_B;
}

static void apply_corner_turn(float target_speed)
{
    int corner_pwm;
    int right_pwm;
    int left_pwm;

    /* Use the same PWM as straight running, but reverse the inner wheel. */
    corner_pwm = (int)(target_speed * BASE_SPEED_TO_PWM);
    corner_pwm = clamp_int(corner_pwm, 0, MAX_DUTY);

    if(corner_mark == IR_OUTER_LEFT) {
        right_pwm = corner_pwm;
        left_pwm = -corner_pwm;
    } else {
        right_pwm = -corner_pwm;
        left_pwm = corner_pwm;
    }

    command_line_motors(right_pwm, left_pwm);
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

void PID_LineControlReset(void)
{
    corner_turn_active = false;
    corner_mark = IR_OUTER_NONE;
    line_stop_armed = false;
    line_stop_active = false;
    line_stop_complete = false;
    line_stop_confirm_ticks = 0U;
    last_right_pwm = 0;
    last_left_pwm = 0;
    stop_right_pwm = 0;
    stop_left_pwm = 0;
}

bool PID_LineStopIsComplete(void)
{
    return line_stop_complete;
}

bool PID_LineStopIsActive(void)
{
    return line_stop_active;
}

void PID_LineSetStopArmed(bool armed)
{
    line_stop_armed = armed;
    if(!armed) {
        line_stop_confirm_ticks = 0U;
    }
}

void pid_control_line(float TargetLine, float TargetSpeed)
{
    float line_now;
    bool raw_line_lost;
    float line_error_abs;
    int base_pwm;
    int turn_pwm;
    int turn_pwm_limit;
    int right_pwm;
    int left_pwm;
    uint8_t black_count;
    ir_outer_direction_t outer_direction;

    line_now = getLine();
    raw_line_lost = IR_LineLost();
    black_count = IR_GetBlackCount();

    if(line_stop_armed && (black_count >= LINE_STOP_BLACK_COUNT)) {
        if(line_stop_confirm_ticks < LINE_STOP_CONFIRM_TICKS) {
            line_stop_confirm_ticks++;
        }
    } else {
        line_stop_confirm_ticks = 0U;
    }

    if(!line_stop_active &&
       (line_stop_confirm_ticks >= LINE_STOP_CONFIRM_TICKS)) {
        line_stop_active = true;
        line_stop_complete = false;
        stop_right_pwm = last_right_pwm;
        stop_left_pwm = last_left_pwm;
        corner_turn_active = false;
        corner_mark = IR_OUTER_NONE;
        PID_Reset(&pidLine);
    }

    if(line_stop_active) {
        stop_right_pwm = ramp_pwm_toward_zero(stop_right_pwm);
        stop_left_pwm = ramp_pwm_toward_zero(stop_left_pwm);
        command_line_motors(stop_right_pwm, stop_left_pwm);
        if((stop_right_pwm == 0) && (stop_left_pwm == 0)) {
            line_stop_complete = true;
        }
        return;
    }

    if(corner_turn_active) {
        if(raw_line_lost) {
            apply_corner_turn(TargetSpeed);
            return;
        }

        /* The first black sample ends the corner turn and resumes line PID. */
        corner_turn_active = false;
        corner_mark = IR_OUTER_NONE;
        PID_Reset(&pidLine);
    } else {
        outer_direction = IR_GetOuterDirection();
        if(outer_direction != IR_OUTER_NONE) {
            corner_mark = outer_direction;
        }
    }

    /* A marked outer receiver followed by white starts an in-place turn. */
    if(raw_line_lost) {
        PID_Reset(&pidLine);
        if(corner_mark != IR_OUTER_NONE) {
            corner_turn_active = true;
            apply_corner_turn(TargetSpeed);
        } else {
            command_line_motors(0, 0);
        }
        return;
    }

    pidLine.target = TargetLine;
    pidLine.now = line_now;
    pid_cal(&pidLine);
    PID_Limit(&pidLine);

    base_pwm = (int)(TargetSpeed * BASE_SPEED_TO_PWM);
    base_pwm = clamp_int(base_pwm, 0, MAX_DUTY);
    turn_pwm_limit = (base_pwm < LINE_TURN_PWM_LIMIT) ?
                     base_pwm : LINE_TURN_PWM_LIMIT;
    turn_pwm = clamp_int((int)pidLine.out,
                         -turn_pwm_limit,
                         turn_pwm_limit);
    line_error_abs = abs_float(pidLine.error[0]);

    if(line_error_abs >= LINE_EXTREME_ERROR_THRESHOLD) {
        if(turn_pwm > 0) {
            turn_pwm += LINE_EXTREME_TURN_BOOST;
        } else if(turn_pwm < 0) {
            turn_pwm -= LINE_EXTREME_TURN_BOOST;
        }
        turn_pwm = clamp_int(turn_pwm,
                             -turn_pwm_limit,
                             turn_pwm_limit);
    }

    /* Motor A is right wheel, motor B is left wheel. */
    right_pwm = clamp_int(base_pwm + turn_pwm, MIN_DUTY, MAX_DUTY);
    left_pwm = clamp_int(base_pwm - turn_pwm, MIN_DUTY, MAX_DUTY);
    command_line_motors(right_pwm, left_pwm);
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

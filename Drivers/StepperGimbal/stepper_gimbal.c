#include "stepper_gimbal.h"

#include "clock.h"
#include "ti_msp_dl_config.h"

#define STEPPER_GIMBAL_TIMER_CLK_HZ       (1000000U)
#define STEPPER_GIMBAL_MIN_STEP_HZ        (20U)
#define STEPPER_GIMBAL_MAX_STEP_HZ        (4000U)
#define STEPPER_GIMBAL_START_STEP_HZ      (300U)
#define STEPPER_GIMBAL_RAMP_INTERVAL_100US (8U)
#define STEPPER_GIMBAL_TEST_STEP_HZ       (120U)
#define STEPPER_GIMBAL_TEST_MOTOR_A_STEPS (12)
#define STEPPER_GIMBAL_TEST_MOTOR_B_STEPS (8)

#define STEPPER_GIMBAL_USE_SYSTICK_STEP   (1)

#define STEPPER_GIMBAL_MOTOR_A_DIR_INVERT (0)
#define STEPPER_GIMBAL_MOTOR_B_DIR_INVERT (0)

#define MOTOR_A_STEP_PORT                 (GPIOA)
#define MOTOR_A_STEP_PIN                  (DL_GPIO_PIN_26)
#define MOTOR_A_STEP_IOMUX                (IOMUX_PINCM59)
#define MOTOR_A_DIR_PORT                  (GPIOB)
#define MOTOR_A_DIR_PIN                   (DL_GPIO_PIN_19)
#define MOTOR_A_DIR_IOMUX                 (IOMUX_PINCM45)
#define MOTOR_A_EN_PORT                   (GPIOB)
#define MOTOR_A_EN_PIN                    (DL_GPIO_PIN_18)
#define MOTOR_A_EN_IOMUX                  (IOMUX_PINCM44)
#define MOTOR_A_TIMER                     (TIMG8)
#define MOTOR_A_TIMER_IRQN                (TIMG8_INT_IRQn)

#define MOTOR_B_STEP_PORT                 (GPIOB)
#define MOTOR_B_STEP_PIN                  (DL_GPIO_PIN_24)
#define MOTOR_B_STEP_IOMUX                (IOMUX_PINCM52)
#define MOTOR_B_DIR_PORT                  (GPIOA)
#define MOTOR_B_DIR_PIN                   (DL_GPIO_PIN_18)
#define MOTOR_B_DIR_IOMUX                 (IOMUX_PINCM40)
#define MOTOR_B_EN_PORT                   (GPIOA)
#define MOTOR_B_EN_PIN                    (DL_GPIO_PIN_27)
#define MOTOR_B_EN_IOMUX                  (IOMUX_PINCM60)
#define MOTOR_B_TIMER                     (TIMG12)
#define MOTOR_B_TIMER_IRQN                (TIMG12_INT_IRQn)

typedef struct {
    StepperGimbalState state;
    GPTIMER_Regs *timer;
    IRQn_Type irq;
    GPIO_Regs *step_port;
    uint32_t step_pin;
    IOMUX_PINCM step_iomux;
    GPIO_Regs *dir_port;
    uint32_t dir_pin;
    IOMUX_PINCM dir_iomux;
    GPIO_Regs *en_port;
    uint32_t en_pin;
    IOMUX_PINCM en_iomux;
    uint8_t dir_invert;
    volatile uint32_t remaining_toggles;
    volatile bool step_is_high;
    volatile uint16_t tick_half_period_100us;
    volatile uint16_t target_half_period_100us;
    volatile uint16_t tick_elapsed_100us;
    volatile uint8_t ramp_elapsed_100us;
    volatile bool target_mode;
    volatile bool velocity_mode;
    volatile int8_t velocity_direction;
    volatile int32_t target_position_steps;
} StepperGimbalAxis;

static StepperGimbalAxis g_axes[2] = {
    {
        .state = {0U, 0U, 0, false, 1},
        .timer = MOTOR_A_TIMER,
        .irq = MOTOR_A_TIMER_IRQN,
        .step_port = MOTOR_A_STEP_PORT,
        .step_pin = MOTOR_A_STEP_PIN,
        .step_iomux = MOTOR_A_STEP_IOMUX,
        .dir_port = MOTOR_A_DIR_PORT,
        .dir_pin = MOTOR_A_DIR_PIN,
        .dir_iomux = MOTOR_A_DIR_IOMUX,
        .en_port = MOTOR_A_EN_PORT,
        .en_pin = MOTOR_A_EN_PIN,
        .en_iomux = MOTOR_A_EN_IOMUX,
        .dir_invert = STEPPER_GIMBAL_MOTOR_A_DIR_INVERT,
        .remaining_toggles = 0U,
        .step_is_high = false,
        .tick_half_period_100us = 1U,
        .target_half_period_100us = 1U,
        .tick_elapsed_100us = 0U,
        .ramp_elapsed_100us = 0U,
        .target_mode = false,
        .velocity_mode = false,
        .velocity_direction = 1,
        .target_position_steps = 0,
    },
    {
        .state = {0U, 0U, 0, false, 1},
        .timer = MOTOR_B_TIMER,
        .irq = MOTOR_B_TIMER_IRQN,
        .step_port = MOTOR_B_STEP_PORT,
        .step_pin = MOTOR_B_STEP_PIN,
        .step_iomux = MOTOR_B_STEP_IOMUX,
        .dir_port = MOTOR_B_DIR_PORT,
        .dir_pin = MOTOR_B_DIR_PIN,
        .dir_iomux = MOTOR_B_DIR_IOMUX,
        .en_port = MOTOR_B_EN_PORT,
        .en_pin = MOTOR_B_EN_PIN,
        .en_iomux = MOTOR_B_EN_IOMUX,
        .dir_invert = STEPPER_GIMBAL_MOTOR_B_DIR_INVERT,
        .remaining_toggles = 0U,
        .step_is_high = false,
        .tick_half_period_100us = 1U,
        .target_half_period_100us = 1U,
        .tick_elapsed_100us = 0U,
        .ramp_elapsed_100us = 0U,
        .target_mode = false,
        .velocity_mode = false,
        .velocity_direction = 1,
        .target_position_steps = 0,
    },
};

static bool g_hold_after_move = true;
static bool g_initialized = false;

static StepperGimbalAxis *axis_from_motor(StepperGimbalMotor motor)
{
    if (motor == STEPPER_GIMBAL_MOTOR_A) {
        return &g_axes[0];
    }
    if (motor == STEPPER_GIMBAL_MOTOR_B) {
        return &g_axes[1];
    }
    return 0;
}

static void init_axis_pins(const StepperGimbalAxis *axis)
{
    DL_GPIO_initDigitalOutput(axis->step_iomux);
    DL_GPIO_clearPins(axis->step_port, axis->step_pin);
    DL_GPIO_enableOutput(axis->step_port, axis->step_pin);

    DL_GPIO_initDigitalOutput(axis->dir_iomux);
    DL_GPIO_clearPins(axis->dir_port, axis->dir_pin);
    DL_GPIO_enableOutput(axis->dir_port, axis->dir_pin);

    DL_GPIO_initDigitalOutput(axis->en_iomux);
    DL_GPIO_clearPins(axis->en_port, axis->en_pin);
    DL_GPIO_enableOutput(axis->en_port, axis->en_pin);
}

static void init_axis_timer(const StepperGimbalAxis *axis)
{
#if STEPPER_GIMBAL_USE_SYSTICK_STEP
    (void)axis;
#else
    static const DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
        .prescale = 9U,
    };
    static const DL_TimerG_TimerConfig timer_config = {
        .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
        .period = (STEPPER_GIMBAL_TIMER_CLK_HZ / (STEPPER_GIMBAL_MIN_STEP_HZ * 2U)) - 1U,
        .startTimer = DL_TIMER_STOP,
        .genIntermInt = DL_TIMER_INTERM_INT_DISABLED,
        .counterVal = 0U,
    };

    DL_TimerG_enablePower(axis->timer);
    DL_TimerG_reset(axis->timer);
    DL_TimerG_setClockConfig(axis->timer, &clock_config);
    DL_TimerG_initTimerMode(axis->timer, &timer_config);
    DL_TimerG_enableInterrupt(axis->timer, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(axis->timer);
    NVIC_SetPriority(axis->irq, 3);
    NVIC_EnableIRQ(axis->irq);
#endif
}

static uint32_t clamp_step_hz(uint32_t step_hz)
{
    if (step_hz < STEPPER_GIMBAL_MIN_STEP_HZ) {
        return STEPPER_GIMBAL_MIN_STEP_HZ;
    }
    if (step_hz > STEPPER_GIMBAL_MAX_STEP_HZ) {
        return STEPPER_GIMBAL_MAX_STEP_HZ;
    }
    return step_hz;
}

static uint32_t step_hz_to_period(uint32_t step_hz)
{
    uint32_t half_period_ticks;

    step_hz = clamp_step_hz(step_hz);
    half_period_ticks = STEPPER_GIMBAL_TIMER_CLK_HZ / (step_hz * 2U);
    if (half_period_ticks == 0U) {
        half_period_ticks = 1U;
    }
    return half_period_ticks - 1U;
}

static uint16_t step_hz_to_half_period_100us(uint32_t step_hz)
{
    uint32_t half_period_ticks;

    step_hz = clamp_step_hz(step_hz);
    half_period_ticks = (10000U + ((step_hz * 2U) - 1U)) /
                        (step_hz * 2U);
    if (half_period_ticks == 0U) {
        half_period_ticks = 1U;
    }
    if (half_period_ticks > 65535U) {
        half_period_ticks = 65535U;
    }
    return (uint16_t)half_period_ticks;
}

static uint16_t start_ramp_half_period_100us(uint16_t target_half_period)
{
    uint16_t start_half_period =
        step_hz_to_half_period_100us(STEPPER_GIMBAL_START_STEP_HZ);

    return (target_half_period > start_half_period) ?
           target_half_period : start_half_period;
}

static void update_axis_speed_ramp(StepperGimbalAxis *axis)
{
    if (axis->tick_half_period_100us == axis->target_half_period_100us) {
        axis->ramp_elapsed_100us = 0U;
        return;
    }

    axis->ramp_elapsed_100us++;
    if (axis->ramp_elapsed_100us < STEPPER_GIMBAL_RAMP_INTERVAL_100US) {
        return;
    }
    axis->ramp_elapsed_100us = 0U;

    if (axis->tick_half_period_100us > axis->target_half_period_100us) {
        axis->tick_half_period_100us--;
    } else {
        axis->tick_half_period_100us++;
    }
}

static uint32_t abs_i32_to_u32(int32_t value)
{
    if (value < 0) {
        return (uint32_t)(-value);
    }
    return (uint32_t)value;
}

static int32_t angle_to_steps(float angle_deg)
{
    float steps = angle_deg / STEPPER_GIMBAL_ANGLE_PER_STEP;

    if (steps >= 0.0f) {
        return (int32_t)(steps + 0.5f);
    }
    return (int32_t)(steps - 0.5f);
}

static void set_axis_direction(StepperGimbalAxis *axis, int8_t direction)
{
    bool high = (direction > 0);

    if (axis->dir_invert) {
        high = !high;
    }

    if (high) {
        DL_GPIO_setPins(axis->dir_port, axis->dir_pin);
    } else {
        DL_GPIO_clearPins(axis->dir_port, axis->dir_pin);
    }

    axis->state.direction = direction;
}

static void stop_axis(StepperGimbalAxis *axis)
{
#if !STEPPER_GIMBAL_USE_SYSTICK_STEP
    DL_TimerG_stopCounter(axis->timer);
#endif
    axis->remaining_toggles = 0U;
    axis->step_is_high = false;
    axis->target_mode = false;
    axis->velocity_mode = false;
    axis->state.is_moving = false;
    axis->ramp_elapsed_100us = 0U;
    DL_GPIO_clearPins(axis->step_port, axis->step_pin);

    if (!g_hold_after_move) {
        DL_GPIO_clearPins(axis->en_port, axis->en_pin);
    }
}

static void step_axis_once(StepperGimbalAxis *axis)
{
    if (!axis->state.is_moving || (axis->remaining_toggles == 0U)) {
        stop_axis(axis);
        return;
    }

    if (axis->step_is_high) {
        DL_GPIO_clearPins(axis->step_port, axis->step_pin);
        axis->step_is_high = false;
    } else {
        DL_GPIO_setPins(axis->step_port, axis->step_pin);
        axis->step_is_high = true;
        axis->state.done_steps++;
        axis->state.position_steps += axis->state.direction;
    }

    axis->remaining_toggles--;
    if (axis->remaining_toggles == 0U) {
        stop_axis(axis);
    }
}

static void step_axis_toward_target(StepperGimbalAxis *axis)
{
    int32_t error;

    if (!axis->state.is_moving) {
        stop_axis(axis);
        return;
    }

    error = axis->target_position_steps - axis->state.position_steps;
    if (error == 0) {
        stop_axis(axis);
        return;
    }

    if (axis->step_is_high) {
        DL_GPIO_clearPins(axis->step_port, axis->step_pin);
        axis->step_is_high = false;
        return;
    }

    set_axis_direction(axis, (error > 0) ? 1 : -1);
    DL_GPIO_setPins(axis->step_port, axis->step_pin);
    axis->step_is_high = true;
    axis->state.done_steps++;
    axis->state.position_steps += axis->state.direction;
}

static void step_axis_velocity(StepperGimbalAxis *axis)
{
    if (!axis->state.is_moving || !axis->velocity_mode) {
        stop_axis(axis);
        return;
    }

    if (axis->step_is_high) {
        DL_GPIO_clearPins(axis->step_port, axis->step_pin);
        axis->step_is_high = false;
        return;
    }

    set_axis_direction(axis, axis->velocity_direction);
    DL_GPIO_setPins(axis->step_port, axis->step_pin);
    axis->step_is_high = true;
    axis->state.done_steps++;
    axis->state.position_steps += axis->state.direction;
}

static void axis_periodic_irq(StepperGimbalAxis *axis)
{
#if STEPPER_GIMBAL_USE_SYSTICK_STEP
    (void)axis;
#else
    switch (DL_TimerG_getPendingInterrupt(axis->timer)) {
        case DL_TIMERG_IIDX_ZERO:
            if (axis->target_mode) {
                step_axis_toward_target(axis);
            } else if (axis->velocity_mode) {
                step_axis_velocity(axis);
            } else {
                step_axis_once(axis);
            }
            break;

        default:
            break;
    }
#endif
}

void StepperGimbal_Init(void)
{
    init_axis_pins(&g_axes[0]);
    init_axis_pins(&g_axes[1]);
    init_axis_timer(&g_axes[0]);
    init_axis_timer(&g_axes[1]);
    StepperGimbal_StopAll();
    StepperGimbal_Enable(STEPPER_GIMBAL_MOTOR_A, true);
    StepperGimbal_Enable(STEPPER_GIMBAL_MOTOR_B, true);
    g_initialized = true;
}

void StepperGimbal_Enable(StepperGimbalMotor motor, bool enable)
{
    StepperGimbalAxis *axis = axis_from_motor(motor);

    if (axis == 0) {
        return;
    }

    if (enable) {
        DL_GPIO_setPins(axis->en_port, axis->en_pin);
    } else {
        DL_GPIO_clearPins(axis->en_port, axis->en_pin);
    }
}

void StepperGimbal_SetHoldAfterMove(bool hold)
{
    g_hold_after_move = hold;
}

bool StepperGimbal_MoveSteps(StepperGimbalMotor motor, int32_t steps,
                             uint32_t step_hz)
{
    StepperGimbalAxis *axis = axis_from_motor(motor);
    uint32_t step_count;
#if !STEPPER_GIMBAL_USE_SYSTICK_STEP
    uint32_t period;
#endif

    if (!g_initialized) {
        StepperGimbal_Init();
    }

    if ((axis == 0) || axis->state.is_moving) {
        return false;
    }

    if (steps == 0) {
        return true;
    }

    step_count = abs_i32_to_u32(steps);
#if !STEPPER_GIMBAL_USE_SYSTICK_STEP
    period = step_hz_to_period(step_hz);
#endif

    set_axis_direction(axis, (steps > 0) ? 1 : -1);
    StepperGimbal_Enable(motor, true);

    axis->target_mode = false;
    axis->velocity_mode = false;
    axis->state.target_steps = step_count;
    axis->state.done_steps = 0U;
    axis->remaining_toggles = step_count * 2U;
    axis->step_is_high = false;
    DL_GPIO_clearPins(axis->step_port, axis->step_pin);

#if STEPPER_GIMBAL_USE_SYSTICK_STEP
    axis->target_half_period_100us = step_hz_to_half_period_100us(step_hz);
    axis->tick_half_period_100us =
        start_ramp_half_period_100us(axis->target_half_period_100us);
    axis->tick_elapsed_100us = 0U;
    axis->ramp_elapsed_100us = 0U;
    axis->state.is_moving = true;
#else
    axis->state.is_moving = true;
    DL_TimerG_stopCounter(axis->timer);
    DL_TimerG_setLoadValue(axis->timer, period);
    DL_TimerG_setTimerCount(axis->timer, period);
    DL_TimerG_clearInterruptStatus(axis->timer, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_startCounter(axis->timer);
#endif

    return true;
}

bool StepperGimbal_SetVelocity(StepperGimbalMotor motor,
                               int32_t signed_step_hz)
{
    StepperGimbalAxis *axis = axis_from_motor(motor);
    uint32_t speed_hz;
    uint16_t half_period_100us;
    int8_t target_direction;
    bool starting;
    bool direction_change;

    if (!g_initialized) {
        StepperGimbal_Init();
    }

    if (axis == 0) {
        return false;
    }

    if (signed_step_hz == 0) {
        stop_axis(axis);
        return true;
    }

    target_direction = (signed_step_hz > 0) ? 1 : -1;
    speed_hz = (signed_step_hz > 0) ?
               (uint32_t)signed_step_hz : (uint32_t)(-signed_step_hz);
    speed_hz = clamp_step_hz(speed_hz);
    half_period_100us = step_hz_to_half_period_100us(speed_hz);

    starting = !axis->state.is_moving || !axis->velocity_mode;
    direction_change =
        axis->state.is_moving && (axis->state.direction != target_direction);

    StepperGimbal_Enable(motor, true);

    axis->target_mode = false;
    axis->velocity_mode = true;
    axis->velocity_direction = target_direction;
    axis->target_half_period_100us = half_period_100us;
    axis->state.target_steps = 0U;

    if (starting) {
        axis->state.done_steps = 0U;
        axis->remaining_toggles = 0U;
        axis->tick_half_period_100us =
            start_ramp_half_period_100us(axis->target_half_period_100us);
        axis->tick_elapsed_100us = 0U;
        axis->ramp_elapsed_100us = 0U;
        axis->step_is_high = false;
        DL_GPIO_clearPins(axis->step_port, axis->step_pin);
    } else if (direction_change) {
        axis->tick_half_period_100us =
            start_ramp_half_period_100us(axis->target_half_period_100us);
        axis->ramp_elapsed_100us = 0U;
    }

    if (!axis->step_is_high) {
        set_axis_direction(axis, target_direction);
    }

    axis->state.is_moving = true;

    return true;
}

bool StepperGimbal_MoveToSteps(StepperGimbalMotor motor, int32_t target_steps,
                               uint32_t step_hz)
{
    StepperGimbalAxis *axis = axis_from_motor(motor);
#if !STEPPER_GIMBAL_USE_SYSTICK_STEP
    uint32_t period;
#endif
    uint16_t half_period_100us;
    bool starting;
    bool direction_change;
    int8_t target_direction;

    if (!g_initialized) {
        StepperGimbal_Init();
    }

    if (axis == 0) {
        return false;
    }

    half_period_100us = step_hz_to_half_period_100us(step_hz);
#if !STEPPER_GIMBAL_USE_SYSTICK_STEP
    period = step_hz_to_period(step_hz);
#endif
    starting = !axis->state.is_moving || !axis->target_mode;
    target_direction = (target_steps > axis->state.position_steps) ? 1 : -1;
    direction_change =
        axis->state.is_moving && (axis->state.direction != target_direction);

    axis->target_position_steps = target_steps;
    axis->target_half_period_100us = half_period_100us;
    axis->target_mode = true;
    axis->velocity_mode = false;
    axis->state.target_steps =
        abs_i32_to_u32(target_steps - axis->state.position_steps);

    StepperGimbal_Enable(motor, true);

    if (target_steps == axis->state.position_steps) {
        stop_axis(axis);
        return true;
    }

    if (starting) {
        axis->state.done_steps = 0U;
        axis->remaining_toggles = 0U;
        axis->tick_half_period_100us =
            start_ramp_half_period_100us(axis->target_half_period_100us);
        axis->tick_elapsed_100us = 0U;
        axis->ramp_elapsed_100us = 0U;
        axis->step_is_high = false;
        DL_GPIO_clearPins(axis->step_port, axis->step_pin);
    } else if (direction_change) {
        axis->tick_half_period_100us =
            start_ramp_half_period_100us(axis->target_half_period_100us);
        axis->ramp_elapsed_100us = 0U;
    }

    if (!axis->step_is_high) {
        set_axis_direction(axis, target_direction);
    }

    axis->state.is_moving = true;

#if !STEPPER_GIMBAL_USE_SYSTICK_STEP
    DL_TimerG_setLoadValue(axis->timer, period);
    if (starting) {
        DL_TimerG_stopCounter(axis->timer);
        DL_TimerG_setTimerCount(axis->timer, period);
        DL_TimerG_clearInterruptStatus(axis->timer,
                                       DL_TIMERG_INTERRUPT_ZERO_EVENT);
        DL_TimerG_startCounter(axis->timer);
    }
#endif

    return true;
}

bool StepperGimbal_MoveAngle(StepperGimbalMotor motor, float angle_deg,
                             uint32_t step_hz)
{
    return StepperGimbal_MoveSteps(motor, angle_to_steps(angle_deg), step_hz);
}

void StepperGimbal_Stop(StepperGimbalMotor motor)
{
    StepperGimbalAxis *axis = axis_from_motor(motor);

    if (axis != 0) {
        stop_axis(axis);
    }
}

void StepperGimbal_StopAll(void)
{
    stop_axis(&g_axes[0]);
    stop_axis(&g_axes[1]);
}

bool StepperGimbal_IsBusy(StepperGimbalMotor motor)
{
    StepperGimbalAxis *axis = axis_from_motor(motor);

    return (axis != 0) ? axis->state.is_moving : false;
}

void StepperGimbal_Tick100us(void)
{
#if STEPPER_GIMBAL_USE_SYSTICK_STEP
    uint32_t i;

    if (!g_initialized) {
        return;
    }

    for (i = 0U; i < 2U; i++) {
        StepperGimbalAxis *axis = &g_axes[i];

        if (!axis->state.is_moving) {
            continue;
        }

        update_axis_speed_ramp(axis);
        axis->tick_elapsed_100us++;
        if (axis->tick_elapsed_100us >= axis->tick_half_period_100us) {
            axis->tick_elapsed_100us = 0U;
            if (axis->target_mode) {
                step_axis_toward_target(axis);
            } else if (axis->velocity_mode) {
                step_axis_velocity(axis);
            } else {
                step_axis_once(axis);
            }
        }
    }
#endif
}

void StepperGimbal_Tick1ms(void)
{
    uint8_t i;

    for (i = 0U; i < 10U; i++) {
        StepperGimbal_Tick100us();
    }
}

int32_t StepperGimbal_GetPositionSteps(StepperGimbalMotor motor)
{
    StepperGimbalAxis *axis = axis_from_motor(motor);

    return (axis != 0) ? axis->state.position_steps : 0;
}

float StepperGimbal_GetAngleDeg(StepperGimbalMotor motor)
{
    return (float)StepperGimbal_GetPositionSteps(motor) *
           STEPPER_GIMBAL_ANGLE_PER_STEP;
}

const StepperGimbalState *StepperGimbal_GetState(StepperGimbalMotor motor)
{
    StepperGimbalAxis *axis = axis_from_motor(motor);

    return (axis != 0) ? &axis->state : 0;
}

static void wait_until_idle(StepperGimbalMotor motor, uint32_t timeout_ms)
{
    unsigned long start;
    unsigned long now;

    (void)mspm0_get_clock_ms(&start);
    while (StepperGimbal_IsBusy(motor)) {
        (void)mspm0_get_clock_ms(&now);
        if ((uint32_t)(now - start) >= timeout_ms) {
            StepperGimbal_Stop(motor);
            break;
        }
    }
}

void StepperGimbal_TestSmallMove(void)
{
    StepperGimbal_SetHoldAfterMove(true);

    (void)StepperGimbal_MoveSteps(STEPPER_GIMBAL_MOTOR_A,
                                  STEPPER_GIMBAL_TEST_MOTOR_A_STEPS,
                                  STEPPER_GIMBAL_TEST_STEP_HZ);
    wait_until_idle(STEPPER_GIMBAL_MOTOR_A, 1000U);
    (void)mspm0_delay_ms(150U);
    (void)StepperGimbal_MoveSteps(STEPPER_GIMBAL_MOTOR_A,
                                  -STEPPER_GIMBAL_TEST_MOTOR_A_STEPS,
                                  STEPPER_GIMBAL_TEST_STEP_HZ);
    wait_until_idle(STEPPER_GIMBAL_MOTOR_A, 1000U);
    (void)mspm0_delay_ms(150U);

    (void)StepperGimbal_MoveSteps(STEPPER_GIMBAL_MOTOR_B,
                                  STEPPER_GIMBAL_TEST_MOTOR_B_STEPS,
                                  STEPPER_GIMBAL_TEST_STEP_HZ);
    wait_until_idle(STEPPER_GIMBAL_MOTOR_B, 1000U);
    (void)mspm0_delay_ms(150U);
    (void)StepperGimbal_MoveSteps(STEPPER_GIMBAL_MOTOR_B,
                                  -STEPPER_GIMBAL_TEST_MOTOR_B_STEPS,
                                  STEPPER_GIMBAL_TEST_STEP_HZ);
    wait_until_idle(STEPPER_GIMBAL_MOTOR_B, 1000U);
}

void TIMG8_IRQHandler(void)
{
    axis_periodic_irq(&g_axes[0]);
}

void TIMG12_IRQHandler(void)
{
    axis_periodic_irq(&g_axes[1]);
}

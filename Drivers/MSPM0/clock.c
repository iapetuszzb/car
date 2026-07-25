#include "ti_msp_dl_config.h"
#include "clock.h"

volatile unsigned long tick_ms;
volatile unsigned long tick_100us;
volatile uint32_t start_time;

#define SYSTICK_PERIOD_US       (100UL)
#define CPU_CYCLES_PER_US       (CPUCLK_FREQ / 1000000UL)

int mspm0_delay_ms(unsigned long num_ms)
{
    start_time = tick_ms;
    while (tick_ms - start_time < num_ms);
    return 0;
}

int mspm0_delay_us(unsigned long num_us)
{
    unsigned long start_us;
    unsigned long now_us;

    if (mspm0_get_clock_us(&start_us) != 0) {
        return 1;
    }

    do {
        if (mspm0_get_clock_us(&now_us) != 0) {
            return 1;
        }
    } while ((unsigned long)(now_us - start_us) < num_us);

    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}

int mspm0_get_clock_us(unsigned long *count)
{
    unsigned long tick_a;
    unsigned long tick_b;
    uint32_t systick_val;
    uint32_t reload_cycles;
    uint32_t elapsed_cycles;
    uint32_t elapsed_us;

    if (!count) {
        return 1;
    }

    do {
        tick_a = tick_100us;
        systick_val = SysTick->VAL;
        tick_b = tick_100us;
    } while (tick_a != tick_b);

    reload_cycles = SysTick->LOAD + 1U;
    elapsed_cycles = reload_cycles - systick_val;
    if (elapsed_cycles >= reload_cycles) {
        elapsed_cycles = 0U;
    }

    elapsed_us = elapsed_cycles / CPU_CYCLES_PER_US;
    if (elapsed_us >= SYSTICK_PERIOD_US) {
        elapsed_us = SYSTICK_PERIOD_US - 1UL;
    }

    count[0] = tick_a * SYSTICK_PERIOD_US + elapsed_us;
    return 0;
}

void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/10000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

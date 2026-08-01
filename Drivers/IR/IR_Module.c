#include "IR_Module.h"
#include "stdbool.h"

bool turnMark = false;
static bool lineLost = true;
static uint8_t blackCount = 0U;
float xun = 4.5f, lastxun = 4.5f;
int line0, line1, line2, line3, line4, line5, line6, line7;

/* The tracking module outputs high on black and low on white. */
#define IR_BLACK_ACTIVE_LEVEL 1U
#define IR_ALL_PINS (IR_DH_PIN_16_PIN | IR_DH_PIN_0_PIN | \
                     IR_DH_PIN_6_PIN | IR_DH_PIN_7_PIN | \
                     IR_DH_PIN_8_PIN | IR_DH_PIN_15_PIN | \
                     IR_DH_PIN_17_PIN | IR_DH_PIN_12_PIN)

static int IR_BlackFromSnapshot(uint32_t snapshot, uint32_t pin)
{
    uint32_t raw_level = (snapshot & pin) ? 1U : 0U;
    return (raw_level == IR_BLACK_ACTIVE_LEVEL) ? 1 : 0;
}

float getLine(void)
{
    uint32_t snapshot;

    blackCount = 0U;
    snapshot = DL_GPIO_readPins(IR_DH_PORT, IR_ALL_PINS);

    line0 = IR_BlackFromSnapshot(snapshot, IR_DH_PIN_16_PIN);
    line1 = IR_BlackFromSnapshot(snapshot, IR_DH_PIN_0_PIN);
    line2 = IR_BlackFromSnapshot(snapshot, IR_DH_PIN_6_PIN);
    line3 = IR_BlackFromSnapshot(snapshot, IR_DH_PIN_7_PIN);
    line4 = IR_BlackFromSnapshot(snapshot, IR_DH_PIN_8_PIN);
    line5 = IR_BlackFromSnapshot(snapshot, IR_DH_PIN_15_PIN);
    line6 = IR_BlackFromSnapshot(snapshot, IR_DH_PIN_17_PIN);
    line7 = IR_BlackFromSnapshot(snapshot, IR_DH_PIN_12_PIN);

    if(line0 == 1) blackCount++;
    if(line1 == 1) blackCount++;
    if(line2 == 1) blackCount++;
    if(line3 == 1) blackCount++;
    if(line4 == 1) blackCount++;
    if(line5 == 1) blackCount++;
    if(line6 == 1) blackCount++;
    if(line7 == 1) blackCount++;

    lineLost = (blackCount == 0U);

    if(blackCount > 0U) {
        xun = (line0 * 1 + line1 * 2 + line2 * 3 + line3 * 4 +
               line4 * 5 + line5 * 6 + line6 * 7 + line7 * 8) /
              (float)blackCount;
        lastxun = xun;
    } else {
        xun = 4.5f;
    }

    turnMark = false;
    return xun;
}

bool IR_LineLost(void)
{
    return lineLost;
}

uint8_t IR_GetBlackCount(void)
{
    return blackCount;
}

ir_outer_direction_t IR_GetOuterDirection(void)
{
    if((line0 == 1) && (line7 == 0)) {
        return IR_OUTER_LEFT;
    }
    if((line7 == 1) && (line0 == 0)) {
        return IR_OUTER_RIGHT;
    }
    return IR_OUTER_NONE;
}

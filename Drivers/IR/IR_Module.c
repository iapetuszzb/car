#include "IR_Module.h"
#include "stdbool.h"

bool turnMark = false;
static bool lineLost = true;
float xun = 4.5f, lastxun = 4.5f, z;
int line0, line1, line2, line3, line4, line5, line6, line7;

/* The tracking module outputs high on black and low on white. */
#define IR_BLACK_ACTIVE_LEVEL 1U

static int IR_ReadBlack(uint32_t pin)
{
    uint32_t raw_level = DL_GPIO_readPins(IR_DH_PORT, pin) ? 1U : 0U;
    return (raw_level == IR_BLACK_ACTIVE_LEVEL) ? 1 : 0;
}

float getLine(void)
{
    z = 0;

    line0 = IR_ReadBlack(IR_DH_PIN_16_PIN);
    line1 = IR_ReadBlack(IR_DH_PIN_0_PIN);
    line2 = IR_ReadBlack(IR_DH_PIN_6_PIN);
    line3 = IR_ReadBlack(IR_DH_PIN_7_PIN);
    line4 = IR_ReadBlack(IR_DH_PIN_8_PIN);
    line5 = IR_ReadBlack(IR_DH_PIN_15_PIN);
    line6 = IR_ReadBlack(IR_DH_PIN_17_PIN);
    line7 = IR_ReadBlack(IR_DH_PIN_12_PIN);

    if(line0 == 1) z++;
    if(line1 == 1) z++;
    if(line2 == 1) z++;
    if(line3 == 1) z++;
    if(line4 == 1) z++;
    if(line5 == 1) z++;
    if(line6 == 1) z++;
    if(line7 == 1) z++;

    lineLost = (z <= 0);

    if(z > 0) {
        xun = (line0 * 1 + line1 * 2 + line2 * 3 + line3 * 4 +
               line4 * 5 + line5 * 6 + line6 * 7 + line7 * 8) / z;
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

#include "IR_Module.h"
#include "stdbool.h"

bool turnMark = false;
float xun = 4.5f, lastxun = 4.5f, z;
int line0, line1, line2, line3, line4, line5, line6, line7;

float getLine(void)
{
    z = 0;

    line0 = DL_GPIO_readPins(IR_DH_PORT, IR_DH_PIN_16_PIN) ? 1 : 0;
    line1 = DL_GPIO_readPins(IR_DH_PORT, IR_DH_PIN_0_PIN) ? 1 : 0;
    line2 = DL_GPIO_readPins(IR_DH_PORT, IR_DH_PIN_6_PIN) ? 1 : 0;
    line3 = DL_GPIO_readPins(IR_DH_PORT, IR_DH_PIN_7_PIN) ? 1 : 0;
    line4 = DL_GPIO_readPins(IR_DH_PORT, IR_DH_PIN_8_PIN) ? 1 : 0;
    line5 = DL_GPIO_readPins(IR_DH_PORT, IR_DH_PIN_15_PIN) ? 1 : 0;
    line6 = DL_GPIO_readPins(IR_DH_PORT, IR_DH_PIN_17_PIN) ? 1 : 0;
    line7 = DL_GPIO_readPins(IR_DH_PORT, IR_DH_PIN_12_PIN) ? 1 : 0;

    if(line0 == 1) z++;
    if(line1 == 1) z++;
    if(line2 == 1) z++;
    if(line3 == 1) z++;
    if(line4 == 1) z++;
    if(line5 == 1) z++;
    if(line6 == 1) z++;
    if(line7 == 1) z++;

    if(z > 0) {
        xun = (line0 * 1 + line1 * 2 + line2 * 3 + line3 * 4 +
               line4 * 5 + line5 * 6 + line6 * 7 + line7 * 8) / z;
        lastxun = xun;
    } else {
        xun = lastxun;
    }

    turnMark = false;
    return xun;
}

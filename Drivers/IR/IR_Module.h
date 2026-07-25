#ifndef _IR_MODULE_H
#define _IR_MODULE_H
#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

typedef enum {
    IR_OUTER_NONE = 0,
    IR_OUTER_LEFT,
    IR_OUTER_RIGHT,
} ir_outer_direction_t;

void IRDM_line_inspection(void);
float angle_wrap(float angle);
float angle_diff(float target, float current);
float getLine(void);
bool IR_LineLost(void);
ir_outer_direction_t IR_GetOuterDirection(void);
void diffControl(void);

#endif

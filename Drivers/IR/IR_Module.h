#ifndef _IR_MODULE_H
#define _IR_MODULE_H
#include "ti_msp_dl_config.h"

void IRDM_line_inspection(void);
float angle_wrap(float angle);
float angle_diff(float target, float current);
float getLine(void);
void diffControl(void);

#endif

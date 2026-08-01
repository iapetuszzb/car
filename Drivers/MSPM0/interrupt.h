#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include <stdbool.h>

void Interrupt_Init(void);
bool LineRunTimer_HasStarted(void);
bool LineRunTimer_IsRunning(void);
unsigned long LineRunTimer_GetElapsedMs(void);

#endif  /* #ifndef _INTERRUPT_H_ */

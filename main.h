#ifndef _MAIN_H_
#define _MAIN_H_

/* Keep the former gimbal implementation available, but do not configure or
 * drive any of its pins in the current car build. */
#define APP_ENABLE_STEPPER_GIMBAL 0
#define APP_ENABLE_SERVO_SWEEP    0
#define APP_ENABLE_VISION_UART    0

#endif  /* #ifndef _MAIN_H_ */

#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>

#define PWM_CHANNELS 5
volatile extern uint16_t pwm_desired[PWM_CHANNELS];

#endif

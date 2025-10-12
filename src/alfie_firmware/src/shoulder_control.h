#ifndef _SHOULDER_CONTROL_H_
#define _SHOULDER_CONTROL_H_

#include <Arduino.h>

void setupShoulderLimitSwitch();
bool readShoulderLimitSwitch();

#define SHOULDER_LIMIT_SWITCH_PIN 34


#endif // _SHOULDER_CONTROL_H_



#include <Arduino.h>
#include "shoulder_control.h"


void setupShoulderLimitSwitch()
{
  pinMode(SHOULDER_LIMIT_SWITCH_PIN, INPUT);
}

bool readShoulderLimitSwitch()
{
  // previously returned active-low (LOW == triggered)
  // Reverse the output so the function reports the opposite logic.
  return digitalRead(SHOULDER_LIMIT_SWITCH_PIN) == HIGH;
}
#include <Arduino.h>
#include "back_control.h"


void setupBackLimitSwitch()
{
  pinMode(BACK_LIMIT_SWITCH_PIN, INPUT);
}

bool readBackLimitSwitch()
{
  // previously returned active-low (LOW == triggered)
  // Reverse the output so the function reports the opposite logic.
  return digitalRead(BACK_LIMIT_SWITCH_PIN) == HIGH;
}
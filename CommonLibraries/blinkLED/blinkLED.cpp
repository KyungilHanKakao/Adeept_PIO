#include "Arduino.h"
#include "blinkLED.h"


void blinkLED(int pin, int duration)
{
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW); 
  delay(duration);
}
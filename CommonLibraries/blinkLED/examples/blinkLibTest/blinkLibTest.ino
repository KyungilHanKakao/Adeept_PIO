#include "Arduino.h"
#include "blinkLED.h"

const int BLINK_SHORT = 250;
const int BLINK_MEDIUM = 500;
const int BLINK_LONG = 1000;

const int firstLedPin  = 13;
const int secondLedPin = 5;
const int thirdLedPin  = 6;
void setup()
{
    pinMode(firstLedPin, OUTPUT);
    pinMode(secondLedPin, OUTPUT);
    pinMode(thirdLedPin, OUTPUT);
    
}

void loop()
{
    blinkLED(firstLedPin, BLINK_SHORT);
    blinkLED(secondLedPin, BLINK_MEDIUM);
    blinkLED(thirdLedPin, BLINK_LONG);

}
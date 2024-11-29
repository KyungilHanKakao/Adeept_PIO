#include <Arduino.h>

# define PIN_LED  13  // Define LED pin as 13.

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  delay(10);
}

void loop()
{
  digitalWrite(PIN_LED, LOW); // LED on.
  Serial.println("LED on");
  delay(1000); // delay 1s.
  digitalWrite(PIN_LED, HIGH);   // LED off.
  Serial.println("LED off");
  delay(2000); // delay 1s.
}

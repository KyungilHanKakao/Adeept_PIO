#include <Arduino.h>
#include "Adeept_Buzzer_For_Arduino.h"

void setup() {
  Buzzer_Setup();    //Buzzer initialization function
  // Buzzer_Alert(beat, rebeat)
  // beat: How many times the buzzer sounds at a time.
  // rebeat: Cycle through several times.
  Buzzer_Alert(4, 3);//Control the buzzer to sound 2 times, 4 sounds each time
}

void loop() {
  
}

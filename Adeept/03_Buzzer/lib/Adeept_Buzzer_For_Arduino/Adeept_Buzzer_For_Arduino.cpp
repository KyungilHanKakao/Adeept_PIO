#include <Arduino.h>
#include "Adeept_Buzzer_For_Arduino.h"

//////////////////////Buzzer drive area///////////////////////////////////
void Buzzer_Setup(void){
  pinMode(PIN_BUZZER, OUTPUT);
  delay(10);
}

//Buzzer alarm function
void Buzzer_Alert(int beat, int rebeat)
{
  beat = constrain(beat, 1, 9);
  rebeat = constrain(rebeat, 1, 255);
  for (int j = 0; j < rebeat; j++){
    for (int i = 0; i < beat; i++){
      tone(PIN_BUZZER, BUZZER_FREQUENCY);
      delay(100);
      noTone(PIN_BUZZER); //stop vocalizing
      delay(100);
    }
    delay(500);
  }
}

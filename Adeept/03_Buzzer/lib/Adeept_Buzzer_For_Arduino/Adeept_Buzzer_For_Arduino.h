#include <Arduino.h>

//////////////////////Buzzer drive area///////////////////////////////////
//Buzzer pin definition             
#define PIN_BUZZER 3                    //Define the pins for the Arduino control buzzer
// #define BUZZER_CHN 0                    //Define the PWM channel for 
#define BUZZER_FREQUENCY 2000           //Define the resonant frequency of the buzzer 

void Buzzer_Setup(void);                //Buzzer initialization
void Buzzer_Alert(int beat, int rebeat);//Buzzer alarm function

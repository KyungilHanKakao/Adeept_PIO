/**********************************************************************
  Filename    : Tracking_Line_module.ino
  Auther      : www.adeept.com
  Modification: 2023/10/09
**********************************************************************/
#include <Arduino.h>

#define Tracking_Left   4 //define Tracking left port
#define Tracking_Mid    5 //define Tracking mid port
#define Tracking_Right  6 //define Tracking right port
int Val[3] = {0,0,0};
int value = 0;
void setup() {
  Serial.begin(115200); //set baud rate.
  
  pinMode(Tracking_Left, INPUT); //define input port
  pinMode(Tracking_Mid, INPUT); 
  pinMode(Tracking_Right, INPUT);
}

void loop() {
  Val[0] = digitalRead(Tracking_Left); // Get the value of D4 and save it in an array.
  Val[1] = digitalRead(Tracking_Mid);  // Get the value of D5 and save it in an array.
  Val[2] = digitalRead(Tracking_Right); // Get the value of D6 and save it in an array.
  Serial.print("Sensor Value (L/M/R): ");
  for (int i = 0; i < 3; i++) {
    Serial.print(Val[i]); // 
    Serial.print(' ');
  }
  Serial.print('\n');
  delay(500);
}


void Track_Read(void){
  Val[0] = digitalRead(Tracking_Left); //Read the value of digital interface 4 and assign it to val[0]
  Val[1] = digitalRead(Tracking_Mid); 
  Val[2] = digitalRead(Tracking_Right); 
  if ((Val[0]==0)&&(Val[1]==0)&&(Val[2]==0)){ // 000
    value = 0;
    }
  else if ((Val[0]==0)&&(Val[1]==1)&&(Val[2]==0)){ // 010
    value = 1;
    }
  else if ((Val[0]==1)&&(Val[1]==0)&&(Val[2]==0)){ // 100
    value = 2;
    }
  else if ((Val[0]==1)&&(Val[1]==1)&&(Val[2]==0)){ // 110
    value = 3;
    }
  else if ((Val[0]==0)&&(Val[1]==0)&&(Val[2]==1)){ // 001
    value = 4;
    }
  else if ((Val[0]==0)&&(Val[1]==1)&&(Val[2]==1)){ // 011
    value = 5;
    }
  else if ((Val[0]==1)&&(Val[1]==1)&&(Val[2]==1)){ // 111
    value = 6;
    }
  return;
}

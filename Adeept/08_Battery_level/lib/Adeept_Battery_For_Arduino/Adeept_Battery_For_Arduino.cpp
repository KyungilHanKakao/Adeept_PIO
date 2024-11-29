#include <Arduino.h>
#include "Adeept_Battery_For_Arduino.h"

////////////////////Battery drive area/////////////////////////////////////
float batteryVoltage = 0;       //Battery voltage variable
float batteryCoefficient = 4;   //Set the proportional coefficient

//Gets the battery ADC value
int Get_Battery_Voltage_ADC(void)
{
  pinMode(PIN_BATTERY, INPUT);
  int batteryADC = 0;
  for (int i = 0; i < 5; i++)
    batteryADC += analogRead(PIN_BATTERY);
  return batteryADC / 5;
}

//Get the battery voltage value
float Get_Battery_Voltage(void)
{
  int batteryADC = Get_Battery_Voltage_ADC();
  // batteryVoltage = (batteryADC / 512.0  * 3.3 )*batteryCoefficient;
  batteryVoltage = (batteryADC / 512.0  *5 *2 );
  if(batteryVoltage >= 8.4){
    batteryVoltage = 8.4;
    }
  return batteryVoltage;
}

// void Set_Battery_Coefficient(float coefficient)
// { 
//   batteryCoefficient = coefficient;
// }

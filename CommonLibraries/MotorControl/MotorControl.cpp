#include "MotorControl.h"

Adafruit_PWMServoDriver pwm_motor = Adafruit_PWMServoDriver(0x5F);

void motorPWM(int channel, int motor_speed){
   motor_speed = constrain(motor_speed, 0, 255);
   int motor_pwm = map(motor_speed, 0, 255, 0, 4095);
   if (motor_pwm == 4095){
      pwm_motor.setPWM(channel, 4096, 0);
   }
   else if (motor_pwm == 0){
      pwm_motor.setPWM(channel, 0, 4096);
   }
   else{
      pwm_motor.setPWM(channel, 0, motor_pwm);
      // pwm_motor.setPWM(channel, 0, 4095 - motor_pwm);
   }
}



void Motor(int Motor_ID, int dir, int Motor_speed){
   if(dir > 0)
   {
      dir = 1;
   }
   else if (dir < 0) 
   {
      dir = -1;
   }
   else {
      dir = 0;
   }

   if (Motor_ID == 1){
      if (dir == 1){
         motorPWM(PIN_MOTOR_M1_IN1, 0);
         motorPWM(PIN_MOTOR_M1_IN2, Motor_speed);
         
         // digitalWrite(PIN_MOTOR_M1_IN1, HIGH);
         // digitalWrite(PIN_MOTOR_M1_IN2, LOW);
      }
      else if (dir == -1){
         motorPWM(PIN_MOTOR_M1_IN1, Motor_speed);
         motorPWM(PIN_MOTOR_M1_IN2, 0);
         // digitalWrite(PIN_MOTOR_M1_IN1, LOW);
         // digitalWrite(PIN_MOTOR_M1_IN2, HIGH);
         // Serial.println("Motor M1 Backward");
      }
      else {
         motorPWM(PIN_MOTOR_M1_IN1, 0);
         motorPWM(PIN_MOTOR_M1_IN2, 0);
         // digitalWrite(PIN_MOTOR_M1_IN1, LOW);
         // digitalWrite(PIN_MOTOR_M1_IN2, LOW);
         // Serial.println("Motor M1 STOP");
      }
   }
   else if (Motor_ID == 2){
      if (dir == 1){
         motorPWM(PIN_MOTOR_M2_IN1, Motor_speed);
         motorPWM(PIN_MOTOR_M2_IN2, 0);
         // digitalWrite(PIN_MOTOR_M2_IN1, HIGH);
         // digitalWrite(PIN_MOTOR_M2_IN2, LOW);
         // Serial.println("Motor M2 Forward");
      }
      else if (dir == -1){
         motorPWM(PIN_MOTOR_M2_IN1, 0);
         motorPWM(PIN_MOTOR_M2_IN2, Motor_speed);
         // digitalWrite(PIN_MOTOR_M2_IN1, LOW);
         // digitalWrite(PIN_MOTOR_M2_IN2, HIGH);
         // Serial.println("Motor M2 Backward");
      }
      else {
         motorPWM(PIN_MOTOR_M2_IN1, 0);
         motorPWM(PIN_MOTOR_M2_IN2, 0);
         // digitalWrite(PIN_MOTOR_M2_IN1, LOW);
         // digitalWrite(PIN_MOTOR_M2_IN2, LOW);
         // Serial.println("Motor M2 STOP");
      }
   }
}

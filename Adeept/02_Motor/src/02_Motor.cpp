#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "Adafruit_PWMServoDriver.h"


#define PIN_MOTOR_M1_IN1 6      //Define the positive pole of M1
#define PIN_MOTOR_M1_IN2 7      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 8      //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 9      //Define the negative pole of M2

// called this way, PCA9685 uses the address 0x5F.
Adafruit_PWMServoDriver pwm_motor = Adafruit_PWMServoDriver(0x5F);
void Motor(int Motor_ID, int dir, int Motor_speed);
void setup() {
  pwm_motor.begin();
  pwm_motor.setPWMFreq(1600);  // Set PWM frequency to 50Hz.

  Serial.begin(9600);
}

void loop() {
  //Motor(motor_ID, direction, speed)
  // motor_ID: Motor number, 1-2(M1~M2)
  // direction: Motor rotation direction. 1 or -1.
  // speed: Motor speed. 0-100.
  Motor(1, 1, 50); // M1, forward rotation, fast rotation.
  Motor(2, 1, 50);  // M2, forward rotation,low rotation.
  delay(2000);      // delay 2s.
  
  Motor(1, 0, 0);   // stop 1s.
  Motor(2, 0, 0);
  delay(1000);

  Motor(1,-1, 50);  // reverse rotation 2s.
  Motor(2, -1, 50);
  delay(2000);

  Motor(1, 0, 0);    // stop 1s.
  Motor(2, 0, 0);
  delay(1000);

}

// Convert motor speed to PWM value.
void motorPWM(int channel, int motor_speed){
  motor_speed = constrain(motor_speed, 0, 50);
  int motor_pwm = map(motor_speed, 0, 50, 0, 4095);
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

// Control motor rotation.
void Motor(int Motor_ID, int dir, int Motor_speed){
  if(dir > 0){dir = 1;}
  else if (dir < 0) {dir = -1;}
  else {dir = 0;}

  if (Motor_ID == 1){
    if (dir == 1){
      motorPWM(PIN_MOTOR_M1_IN1, 0);
      motorPWM(PIN_MOTOR_M1_IN2, Motor_speed);
      Serial.println("Motor M1 1");
    }
    else if (dir == -1){
      motorPWM(PIN_MOTOR_M1_IN1, Motor_speed);
      motorPWM(PIN_MOTOR_M1_IN2, 0);
      Serial.println("Motor M1 -1");
      }
    else {
      motorPWM(PIN_MOTOR_M1_IN1, 0);
      motorPWM(PIN_MOTOR_M1_IN2, 0);
      Serial.println("Motor M1 STOP");
      }
  }
  else if (Motor_ID == 2){
    if (dir == 1){
      motorPWM(PIN_MOTOR_M2_IN1, Motor_speed);
      motorPWM(PIN_MOTOR_M2_IN2, 0);
      Serial.println("Motor M2 1");
    }
    else if (dir == -1){
      motorPWM(PIN_MOTOR_M2_IN1, 0);
      motorPWM(PIN_MOTOR_M2_IN2, Motor_speed);
      Serial.println("Motor M2 -1");
      }
    else {
      motorPWM(PIN_MOTOR_M2_IN1, 0);
      motorPWM(PIN_MOTOR_M2_IN2, 0);
      Serial.println("Motor M2 STOP");
      }
  }
}

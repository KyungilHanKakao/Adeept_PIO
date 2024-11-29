#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PIN_MOTOR_M1_IN1 6      // Define the positive pole of M1
#define PIN_MOTOR_M1_IN2 7      // Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 8      // Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 9      // Define the negative pole of M2


// #define PIN_MOTOR_M1_PWM A1      //Define the PWM of M1
// #define PIN_MOTOR_M2_PWM A2      //Define the PWM of M2

// Called this way, PCA9685 uses the address 0x5F.
extern Adafruit_PWMServoDriver pwm_motor;

void motorPWM(int channel, int motor_speed);
void Motor(int Motor_ID, int dir, int Motor_speed);

#endif

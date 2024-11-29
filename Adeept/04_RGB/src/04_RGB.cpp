/***********************************************************
File name: RGB LED
Website: www.adeept.com
Date: 2023/10/09
***********************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define FREQ 1500 //  Set the freuqency to 1500Hz.
float brightness_level = 3; // 1-10;

void RGB_brightness(int level);
void color(int index,float R,float G,float B);
Adafruit_PWMServoDriver pwm_RGB = Adafruit_PWMServoDriver(0x5F);

void setup() {
  // pwm_RGB.begin();
  pwm_RGB.setPWMFreq(FREQ);  // Set the PWM frequency.
  pwm_RGB.begin();
  RGB_brightness(brightness_level); // Set RGB LED brightness.
  delay(100);
}

void loop() {
  // (RGB_ID,  R,G,B)
  // RGB_ID: 1 or 2.
  // R,G,B: RGB color value， each value ranges from 0-255.
color(1, 0,0,255);  // RGB1 light Blue.
color(2, 0,0,255);  // RGB2 light Blue.
delay(1000);        // delay 1s.
color(1, 0,255,0);  // Green.
color(2, 0,255,0);
delay(1000);
color(1, 255,0,0);  // Red.
color(2, 255,0,0);
delay(1000);
color(1, 0,0,0);    // Light off.
color(2, 0,0,0);
delay(1000);
}

// Set RGB LED brightness value.
void RGB_brightness(int level){
  brightness_level = level/10.0;
}

// Set RGB LED color value.
void color(int index,float R,float G,float B){
  R = int(map(R, 0,255, 0, 4095*brightness_level));
  G = int(map(G, 0,255, 0, 4095*brightness_level));
  B = int(map(B, 0,255, 0, 4095*brightness_level));

  if (index == 1){
    pwm_RGB.setPWM(0, 0, 4095-R); // (CH0, 0, PWM value)
    pwm_RGB.setPWM(1, 0, 4095-G); // (CH1, 0, PWM value)
    pwm_RGB.setPWM(2, 0, 4095-B); // (CH2, 0, PWM value)
  }
  else if (index == 2){
    pwm_RGB.setPWM(3, 0, 4095-R);
    pwm_RGB.setPWM(4, 0, 4095-G);
    pwm_RGB.setPWM(5, 0, 4095-B);
  }
}

#include <Arduino.h>
#include "I2Cdev.h"            // I2C communication to obtain MPU6050 data
#include "MPU6050_6Axis_MotionApps20.h"  // MPU6050 gyroscope control library
#include "Wire.h"
#include <TimerOne.h>


MPU6050 mpu ;          // Instantiate an MPU6050 object named mpu
int16_t ax, ay, az, gx, gy, gz;  // MPU6050 gyroscope 6 axis data parameters

//****************** Angle data set *******************
float K1 = 0.05;        // Weight of the accelerometer value
float angle0 = 0.00;    // Mechanical balance Angle

void setup() {
 // Join the I2C bus
 Wire.begin();                 // Add the I2C bus sequence
 Serial.begin(115200);           // Enable the serial port and set the baud rate to 115200
 delay(1500);    // delay1.5S
 mpu.initialize();             // Initialize the MPU6050 gyroscope
 delay(2); 
}

void loop() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     // The IIC obtains the MPU6050 six-axis data ax ay az gx gy gz
    // Output gyroscope data
    Serial.print("X-axis Angle: "); Serial.println(ax);
    Serial.print("Y-axis Angle: "); Serial.println(ay);
    Serial.print("Z-axis Angle: "); Serial.println(az);
    Serial.print("X-axis angular velocity: "); Serial.println(gx);
    Serial.print("Y-axis angular velocity: "); Serial.println(gy);
    Serial.print("Z-axis angular velocity: "); Serial.println(gz);
    Serial.println();
    delay(1500);
}

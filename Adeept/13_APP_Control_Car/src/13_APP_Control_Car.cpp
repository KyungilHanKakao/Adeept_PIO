#include <Arduino.h>
#include <SPI.h>
#include <TimerOne.h>

#include "MsTimer2.h"
#include "Adeept_Balance2WD.h"
#include "Adeept_KalmanFilter.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include "MotorControl.h"
#include "PinChangeInterrupt.h"
#include "Servo.h"

#include <Adafruit_NeoPixel.h>
#define LED_PIN    A3         // WS2812 connect to pin A3 .
#define NUM_LEDS   10         // LED number.
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

#define FREQ 1500 //  Set the freuqency to 1500Hz.
float brightness_level = 3; // 1-10;
Adafruit_PWMServoDriver pwm_RGB = Adafruit_PWMServoDriver(0x5F);

void judgement();
void control(int value);
void control(int value);
void Code_left();
void Code_right();
void RGB_brightness(int );
void color(int index,float R,float G,float B);
float getDistance();
void Keep_Distance();
void Avoid_Obstacles();
int StopFunction();





MPU6050 mpu; //Instantiate an MPU6050 object with the object name mpu
Adeept_Balance2WD balancecar;//Instantiate an balance object with the object name balancecar
Adeept_KalmanFilter kalmanfilter;//Instantiate an KalmanFilter object with the object name kalmanfilter
//Adeept_Distance Dist;//Instantiate an distance object with the object name Dist

int16_t ax, ay, az, gx, gy, gz;

 // PCINT number.
#define PIN_MOTOR_M1_isr  (A1)     //Define the interrupt of A1.
#define PIN_MOTOR_M2_isr  (A2)     //Define the interrupt of A2.

Servo s;
String phone1 = "forwardStart\n";
String phone2 = "backwardStart\n";
String phone3 = "leftStart\n";
String phone4 = "rightStart\n";
String phone7 = "forwardStop\n";
String phone8 = "backwardStop\n";
String phone9 = "leftStop\n";
String phone10 = "rightStop\n";

String phone5 = "aStart\n";
String phone6 = "bStart\n";
String phone11 = "cStart\n";
String phone12 = "dStart\n";
String phone13 = "aStop\n";
String phone14 = "bStop\n";
String phone15 = "cStop\n";
String phone16 = "dStop\n";

String phone17 = "lookLeftStart\n";
String phone18 = "lookLeftStop\n";
String phone19 = "lookRightStart\n";
String phone20 = "lookRightStop\n";
String phone21 = "downStart\n";
String phone22 = "downStop\n";
String phone23 = "upStart\n";
String phone24 = "upStop\n";

String comdata = "";
char judge;

int trigPin = 7;                  // define Trig pin for ultrasonic ranging module
int echoPin = 8;                  // define Echo pin for ultrasonic ranging module
float maxDistance = 200;          // define the range(cm) for ultrasonic ranging module, Maximum sensor distance is rated at 400-500cm.
float soundVelocity = 340;        // Sound velocity = 340 m/s
float rangingTimeOut = 2 * maxDistance / 100 / soundVelocity * 1000000; // define the timeout(ms) for ultrasonic ranging module

int Function_Stop_flag = 0;

/****************************Declare a custom variable*****************/
int time;
byte inByte; //The serial port receives the byte
int num;
double Setpoint;                                         //Angle DIP setpoint, input, and output
double Setpoints, Outputs = 0;                           //Speed DIP setpoint, input, and output
double angleoutput;

double kp = 25.92, ki = 0, kd = 0.7776;  // 0.6*0.6*0.6
double kp_speed = 28, ki_speed = 0.14, kd_speed = 0.00; 
double kp_turn = -55, ki_turn = 0, kd_turn = -0.60;  

//Steering PID parameters
double setp0 =1, dpwm = 0, dl = 0; //Angle balance point, PWM difference, dead zone, PWM1, PWM2
float value;
 
/********************angle data*********************/
float Q;
float Angle_ax; //The angle of inclination calculated from the acceleration
float Angle_ay;
float K1 = 0.05; // The weight of the accelerometer
float angle0 =  -6.5; //Mechanical balance angle 5.5  6.05
//float angle0 =  -5.45; //Mechanical balance angle 6.05
int slong;

/***************Kalman_Filter*********************/
float Q_angle = 0.001, Q_gyro = 0.005; //Angle data confidence, angular velocity data confidence
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; //Filter method sampling time interval milliseconds
float dt = timeChange * 0.001; //Note: The value of dt is the filter sampling time

/******************* speed count ************/
volatile long count_right = 0;//Use the volatile long type to ensure that the value is valid for external interrupt pulse count values used in other functions
volatile long count_left = 0;//Use the volatile long type to ensure that the value is valid for external interrupt pulse count values used in other functions
int speedcc = 0;

/*******************************Pulse calculation*****************************/
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;

/********************Turn the parameters of rotation**********************/
int turncount = 0;
float turnoutput = 0;

/****************Control volume*******************/
int front = 0;//Forward variable
int back = 0;//Backward variables
int turnl = 0;//Turn left mark
int turnr = 0;//Turn right
int spinl = 0;//Left rotation mark
int spinr = 0;//Right turn mark

/***************Ultrasonic velocity******************/
float distance;
int detTime=0; 

/*Pulse calculation */
void countpluse(){
  lz = count_left;
  rz = count_right;
  
  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0)){//Car movement direction to determine: back when (PWM is the motor voltage is negative) pulse number is negative
    rpluse = -rpluse;
    lpluse = -lpluse;
  }else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0)){//Car movement direction to determine: forward (PWM is the motor voltage is positive) pulse number is negative
    rpluse = rpluse;
    lpluse = lpluse;
  }else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0)){////Car movement direction to determine: right rotation, the right pulse number is positive, the number of left pulse is negative.
    rpluse = rpluse;
    lpluse = -lpluse;
  }else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0)){//Car movement direction to determine: left rotation, the right pulse number is negative, the number of left pulse is positive.
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  //To judge
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;
  //Every 5ms into the interruption, the number of pulses superimposed
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
}
/*Angle PD*/
void angleout(){
  balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD angle loop control
}

/*Interrupt timing 5ms timer interrupt*/
void inter(){
  sei();                                        
  countpluse();                                     //Pulse superposition of sub - functions
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC gets MPU6050 six axis data ax ay az gx gy gz
  kalmanfilter.angleTest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro,R_angle,C_0,K1);  //Get angle and Kaman filter
  angleout();                                       //Angle loop PD control
  speedcc++;
  // Speed loop control
  if (speedcc >= 8){                                //50ms into the speed loop control
      Outputs = balancecar.speedPiOut(kp_speed,ki_speed,kd_speed,front,back,setp0);
      speedcc = 0;
  }
    //Steering control
  turncount++;
  if (turncount > 2){                                //10ms into the rotation control
      turnoutput = balancecar.turnSpin(turnl,turnr,spinl,spinr,kp_turn,kd_turn,kalmanfilter.Gyro_z);  //Rotary subfunction
      turncount = 0;
  }
  balancecar.posture++;
  balancecar.pwma(Outputs,turnoutput,kalmanfilter.angle,kalmanfilter.angle6,turnl,turnr,spinl,spinr,front,back,kalmanfilter.accelz);//car total PWM output   
  
}

void setup() {
  pwm_motor.begin();
  pwm_motor.setPWMFreq(1600);  // Set PWM frequency to 50Hz.

  pinMode(PIN_MOTOR_M1_isr, INPUT_PULLUP);
  pinMode(PIN_MOTOR_M2_isr, INPUT_PULLUP);

//  Dist.begin(8,7);//begin(int echoPin, int trigPin)
  //Initialize the I2C bus
  Wire.begin();  
  //Turn on the serial port and set the baud rate to 9600
  //Communicate with the Bluetooth module
  Serial.begin(9600); 
  delay(150);
  //Initialize the MPU6050
  mpu.initialize();    
  delay(2);
  //5ms timer interrupt setting. Use timer2. Note: Using timer2 will affect the PWM output of pin3 and pin11.
  //Because the PWM is used to control the duty cycle timer, so when using the timer should pay attention to 
  //see the corresponding timer pin port.
  MsTimer2::set(5, inter);
  MsTimer2::start();

  pwm_RGB.setPWMFreq(FREQ);  // Set the PWM frequency.
  pwm_RGB.begin();
  RGB_brightness(brightness_level); // Set RGB LED brightness.
  delay(10);

  s.attach(2);
  s.write(90);
  Serial.begin(115200);       // set up a wifi serial communication baud rate 115200
   
  Serial.println("AT+CWMODE=3\r\n");//set to softAP+station mode
  delay(2000);     //delay 4s  
  Serial.println("AT+CWSAP=\"Adeept_ESP8266\",\"12345678\",8,2\r\n");   //TCP Protocol, server IP addr, port
  delay(1000);     //delay 4s
  Serial.println("AT+RST\r\n");     //reset wifi
  delay(1000);     //delay 4s
  Serial.println("AT+CIPMUX=1\r\n");//set to multi-connection mode
  delay(1000);
  Serial.println("AT+CIPSERVER=1\r\n");//set as server
  delay(1000);
  Serial.println("AT+CIPSTO=7000\r\n");//keep the wifi connecting 7000 seconds
  delay(1000);

  pixels.begin();         // Initialize the NeoPixel library.
  pixels.setBrightness(5); // Set WS2812 LED brightness.

  pinMode(trigPin, OUTPUT); // set trigPin to output mode
  pinMode(echoPin, INPUT);  // set echoPin to input mode
}

void loop() {
//  Serial.println(kalmanfilter.angle);
  while(Serial.available()>0)
   {  
    comdata += char(Serial.read());
    delay(1);
   }
   
   judgement();
   while (judge != 0) {
    control(judge);
    judgement();
  }

  
  attachPCINT(digitalPinToPCINT(PIN_MOTOR_M1_isr), Code_left, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_MOTOR_M2_isr), Code_right, CHANGE);
  
}

void judgement() {
  judge = 0;
  if (comdata.length() > 0){
    if (comdata.endsWith(phone1)) {
    judge=1;
  } else if (comdata.endsWith(phone2)) {
    judge=2;
  } else if (comdata.endsWith(phone3)) {
    judge=3;
  } else if (comdata.endsWith(phone4)) {
    judge=4;
  } 

  else if (comdata.endsWith(phone7)) {
    judge=7;
  } else if (comdata.endsWith(phone8)) {
    judge=8;
  } else if (comdata.endsWith(phone9)) {
    judge=9;
  } else if (comdata.endsWith(phone10)) {
    judge=10;
  } 
  
  else if (comdata.endsWith(phone5)) {
    judge=5;
  } else if (comdata.endsWith(phone6)) {
    judge=6;
  } else if (comdata.endsWith(phone11)) {
    judge=11;
  } else if (comdata.endsWith(phone12)) {
    judge=12;
  } 
  
  else if (comdata.endsWith(phone17)) {
    judge=17;
  } else if (comdata.endsWith(phone19)) {
    judge=19;
  } else if (comdata.endsWith(phone21)) {
    judge=21;
  } else if (comdata.endsWith(phone23)) {
    judge=23;
  } 
    comdata = "";

    delay(10);
  }
}

void control(int value){ 
//  Serial.println(value);
  switch(value) {
    case 1:
      front = -25;
      back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;
      //Serial.println(front);
      break;
    case 2:
      back = 25;
      front = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;
      break;
    case 3:
      turnl = 1;
      turnr = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0;
      break;
    case 4:
      turnr = 1;
      turnl = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0;
      break;
      
    case 5:
      Function_Stop_flag = 0;
      Keep_Distance();
      break;
      
    case 6:
      front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0;
      break;
      
    case 7:  //forward stop
      front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;
      break;

    case 8:  //backward stop
      front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;

      break;

    case 9:
      front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0;
      break;

    case 10:
      front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0;
      break;

      
    case 11:
      Function_Stop_flag = 0;
      Avoid_Obstacles();
      break;
      
    case 12:
      front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0;
      break;
    
    case 17:
      color(1, 255, 0, 0);
      color(2, 255, 0, 0);
      break;
    case 19:
      color(1, 0, 0, 0);
      color(2, 0, 0, 0);
      break;

    case 23:
      for(int i=0;i<=NUM_LEDS;i++){
        pixels.setPixelColor(i,pixels.Color(255,0,0)); // red
      }
      pixels.show();  // Send the command.
      break;
    case 21:
      pixels.clear();
      for(int i=0;i<=NUM_LEDS;i++){
        pixels.setPixelColor(i,pixels.Color(0,0,0)); // red
      }
      pixels.show();  // Send the command.
      break;

      
    default:
      break;
  }

}


/*Left speed chart*/
void Code_left() {
  count_left ++;
} 
/*Right speed chart count*/
void Code_right() {
  count_right ++;
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
float getDistance() { 
  unsigned long pingTime; // save the high level time returned by ultrasonic ranging module
  float distance;         // save the distance away from obstacle
  // set the trigPin output 10us high level to make the ultrasonic ranging module start to measure
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // get the high level time returned by ultrasonic ranging module
  pingTime = pulseIn(echoPin, HIGH, rangingTimeOut);
  if (pingTime != 0) {  // if the measure is not overtime
    distance = pingTime * soundVelocity / 2 / 10000;  // calculate the obstacle distance(cm) according to the time of high level returned
    return distance;    // return distance(cm)
  }
  else                  // if the measure is overtime
    return maxDistance; // returns the maximum distance(cm)
}
void Keep_Distance(){
  while (1){
    StopFunction();
    if (Function_Stop_flag == 1){ // Press OK, stop function.
      break;
    }
    noInterrupts(); 
    distance = getDistance();
    interrupts(); 
    delay(50);
    if (distance < 30){
        back = 20;
        front = 0;
      }
    else if (distance > 40){
        front = -20;
        back = 0;
      }
    else {
        front = 0; back = 0; turnl = 0; turnr = 0;
      }
    delay(100);
  }
}

void Avoid_Obstacles(){
  while (1){
    StopFunction();
    if (Function_Stop_flag == 2){ // Press OK, stop function.
      break;
    }
    noInterrupts();
    distance = getDistance();
    interrupts(); 
    delay(50);
    if (distance < 20){
        back = 15;
        front = 0; turnl = 0; turnr = 0;
      }
    else if (distance<40&&distance>20){
        turnl = 1; // control smart 2WD balance turn left 
        front = 0; back = 0; turnr = 0;
      }
    else {
        front = -15;// control 2WD balance car backwards
        back = 0;  turnl = 0; turnr = 0;
      }
    delay(100);
  }
}

int StopFunction(){
  while(Serial.available()>0){  
    comdata += char(Serial.read());
    delay(1);
   }
  if (comdata.length() > 0){ 
    if(comdata.endsWith(phone6)){// Stop Avoid Obstacles function.
      Function_Stop_flag = 1;
      judge=6;
    }
    else if(comdata.endsWith(phone12)){ // Stop Light Tracking function.
      Function_Stop_flag = 2;
      judge=12;
    }
    delay(10);
  }
  return 0;
}

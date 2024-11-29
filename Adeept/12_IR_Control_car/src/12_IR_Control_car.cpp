#include <Arduino.h>
#include <TimerOne.h>
#include <SPI.h>

#include "MsTimer2.h"
#include "Adeept_Balance2WD.h"
#include "Adeept_KalmanFilter.h"
#include "Adeept_Distance.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Adafruit_PWMServoDriver.h>
#include "MotorControl.h"
#include "PinChangeInterrupt.h"
#include <IRremote.h> //Call the library corresponding to the infrared remote control.
#define RECV_PIN     2        // Infrared receiving pin

int StopFunction();
void color(int index,float R,float G,float B);
void Keep_Distance();
void Avoid_Obstacles();
void Code_right();
void Code_left();
int switch_irr(int irr_data);
void control(unsigned long value);
void RGB_brightness(int level);
void inter();

IRrecv irrecv(RECV_PIN);      // Create a class object used to receive class
decode_results results;       // Create a decoding results class object
unsigned long strP,strNow;

#define FREQ 1500 //  Set the freuqency to 1500Hz.
float brightness_level = 3; // 1-10;
Adafruit_PWMServoDriver pwm_RGB = Adafruit_PWMServoDriver(0x5F);

MPU6050 mpu; //Instantiate an MPU6050 object with the object name mpu
Adeept_Balance2WD balancecar;//Instantiate an balance object with the object name balancecar
Adeept_KalmanFilter kalmanfilter;//Instantiate an KalmanFilter object with the object name kalmanfilter
Adeept_Distance Dist;//Instantiate an distance object with the object name Dist

int16_t ax, ay, az, gx, gy, gz;

 // PCINT number.
#define PIN_MOTOR_M1_isr  (A1)     //Define the interrupt of A1.
#define PIN_MOTOR_M2_isr  (A2)     //Define the interrupt of A2.

#include <Adafruit_NeoPixel.h>
#define LED_PIN    A3         // WS2812 connect to pin A3 .
#define NUM_LEDS   10         // LED number.
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

int trigPin = 7;                  // define Trig pin for ultrasonic ranging module
int echoPin = 8;                  // define Echo pin for ultrasonic ranging module
float maxDistance = 200;          // define the range(cm) for ultrasonic ranging module, Maximum sensor distance is rated at 400-500cm.
float soundVelocity = 340;        // Sound velocity = 340 m/s
float rangingTimeOut = 2 * maxDistance / 100 / soundVelocity * 1000000; // define the timeout(ms) for ultrasonic ranging module
int Function_flag = 0;

/****************************Declare a custom variable*****************/
int time;
byte inByte; //The serial port receives the byte
int num;
double Setpoint;                                         //Angle DIP setpoint, input, and output
double Setpoints, Outputs = 0;                           //Speed DIP setpoint, input, and output
double angleoutput;

double kp = 25.92, ki = 0, kd = 0.7776;  // 0.6x0.6*0.6..
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
float angle0 =  -6.05; //Mechanical balance angle 5.5
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

/****************control volume*******************/
int front = 0;//Forward variable
int back = 0;//Backward variables
int turnl = 0;//Turn left mark
int turnr = 0;//Turn right
int spinl = 0;//Left rotation mark
int spinr = 0;//Right turn mark

/***************Ultrasonic velocity******************/
int distance;
int detTime=0; 

void setup() {
  pwm_motor.begin();
  pwm_motor.setPWMFreq(1600);  // Set PWM frequency to 50Hz.
  pinMode(PIN_MOTOR_M1_isr, INPUT_PULLUP);
  pinMode(PIN_MOTOR_M2_isr, INPUT_PULLUP);
  Dist.begin(8,7);//begin(int echoPin, int trigPin)
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
  
//  Serial.begin(115200);       // Initialize the serial port and set the baud rate to 115200
  irrecv.enableIRIn();        // Start the receiver
//  Serial.print("IRrecvDemo is now running and waiting for IR message on Pin ");
//  Serial.println(RECV_PIN);   //print the infrared receiving pin

  pwm_RGB.setPWMFreq(FREQ);  // Set the PWM frequency.
  pwm_RGB.begin();
  RGB_brightness(brightness_level); // Set RGB LED brightness.
  delay(100);

  pixels.begin();         // Initialize the NeoPixel library.
  pixels.setBrightness(5); // Set WS2812 LED brightness.

}

void loop() {
  if (irrecv.decode(&results)) {          // Waiting for decoding
    unsigned long value = results.value;
   value = switch_irr(value);
    Serial.println(value);           // Print out the decoded results
    control(value);
    irrecv.resume();                      // Release the IRremote. Receive the next value
      
  }else {                                // When no value is received
    turnr = 0;
    turnl = 0;
    
    front = 0;
    back = 0;
    spinl = 0;
    spinr = 0;
    turnoutput = 0;
}
  delay(100);
  
  attachPCINT(digitalPinToPCINT(PIN_MOTOR_M1_isr), Code_left, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_MOTOR_M2_isr), Code_right, CHANGE);
  
}



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



void control(unsigned long value){
  Serial.println(value);
  switch (value) {
    
    // Move Control
    case 1:
      color(1, 0,0,255);
      color(2, 0,0,255); 
      break;
    case 2:
      color(1, 255,0,0);
      color(2, 255,0,0); 
      break;
    case 3:
      color(1, 0,0,0);
      color(2, 0,0,0); 
      break;

    case 4:
      for(int i=0;i<=NUM_LEDS;i++){
        pixels.setPixelColor(i,pixels.Color(0,0,255)); // blue
      }
      pixels.show();  // Send the command.
      break;
    case 5:
      for(int i=0;i<=NUM_LEDS;i++){
        pixels.setPixelColor(i,pixels.Color(255,0,0)); // red
      }
      pixels.show();  // Send the command.
      break;
    case 6:
      for(int i=0;i<=NUM_LEDS;i++){
        pixels.setPixelColor(i,pixels.Color(0,0,0)); // red
      }
      pixels.show();  // Send the command.
      break;

    case 10:
      Function_flag = 0;
      Keep_Distance(); // Follow car function
      break;
    case 11:
      Function_flag = 0;
      Avoid_Obstacles(); // Avoid Obstacles function
      break;
    case 16:  // OK. Stop function.
      turnr = 0; turnl = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0;
      break;
    
    
    case 12:  //up
      front = -50;
      back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;
      break;
    case 13:  //down
      back = 50;
      front = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;
      break;
    case 14:  //left
      turnl = 1;
      turnr = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0;
      break;
    case 15:  //right
      turnr = 1;
      turnl = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0;
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

int switch_irr(int irr_data)
{
  switch(irr_data)
  {
  case 16750695: return 0;
  case 16753245: return 1;
  case 16736925: return 2;
  case 16769565: return 3;
  case 16720605: return 4;
  case 16712445: return 5;
  case 16761405: return 6;
  case 16769055: return 7;
  case 16754775: return 8;
  case 16748655: return 9;
  case 16738455: return 10; // *
  case 16756815: return 11; // #
  case 16718055: return 12; // up
  case 16730805: return 13; // down
  case 16716015: return 14; // left
  case 16734885: return 15; // right
  case 16726215: return 16; // ok
  }
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
    if (Function_flag == 1){ // Press OK, stop function.
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
    if (Function_flag == 1){ // Press OK, stop function.
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
    if (irrecv.decode(&results)) {          // Waiting for decoding
    unsigned long value = results.value;
    value = switch_irr(value);
    // Serial.println(value);
    if (value == 16){ // OK, Stop function.
      Function_flag = 1;}
    else{
      Function_flag = 0;
    }
    irrecv.resume();                      // Release the IRremote. Receive the next value
  }
  // delay(100);
}


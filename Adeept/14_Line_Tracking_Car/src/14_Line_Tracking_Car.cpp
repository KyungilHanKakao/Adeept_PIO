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

MPU6050 mpu; //Instantiate an MPU6050 object with the object name mpu
Adeept_Balance2WD balancecar;//Instantiate an balance object with the object name balancecar
Adeept_KalmanFilter kalmanfilter;//Instantiate an KalmanFilter object with the object name kalmanfilter
Adeept_Distance Dist;//Instantiate an distance object with the object name Dist
void countpluse();
void angleout();
void Code_right();
int Track_Read(void);
void Line_Tracking();
void Code_left();
void inter();

int16_t ax, ay, az, gx, gy, gz;
// extern int pwm2;
#define PIN_MOTOR_M1_IN1 6      //Define the positive pole of M1
#define PIN_MOTOR_M1_IN2 7      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 8      //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 9      //Define the negative pole of M2

 // PCINT number.
#define PIN_MOTOR_M1_isr  (A1)     //Define the interrupt of A1.
#define PIN_MOTOR_M2_isr  (A2)     //Define the interrupt of A2.

#define Tracking_Left   4 //define Tracking left port
#define Tracking_Mid    5 //define Tracking mid port
#define Tracking_Right  6 //define Tracking right port
int Val[3] = {0,0,0};
int value;
/****************************Declare a custom variable*****************/
int time;
byte inByte; //The serial port receives the byte
int num;
double Setpoint;                                         //Angle DIP setpoint, input, and output
double Setpoints, Outputs = 0;                           //Speed DIP setpoint, input, and output
double angleoutput;

double kp = 25.92, ki = 0, kd = 0.7776;

double kp_speed = 22.3, ki_speed = 0.1115, kd_speed = 0.00; 

double kp_turn = -55, ki_turn = 0, kd_turn = -0.60; 



//Steering PID parameters
double setp0 =1, dpwm = 0, dl = 0; //Angle balance point, PWM difference, dead zone, PWM1, PWM2

 
/********************angle data*********************/
float Q;
float Angle_ax; //The angle of inclination calculated from the acceleration
float Angle_ay;
float K1 = 0.05; // The weight of the accelerometer
float angle0 =  -5.5; //Mechanical balance angle 5.5
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

/****************Bluetooth control volume*******************/
int front = 0;//Forward variable
int back = 0;//Backward variables
int turnl = 0;//Turn left mark
int turnr = 0;//Turn right
int spinl = 0;//Left rotation mark
int spinr = 0;//Right turn mark

/***************Ultrasonic velocity******************/
int distance;
int detTime=0; 

/*Pulse calculation 脉冲计算*/
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
  if (speedcc >= 8){                                //50ms into the speed loop control
      Outputs = balancecar.speedPiOut(kp_speed,ki_speed,kd_speed,front,back,setp0);
      speedcc = 0;
  }
    //turn转向环调节
  turncount++;
  Serial.println(turncount);
  if (turncount > 2){                                //10ms into the rotation control
      Serial.println("turncount");
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

  pinMode(Tracking_Left, INPUT); //define input port
  pinMode(Tracking_Mid, INPUT); 
  pinMode(Tracking_Right, INPUT);
  
}


void loop() {
  attachPCINT(digitalPinToPCINT(PIN_MOTOR_M1_isr), Code_left, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_MOTOR_M2_isr), Code_right, CHANGE);

  Val[0] = digitalRead(Tracking_Left); // Get the value of D4 and save it in an array.
  Val[1] = digitalRead(Tracking_Mid);  // Get the value of D5 and save it in an array.
  Val[2] = digitalRead(Tracking_Right); // Get the value of D6 and save it in an array.

  Line_Tracking();
  
}

/*Left speed chart*/
void Code_left() {
  count_left ++;
} 
/*Right speed chart count*/
void Code_right() {
  count_right ++;
} 

void Line_Tracking(){
    value = Track_Read();  //Read the value of the tracking module.
    switch (value)  
    {
      case 0:   //000 stop
        turnl = 0; turnr = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0;
        break;

      case 1:   //010 forward
        front = -5;
        break;
      
      case 2:   //100 Left
        turnl = 1;
        break;
        
      case 3:   //110 Left
        turnl = 1;
        break;
        
      case 4:   //001 right
        turnr = 1;
        break;
        
      case 5:   //011 right
        turnr = 1;
        break;

      case 6:   //111 stop
        turnl = 0; turnr = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0;
        break;
      case 7:   //101 forward
        front = -5;
        break;
      default:
        break;
    }
    // delay(100);
}

int Track_Read(void){
  int value = 0;
  Val[0] = digitalRead(Tracking_Left);
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
  else if ((Val[0]==1)&&(Val[1]==0)&&(Val[2]==1)){ // 101
    value = 7;
    }
  return value;
}

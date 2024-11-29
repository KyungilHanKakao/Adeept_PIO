#include "Adeept_Balance2WD.h"
#include <Adafruit_PWMServoDriver.h>
#include "MotorControl.h"


double Adeept_Balance2WD::speedPiOut(double kps,double kis,double kds,int f,int b,double p0)
{
  float speeds = (pulseleft + pulseright) * 0.1;         //Speed pulse value,0.1:Roughly convert the speed value of 5ms
  pulseright = pulseleft = 0;
  speeds_filterold *= 0.7;                               //First order complementary filtering
  float speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions += f;    //Full control volume integration
  positions += b;    //Full control volume integration
  positions = constrain(positions, -2000,2000);          //Anti-integral saturation
  double output = kis * (p0 - positions) + kps * (p0 - speeds_filter);//Speed loop control
  if(flag1==1)
  {
  positions=0;
  // output=0;
  }
  
  return output;
}

float Adeept_Balance2WD::turnSpin(int turnleftflag,int turnrightflag,int spinleftflag,int spinrightflag,double kpturn,double kdturn,float Gyroz)
{
  int spinonce = 0;
  float turnspeed = 0;
	float rotationratio = 0;
  float turnout_put = 0;
	
  if (turnleftflag == 1 || turnrightflag == 1 || spinleftflag == 1 || spinrightflag == 1)
  {
    if (spinonce == 0)       //Before the rotation to determine the current speed, enhance the car adaptability.
    {
      turnspeed = ( pulseright + pulseleft);       //The current speed of the car (Pulse representation)
	  spinonce++;
    }

    if (turnspeed < 0)       //The current speed of the car
    {
      turnspeed = -turnspeed;

    }
    if(turnleftflag==1||turnrightflag==1)
    {
     turnmax=3;
     turnmin=-3;
    }
    if(spinleftflag==1||spinrightflag==1)
    {
      turnmax=10;
      turnmin=-10;
    }
    rotationratio = 5 / turnspeed;       //According to the car speed set value
    if (rotationratio < 0.5)rotationratio = 0.5;
    if (rotationratio > 5)rotationratio = 5;
  }
  else
  {
    rotationratio = 0.5;
    spinonce = 0;
    turnspeed = 0;
  }
  if (turnleftflag == 1 || spinleftflag == 1)       //According to the direction parameters superimposed
  {
    turnout += rotationratio;
  }
  else if (turnrightflag == 1 || spinrightflag == 1)//According to the direction parameters superimposed
  {
    turnout -= rotationratio;
  }
  else turnout = 0;
  if (turnout > turnmax) turnout = turnmax;        //Amplitude maximum setting
  if (turnout < turnmin) turnout = turnmin;        //Amplitude minimum setting

  turnout_put = -turnout * kpturn - Gyroz * kdturn;//Rotation PD algorithm control, fusion speed and Z axis rotation positioning.
	return turnout_put;
}

void Adeept_Balance2WD::pwma(double speedoutput,float rotationoutput,float angle,float angle6,int turnleftflag,int turnrightflag,int spinleftflag,int spinrightflag,
	int f,int b,float accelz)
{

    // Serial.print("angle/speed/turn:");
    // Serial.print(angleoutput);
    // Serial.print("/");
    // Serial.print(speedoutput);
    // Serial.print("/");
    // Serial.println(rotationoutput);
  pwm1 = -angleoutput - speedoutput - rotationoutput; //Left motor PWM output value
  pwm2 = -angleoutput - speedoutput + rotationoutput; //Right motor PWM output value

  //Amplitude limit
  if (pwm1 > 255) pwm1 = 255;
  if (pwm1 < -255) pwm1 = -255;
  if (pwm2 > 255) pwm2 = 255;
  if (pwm2 < -255) pwm2 = -255;
  //The angle is too large to stop the motor.
  if (angle > 30 || angle < -30){
    pwm1 = 0;
    pwm2 = 0;
  }

   if (turnleftflag == 0 && turnrightflag == 0 && spinleftflag == 0 && spinrightflag == 0 && f == 0 && b == 0){
   
	  flag1=1;

  }else {
     stopl=stopr=0;
     flag1=0;
  }

int LStar_PWM = 0;
int RStar_PWM = 0;
   //Positive and negative output judgment of motor (left motor judgment)
  if (pwm1 >= 0) {

    Motor(1, -1, pwm1 + LStar_PWM);
    // Motor(1, -1, LStar_PWM);
    // analogWrite(PIN_MOTOR_M1_PWM, pwm1);
    // Serial.println(pwm1);
    // Serial.println("______________");
    // Serial.println(output);

  } else {
    Motor(1, 1, (-pwm1)+ LStar_PWM);
    // Motor(1, 1, LStar_PWM);
    // Serial.println(pwm1);
  }
   //Positive and negative output judgment of motor (right motor judgment)
  if (pwm2 >= 0) {
    Motor(2, -1, pwm2 + RStar_PWM);
    // Motor(2, -1, RStar_PWM);
    // Serial.println("Motor M2 Forward");
    // Serial.println(pwm2);
  } else {
    Motor(2, 1, -(pwm2) + RStar_PWM);
    // Motor(2, 1, RStar_PWM);
    // Serial.println("Motor M2 Backward");
    // Serial.println(pwm2);
  }
}


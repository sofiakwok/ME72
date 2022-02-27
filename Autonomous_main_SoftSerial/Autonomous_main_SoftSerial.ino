// Line IR Pin connections
// FR = Front Right
// FL = Front Left
// BR = Back Right
// BL = Back Left

// on nano, master/main board

const int FR_anl_pin = A0;
int FR_anl = 0;

const int FL_anl_pin = A1;
int FL_anl = 0;

const int BR_anl_pin = A2;
int BR_anl = 0;

const int BL_anl_pin = A3;
int BL_anl = 0;

// Threshold for IR line sensors
const int threshold = 500; //600;
//200 is better if there are bad scratches

#include <Servo.h>

// The PWM pins for the motors
const int MotorRPin = 5; 
const int MotorLPin = 6; 

Servo MotorR;
Servo MotorL;

const int max_speed = 100;
const int med_speed = 80; //55;
const int low_speed = 55;

const int motorLowR = 828;
const int motorHighR = 1825;

const int motorLowL = 880;
const int motorHighL = 1885;

//motorLow = 828/880 two nanos; 808 one nano one mega; 825 two mega;
//motorHigh = 1825/1885 two nanos; 1805 one nano one mega; 1825 two mega;

int angleAtMinDist = 0; 
int angles[] = {0,0,0,0,0};
int num_angles = 0;
#include "QuickMedianLib.h"  

// For kill switch
bool kill_switch = true;
volatile int pwm_value = 0;
volatile int prev_time = 0;

void setup() {
  Serial.begin(115200); // for the other board

  // For the kill switch
  // when pin D3 goes high, call the rising function
  attachInterrupt(1, rising, RISING);
  
  // pinModes
  // line detector IRs
  pinMode(FR_anl_pin,INPUT);
  pinMode(FL_anl_pin,INPUT);
  pinMode(BR_anl_pin,INPUT);
  pinMode(BL_anl_pin,INPUT);

  // Motor PPM signals & initially set motors to 0 speed
  MotorR.attach(MotorRPin);
  MotorL.attach(MotorLPin);
  MotorR.writeMicroseconds(map(0, -100, 100, motorLowR, motorHighR));
  MotorL.writeMicroseconds(map(0, -100, 100, motorLowL, motorHighL));

  while (kill_switch==true) {
    MotorR.writeMicroseconds(map(0, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(0, -100, 100, motorLowL, motorHighL));
    delay(100);
  }
  
  // Five seconds before match starts
  idle();

}

void loop() {
  while (kill_switch==false) {

        // First check IR sensors
        FR_anl = analogRead(FR_anl_pin);
        FL_anl = analogRead(FL_anl_pin);
        BR_anl = analogRead(BR_anl_pin);
        BL_anl = analogRead(BL_anl_pin);
    
        if (BR_anl <= threshold) {
          aborts(3);
        } else if (BL_anl <= threshold) {
          aborts(4);
        } else if (FR_anl <= threshold) {
          aborts(1);
        } else if (FL_anl <= threshold) {
          aborts(2);
        }
  
        else {
          if (Serial.available() > 0) { // if getting a signal from other board
            angleAtMinDist = Serial.read();
            angleAtMinDist = map(angleAtMinDist, 0, 255, 0, 360);
            angleAtMinDist = (angleAtMinDist+90)%360; // to account for rotation of LiDAR      
            
            angles[num_angles%5] = {angleAtMinDist};
            angleAtMinDist = QuickMedian<int>::GetMedian(angles, 5);
            num_angles++;
            
            attack(angleAtMinDist);
          }
        }
        // If object is not within the required distance then won't receive a signal from other board 
   
        delay(500); // TRY DELETING       
      } 

  // gets here if kill_switch = true
    MotorR.writeMicroseconds(map(0, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(0, -100, 100, motorLowL, motorHighL));
    delay(1000);    
}

void idle() {
  delay(5000);
}


void attack(int angle) {
  // angle is in degrees
  if (angle < 25 || angle > 335) {
    // If opponent is right in front then charge full speed forward
    MotorR.writeMicroseconds(map(low_speed, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(low_speed, -100, 100, motorLowL, motorHighL));
    //delay(80);
  }
  // Otherwise turn until opponent is straight ahead and then on
  // the next loop the robot will charge forward
  else if (angle < 90) { // turn right
    MotorR.writeMicroseconds(map(-low_speed, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(low_speed, -100, 100, motorLowL, motorHighL));
  } else if (angle < 180) { // turn right stronger
    MotorR.writeMicroseconds(map(-low_speed, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(low_speed, -100, 100, motorLowL, motorHighL));
  } else if (angle < 270) { // turn left stronger
    MotorR.writeMicroseconds(map(low_speed, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(-low_speed, -100, 100, motorLowL, motorHighL));
  } else { // turn left
    MotorR.writeMicroseconds(map(low_speed, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(-low_speed, -100, 100, motorLowL, motorHighL));
  } 

  int i=0;
  while (FTLINE()==false && BKLINE()==false && i < 10) { //if all IR are off and counter hasn't maxed out
    delay(15); // drive
    i++;
  }
    
  MotorR.writeMicroseconds(map(0, -100, 100, motorLowR, motorHighR));
  MotorL.writeMicroseconds(map(0, -100, 100, motorLowL, motorHighL));
  //delay(500); CAN'T HAVE A DELAY HERE
}

void aborts(int corner) {
  // 1 = Front Right
  // 2 = Front Left
  // 3 = Back Right
  // 4 = Back Left
  
  if (corner==1 || corner==2) {
    // Retreat back
    MotorR.writeMicroseconds(map(-max_speed, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(-max_speed, -100, 100, motorLowL, motorHighL));
    //delay(125);

    int i=0;
    while (BKLINE()==false && kill_switch==false && i < 50) { //if both back IR are off keep retreating
      delay(5);
      i++;
    }

    while(FTLINE()==true && kill_switch==false) { // If front line still detected retreat back more
      delay(10); // Short delay      
    }

    // Once line is cleared, turn
    // Incorporate opponent search
    if (corner==1) {
      // Turn left
      MotorR.writeMicroseconds(map(med_speed, -100, 100, motorLowR, motorHighR));
      MotorL.writeMicroseconds(map(-med_speed, -100, 100, motorLowL, motorHighL));
      delay(180);
    }
    else {
      // Turn right
      MotorR.writeMicroseconds(map(-med_speed, -100, 100, motorLowR, motorHighR));
      MotorL.writeMicroseconds(map(med_speed, -100, 100, motorLowL, motorHighL));
      delay(180);     
    }
  }
  else { // corner =3,4 
    // Go forward
    MotorR.writeMicroseconds(map(max_speed, -100, 100, motorLowR, motorHighR));
    MotorL.writeMicroseconds(map(max_speed, -100, 100, motorLowL, motorHighL));
    //delay(150);

    int i=0;
    while (FTLINE()==false && kill_switch==false && i < 50) { //if both front IR are off
      delay(10); // keep driving forward
      i++;
    }

    while(BKLINE()==true && kill_switch==false) { // If back line still detected go forward more
      delay(10); // Short delay      
    }
  }
  MotorR.writeMicroseconds(map(0, -100, 100, motorLowR, motorHighR));
  MotorL.writeMicroseconds(map(0, -100, 100, motorLowL, motorHighL));
  // add delay here if needed at zero speed
}

bool FTLINE() { // Check if line is still detected in front
  FR_anl = analogRead(FR_anl_pin);
  FL_anl = analogRead(FL_anl_pin);
  if (FR_anl <= threshold || FL_anl <= threshold) {
    return true;
  }
  else {return false;}
}
bool BKLINE() { // Check if line is still detected in back
  BR_anl = analogRead(BR_anl_pin);
  BL_anl = analogRead(BL_anl_pin);
  if (BR_anl <= threshold || BL_anl <= threshold) {
    return true;
  }
  else {return false;}
}


// For kill switch interrupts
void rising() {
  attachInterrupt(1, falling, FALLING);
  prev_time = micros();
}
 
void falling() {
  attachInterrupt(1, rising, RISING);
  pwm_value = micros()-prev_time;
  if (pwm_value > 1900) { // Up is on and down is off
    kill_switch = true;
  }
  else {
    kill_switch = false;
  }
}

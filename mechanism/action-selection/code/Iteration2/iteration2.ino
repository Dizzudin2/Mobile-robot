#include <HCSR04.h>
#include <L298NX2.h>
#include <QMC5883LCompass.h>

// assign pin.
//motor 1(Right) pin
const unsigned int EN_1 = 11;
const unsigned int IN1_1 = 10;
const unsigned int IN2_1 = 9;
//motor 2(left) pin
const unsigned int EN_2 = 3;
const unsigned int IN1_2 = 4; 
const unsigned int IN2_2 = 5;
//ultrasonic sensor pin
int triggerPin = A0;
int echoPin = A1;

// Initialize sensor and motor.
L298NX2 myMotors(EN_1, IN1_1, IN2_1, EN_2, IN1_2, IN2_2);
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);
QMC5883LCompass compass;


String B[] = {"avoid obstacle" ,"find north","cruise"}; //Behaviour
double G[] = {1.5,1.0,0.5}; //Behaviour gain vector
float activation_level = 0.5; //Behaviour activation level
const int b = sizeof(B)/sizeof(B[0]);
int Dmax = 30;
int Dmin = 10;

void setup() {
  Serial.begin(9600);
  compass.init();
}

void loop() {
  
  Serial.println(millis());
  float R[b] = {0,0,0};

//Avoid obstacle behaviour
  float distanceFront = distanceSensor.measureDistanceCm();
  float avoidObstacleSensorStrength=0;
  
  if(distanceFront>=Dmax || distanceFront==-1){
    avoidObstacleSensorStrength = 0;
  }else if(distanceFront<Dmax && distanceFront>=Dmin){
    avoidObstacleSensorStrength = 1 - ((distanceFront-Dmin)/(Dmax-Dmin));
  }else{
    avoidObstacleSensorStrength = 1;
  }

  if(avoidObstacleSensorStrength>=activation_level){
    R[0]= avoidObstacleSensorStrength * G[0];
  }

  
//  Find North Behavior
  compass.read();
  int azimuth = compass.getAzimuth();
  float findNorthSensorStrength=0;

  if(azimuth>5 && azimuth<355){
    findNorthSensorStrength = 1;
  }else{
    findNorthSensorStrength = 0;
  }

  if(findNorthSensorStrength>=activation_level){
    R[1]= findNorthSensorStrength * G[1];
  }

//  Cruise Behaviour
    R[2]= 1 * G[2];
  
// Action Selection
  float max_r = 0;
  int max_i = 0;
  for ( int i = 0; i < b; i++ )
  {
    if ( R[i] > max_r )
    {
      max_r = R[i];
      max_i = i;
    }
  }

  if(B[max_i] == "avoid obstacle"){

    stopMove();
    delay(500);
    moveBackward();
    delay(500);
    rotateRight();
    delay(500);
    moveForward();
    delay(500);
    
  }else if(B[max_i] == "find north"){
    
    if(azimuth>=180 && azimuth <=355){
      rotateRight();
    }else if(azimuth>=5 && azimuth<180){
      rotateLeft();
    }
    
  }else if(B[max_i] == "cruise"){
    
    moveForward();
    
  }
}

void moveForward(){
  myMotors.setSpeedA(100);
  myMotors.setSpeedB(100);
  myMotors.forward();
}


void moveBackward(){
  myMotors.setSpeedA(100);
  myMotors.setSpeedB(100);
  myMotors.backward();
}

void rotateRight(){
  myMotors.setSpeedA(0);
  myMotors.setSpeedB(100);
  myMotors.forwardB();
  myMotors.backwardA();
}

void rotateLeft(){
  myMotors.setSpeedA(100);
  myMotors.setSpeedB(0);
  myMotors.forwardA();
  myMotors.backwardB();
}

void stopMove(){
  myMotors.setSpeedA(0);
  myMotors.setSpeedB(0);
  myMotors.forwardB();
  myMotors.backwardA();
}
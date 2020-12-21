#include <HCSR04.h>
#include <L298NX2.h>
#include <QMC5883LCompass.h>

// assign sensor and motor pin.
//motor 1(Right) pin
const unsigned int EN_1 = 11;
const unsigned int IN1_1 = 10;
const unsigned int IN2_1 = 9;
//motor 2(left) pin
const unsigned int EN_2 = 3;
const unsigned int IN1_2 = 4; 
const unsigned int IN2_2 = 5;
//ultrasonic sensor(front) pin
int triggerPin = A0;
int echoPin = A1;

// Initialize sensor and motor.
L298NX2 myMotors(EN_1, IN1_1, IN2_1, EN_2, IN1_2, IN2_2);
UltraSonicDistanceSensor distanceSensorFront(triggerPin, echoPin);
QMC5883LCompass compass;

String motorCommand[] = {"Forward", "Move Right", "Move Left", "Rotate Left","Rotate Right","stop"};
String B[] = {"avoid obstacle", "find north", "cruise"};
float G[]={1.5,1.0,0.5};
float activation_level = 0.5;
const int m = sizeof(motorCommand)/sizeof(motorCommand[0]);
const int b = sizeof(B)/sizeof(B[0]);

int Dmax = 30;
int Dmin = 10;

void setup() {
  Serial.begin(9600);
  compass.init();
}

void loop() {
  
  Serial.println(millis());
   
  float vote[b][m];

  //Obstacle avoidance behaviour
  float distanceFront = distanceSensorFront.measureDistanceCm();
  float avoidObstacleSensorStrength=0;
  if(distanceFront>=Dmax || distanceFront==-1){
    avoidObstacleSensorStrength = 0;
  }else if(distanceFront<Dmax && distanceFront>=Dmin){
    avoidObstacleSensorStrength = 1 - ((distanceFront-Dmin)/(Dmax-Dmin));
  }else{
    avoidObstacleSensorStrength = 1;
  }

  if(avoidObstacleSensorStrength>=activation_level){
    for(int i=0; i<m;i++){
      if(i==0){
        vote[0][i]=-1 * avoidObstacleSensorStrength;
      }else if(i==1 || i==2){
        vote[0][i]=1 * avoidObstacleSensorStrength;
      }else if(i==3 || i==4){
        vote[0][i]=0 * avoidObstacleSensorStrength;
      }else if(i==5){   
        vote[0][i]=0 * avoidObstacleSensorStrength; 
      }
    }
  }else{
    for(int i=0; i<m;i++){
      vote[0][i]=0;
    }
  }
  
//  Find north behaviour
  compass.read();
  int azimuth = compass.getAzimuth();
  float findNorthSensorStrength=0;
  if(azimuth>3 && azimuth<357){
    findNorthSensorStrength = 1;
  }else{
    findNorthSensorStrength = 0;
  }
  
  if(findNorthSensorStrength>=activation_level){
    if(azimuth>=180 && azimuth <=357){
      for(int i=0; i<m;i++){
        if(i==0){
          vote[1][i]=0 * findNorthSensorStrength;
        }else if(i==1 || i==2){
          vote[1][i]=0 * findNorthSensorStrength;
        }else if(i==3){
          vote[1][i]=0 * findNorthSensorStrength;
        }else if(i==4){
          vote[1][i]=1 * findNorthSensorStrength;
        }else if(i==5){   
          vote[1][i]=0 * findNorthSensorStrength; 
        }
      }
    }else if(azimuth>=3 && azimuth<180){
      for(int i=0; i<m;i++){
        if(i==0){
          vote[1][i]=0 * findNorthSensorStrength;
        }else if(i==1 || i==2){
          vote[1][i]=0 * findNorthSensorStrength;
        }else if(i==3){
          vote[1][i]=1 * findNorthSensorStrength;
        }else if(i==4){
          vote[1][i]=0 * findNorthSensorStrength;
        }else if(i==5){   
          vote[1][i]=0 * findNorthSensorStrength; 
        }
      }
    }
  }else{
    for(int i=0; i<m;i++){
      vote[1][i]=0;
    }
  }
  
//  Cruise Behaviour
  for(int i=0; i<m;i++){
    if(i==0){
      vote[2][i]=1;
    }else if(i==1 || i==2){
      vote[2][i]=0;
    }else if(i==3 || i==4){
      vote[2][i]=0;
    }else if(i==5){   
      vote[2][i]=-1; 
    }
  }
  
  centerArbiter(vote,G);
}

int centerArbiter(float vote[b][m], float gain[b]){
  
  float finalVote[m];
  float max_v = 0;
  int max_i = 0;
  
  for (int i = 0; i < m; i++) 
  {
      finalVote[i] = 0;
  }
  
  for (int i=0; i <  m ; i++) {
    for (int j=0; j <  b ; j++) {
       finalVote[i] = finalVote[i] + vote[j][i] * gain[j];
    }
  }

  for ( int i = 0; i < m; i++ )
  {
    if ( finalVote[i] > max_v )
    {
      max_v = finalVote[i];
      max_i = i;
    }
  }

  for ( int i = 0; i < m; i++ )
  {
    Serial.println(finalVote[i]);
  }

  if(max_i == 0){
    moveForward();
    Serial.println("forward");
  }
  
  if(max_i == 1){
    Serial.println("move Right");
    moveRight();
  }
  
  if(max_i == 2){
    Serial.println("move Left");
    moveLeft();
  }

  if(max_i == 3){
    rotateLeft();
    Serial.println("rotate-left");
  }
  
  if(max_i == 4){
    rotateRight();
    Serial.println("Rotate Right");
  }

  if(max_i == 5){
    stopMove();
    Serial.println("Stop");
  }

}


void moveForward(){
  myMotors.setSpeedA(100);
  myMotors.setSpeedB(100);
  myMotors.forward();
}

void moveLeft(){

}

void moveRight(){
  stopMove();
  delay(500);
  rotateRight();
  delay(700);
  moveForward();
  delay(500);
  rotateLeft();
  delay(700);
}

void rotateRight(){
  myMotors.setSpeedA(0);
  myMotors.setSpeedB(80);
  myMotors.forwardB();
  myMotors.backwardA();
}

void rotateLeft(){
  myMotors.setSpeedA(80);
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

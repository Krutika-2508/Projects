#include <Encoder.h>
#include <stdlib.h>

// Define Motor struct
// Error will be determined by either position or speed depending on motor type
struct Motor{
  int MControl;
  int P1;
  int P2;
  int PPR;
  double prevPos;
  double curPos;
  double desiredPos;
  double prevSpeed;
  double curSpeed;
  double desiredSp;
  double Error;
  double lastError;
  double totalError;
  int Input;
  boolean Forward;
};

// define motor struct constructor
struct Motor construct(Motor myMotor, int MC, int P1, int P2, int PPR){
  /* 
   *  Initialize motor struct
   *  MC = PWM control
   *  P1 & P2 (Determine direction, one is High and one is low)
   *  PPR (Pulses per revolution)
  */
  myMotor.MControl = MC;
  myMotor.P1 = P1;
  myMotor.P2 = P2;
  myMotor.PPR = PPR;
  myMotor.prevPos = 0;
  myMotor.curPos = 0;
  myMotor.desiredPos = 0;
  myMotor.prevSpeed = 0;
  myMotor.curSpeed = 0;
  myMotor.desiredSp = 0; 
  myMotor.Error = 0;
  myMotor.lastError = 0;
  myMotor.totalError = 0;
  myMotor.Input = 0;
  myMotor.Forward = true;
  return myMotor;
}

// Create Motor objects
Motor FWD = construct(FWD, 5, 6, 7, 2160);
Motor FWS = construct(FWS, 2, 3, 4, 7920);
Motor RWD = construct(RWD, 9, 12, 24, 2160);
Motor RWS = construct(RWS, 8, 10, 11, 7920); 

// Create Encoder objects; first pin of all is an interrupt pin
Encoder FWDE(23, 22);
Encoder FWSE(21, 20);
Encoder RWDE(38, 37);
Encoder RWSE(36, 35);

//Create other necessary parameters
double r_F = .05; // Front wheel radius (m)
double r_R = .05; // Rear wheel radius (m)
double L   = 0.5; // Wheel base      
//double R   = 2*L;
double time_spent = 0;
double In[4] = {0, 0, 0, 0}; // [w_F, w_R, phi_F, phi_R];
int U = 0;

double prevTime = 0;
double curTime = 0;
double dt = 0; 
double initialTime =0;

//FOR LED BLINK
const int ledPin =  13;
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 1000;           // interval at which to blink (milliseconds)


void setup() {
  // put your setup code here, to run once:
  //pinMode(ledPin, OUTPUT); // set the digital pin as output:0
  
  pinMode(FWD.MControl, OUTPUT);
  pinMode(FWS.MControl, OUTPUT);
  pinMode(RWD.MControl, OUTPUT);
  pinMode(RWS.MControl, OUTPUT);

  pinMode(FWD.P1, OUTPUT);
  pinMode(FWD.P2, OUTPUT);
  pinMode(FWS.P1, OUTPUT);
  pinMode(FWS.P2, OUTPUT);
  pinMode(RWD.P1, OUTPUT);
  pinMode(RWD.P2, OUTPUT);
  pinMode(RWS.P1, OUTPUT);
  pinMode(RWS.P2, OUTPUT);

  digitalWrite(FWD.P1, HIGH);
  digitalWrite(FWD.P2, LOW);
  digitalWrite(FWS.P1, HIGH);
  digitalWrite(FWS.P2, LOW);
  digitalWrite(RWD.P1, HIGH);
  digitalWrite(RWD.P2, LOW);
  digitalWrite(RWS.P1, HIGH);
  digitalWrite(RWS.P2, LOW);
  delay(500);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Initial =0;
  // Calculate Time and Time step
  delay(10);
  prevTime = curTime; 
  curTime = micros()/1000000.0;
  dt = curTime-prevTime;
  /*Serial.println("curTime");

  Serial.println(curTime);
  time_spent=(2*PI*L)/abs(w_R(curTime)-w_F(curTime));
  Serial.println("Time spent");

  Serial.println(time_spent);
  Serial.println(initialTime);

  if((initialTime+time_spent-1)< curTime && (initialTime+time_spent+1)> curTime ){
    Serial.println("R changed");
    R = -R/abs(R);
    initialTime = curTime;
    Serial.println(R);
  }*/
  
  
  
  // Calculate Inputs
  Inputs(In, U, r_F, r_R, curTime);
  
   // Check condition for Segway Mode
   /*R=1;      
   L=1;      // Bike length
   if(phi_F(curTime) == phi_R(curTime)){
      phi = phi_F(curTime);
        if(phi ==PI/2 ){
          print("Segway Mode Activated")
          R = (L/2)*(w_F(curTime)+w_R(curTime))*(1/(w_F(curTime)-w_R(curTime)))
          condition = (R-L/2)/(R+L/2)
            if(w_F(curTime)/w_R(curTime) == condition)
        }
   }*/
    
    
  //Serial.println("before:forward wheel speed");
  //Serial.println(w_F(curTime));
  
  
  FWD.desiredSp = w_F(curTime);//PI*sin(millis()/1000.0);//In[0];
  //Serial.println("after:forward wheel speed");
  //Serial.println(w_F(curTime));
  //Serial.println(FWD.desiredSp);
  RWD.desiredSp = w_R(curTime);//PI*sin(millis()/1000.0);//In[1];
  FWS.desiredPos = phi_F(curTime);//PI*sin(millis()/1000.0);//-In[2];
  RWS.desiredPos = phi_R(curTime);// PI*sin(millis()/1000.0);//-In[3];

  // Calculate States
  FWD = calculateState(FWD, FWDE);
  RWD = calculateState(RWD, RWDE);
  FWS = calculateState(FWS, FWSE);
  RWS = calculateState(RWS, RWSE);

  // Drive Bike
  FWD = PIDSpeed(FWD);
  RWD = PIDSpeed(RWD);
  FWS = PIDPosition(FWS);
  RWS = PIDPosition(RWS);
 
  //Serial.println("FWD");
  debugDriveMotor(FWD);
 // debugDriveMotor(FWD);
 //Serial.println("forward wheel Drive :cycle");
 //debugDriveMotor(FWD);
 //Serial.println("Rear wheel:cycle");
 //.nmklm,debugDriveMotor(RWD);
//Serial.println("Rear wheel:cycle");
//debugSteeringMotor(RWS);
  
}

Motor PIDSpeed(Motor M){
  //PID controller that controls speed of motor
  //Changes direction of motors as long as motor speed is 0;
  //Error is difference in desired speed and current speed
  // prevIn is the previous input that was given to the H-bridge/motor
  double P = 50;
  double I = 350;
  double D = 0;
  M.lastError = M.Error;
  M.Error = M.desiredSp-M.curSpeed;
  double accel = (M.Error-M.lastError)/dt;
  double dcOffset = 0;

  if (M.desiredSp !=0) dcOffset = 60*abs(M.desiredSp)/M.desiredSp;
  
  double newIn = dcOffset+M.Error*P; //proportional controller 
  M = AntiWindup(M, newIn);
  newIn += M.totalError*I;
  if ((M.curSpeed > 0 && M.desiredSp<=0) || (M.curSpeed<0 && M.desiredSp>=0)) {
    newIn = 0;
  } else if (M.prevSpeed != 0 && ((M.Forward & M.desiredSp < 0) || (!M.Forward & M.desiredSp > 0))){
      newIn = 0; // this is done to ensure that the motor has stopped and the reading is not a mistake
  } else if ((M.Forward && newIn<0) || (!M.Forward && newIn >0)){
    newIn = 0;
  } 
  
  if (newIn > 255){
    newIn = 255;
  } else if (newIn < -255){
    newIn = -255;
  } 

  if (M.prevSpeed == 0 && M.curSpeed == 0){
    if (M.Forward & M.desiredSp < 0) {
      M = switchDirection(M);
    } else if (!M.Forward && M.desiredSp > 0){
      M = switchDirection(M);
    } 
  }
  if (newIn<0) {
    M.Input = (int) -newIn;
    analogWrite(M.MControl, M.Input);
  }
  else {
    M.Input = (int) newIn;
    analogWrite(M.MControl, M.Input);
  }
  return M;
}

Motor PIDPosition(Motor M){
  //PID controller that controls speed of motor
  //Changes direction of motors as long as motor speed is 0;
  //Error is difference in desired speed and current speed
  // prevIn is the previous input that was given to the H-bridge/motor
  double P = 200;
  double I = 50;
  double D = 0;
  if (M.curSpeed != 0){D = -1*abs(M.curSpeed)/M.curSpeed;}
  M.lastError = M.Error;
  M.Error = M.desiredPos-M.curPos;
  double accel = (M.Error-M.lastError)/dt;
  double dcOffset = 0;

  if (M.curSpeed > 0) dcOffset = 73.5*abs(M.curSpeed)/M.curSpeed;
  if (M.curSpeed < 0) dcOffset = 100.5*abs(M.curSpeed)/M.curSpeed;
  
  double newIn = dcOffset+M.Error*(P+D);
  M = AntiWindup(M, newIn);
  newIn += M.totalError*I;
  if ((M.curSpeed > 0 && M.desiredPos<M.curPos) || (M.curSpeed<0 && M.desiredPos>M.curPos)) {
    newIn = 0;
    }else if (M.prevSpeed != 0 && ((M.Forward && M.desiredPos<M.curPos) || (!M.Forward && M.desiredPos>M.curPos))){
      newIn = 0; // this is done to ensure that the motor has stopped and the reading is not a mistake
  }
  
  if (newIn > 255){
    newIn = 255;
  } else if (newIn < -255){
    newIn = -255;
  } 

  if (M.prevSpeed == 0 & M.curSpeed == 0){
    if (M.Forward && M.desiredPos<M.curPos) {
      Serial.println("Switch1");
      M = switchDirection(M);
    } else if (!M.Forward && M.desiredPos>M.curPos){
      Serial.println("Switch2");
      M = switchDirection(M);
    } 
  }
  if (newIn<0){
    M.Input = (int) -newIn;
    analogWrite(M.MControl, M.Input);
  }
  else  {
    M.Input = (int) newIn;
    analogWrite(M.MControl, M.Input);
  }
  return M;

}


Motor AntiWindup(Motor M, double newIn){
  if (abs(M.totalError)>3) M.totalError= 3*abs(M.totalError)/M.totalError;
  if (newIn>=255 || newIn <= -255){
    return M;
  } else {
  M.totalError += M.Error*dt;
  if (M.totalError!=0) { M.totalError += -.001*dt*abs(M.totalError)/M.totalError; }
  return M;
  }
}

Motor switchDirection(Motor M){
  if (M.Forward){
    digitalWrite(M.P1, LOW);
    digitalWrite(M.P2, HIGH);
    M.Forward = false;
    }
  else{
    digitalWrite(M.P1, HIGH);
    digitalWrite(M.P2, LOW);
    M.Forward = true;
  }
  return M;
}

Motor calculateState(Motor M, Encoder E){
  M.prevPos = M.curPos;
  
  M.curPos = 2*PI*(((double) E.read())/M.PPR); //E.read = counts measured by encoder, 2pi(counts/partsperrev)=rotation of motor in radians
  M.prevSpeed = M.curSpeed;
  M.curSpeed = (M.curPos-M.prevPos)/dt;
  return M;
}



// This function is used to assign inputs in case of Bicycle mode
void Inputs(double arr[], int U, double r_F, double r_R, double t){
  // returns missing Input value
  // In = [w_F, w_R, phi_F, phi_R]
  if (U = 1){
    double u = w_R(t)*r_F*cos(phi_R(t))/(r_F*cos(phi_F(t)));
    arr[0] = u;
    arr[1] = w_R(t);
    arr[2] = phi_F(t);
    arr[3] = phi_R(t);
    return;
  } else if (U = 2){
    double u = w_F(t)*r_F*cos(phi_F(t))/(r_R*cos(phi_R(t)));
    arr[0] = w_F(t);
    arr[1] = u;
    arr[2] = phi_F(t);
    arr[3] = phi_R(t);
    return;
  } else if (U = 3){
    double u = acos(w_R(t)*r_R*cos(phi_R(t))/(w_F(t)*r_F));
    arr[0] = w_F(t);
    arr[1] = w_R(t);
    arr[2] = u;
    arr[3] = phi_R(t);
    return;
  } else if (U = 4){
    double u = acos(w_F(t)*r_F*cos(phi_F(t))/(w_R(t)*r_R));
    arr[0] = w_F(t);
    arr[1] = w_R(t);
    arr[2] = phi_F(t);
    arr[3] = u;
    return;
  }
}

// Input functions [w_F, w_R, phi_F, phi_R]
double w_F(double t){
  return  (0.4+(L*(0.3+sin(t))))/0.1;//3*PI*sin(t/4)
  //delay(10);2);  // not executed for U=1

}

double w_R(double t){
  
  return (0.4-(L*(0.3+sin(t))))/0.1;//*sin(t/4);
}

double phi_F(double t){
  return 0;//sin(t);//(PI/2)*sin(0);
}

double phi_R(double t){
  return 0;//sin(t); //-(PI/2)*sin(0);
}

//functions for debugging
void debugDriveMotor(Motor M){
  //Serial.println(M.curSpeed);
  //Serial.println("FWD");
  //Serial.println(M.desiredSp);
  //Serial.println(M.Error);
  //Serial.println(M.totalError);
  //Serial.println(M.Input);
  //Serial.println(M.Forward);
 
}

void debugSteeringMotor(Motor M){
  //Serial.println("new cycle");
  //Serial.println(M.curPos);
  //Serial.println(M.desiredPos);
  //Serial.println(M.Error);
  //Serial.println(M.totalError);
  //Serial.println(M.Input);
  //Serial.println(M.Forward);
}

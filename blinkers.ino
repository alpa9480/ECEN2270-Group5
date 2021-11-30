//Blinkers
//Calibrated for Alexis robot

//Pin values
#define rENC 2
#define lENC 4
#define button 6
#define rFWD 7
#define rBCK 8
#define rPWM 9
#define lFWD 11
#define lBCK 12
#define lPWM 10
#define rBlink 3
#define lBlink 19 //---I2C pin
#define trigPin 14
#define echoPin 15
#define objDetected 16
#define usInt 17

//Functions
void blinkers();
void rENC_ISR();
void lENC_ISR();
void driveStraight(float dist, bool direc);
void turn(float tAngle, bool direc); 
void speedUp(float frac);
void speedDown(bool wheel);
void usFunction();
void us_ISR();

//volatile globals
volatile bool rState;
volatile bool lState;
volatile bool blinkRightEnable = 0;
volatile bool blinkLeftEnable = 0;
volatile long rEC = 0; //pulse counter right
volatile long lEC = 0; //pulse counter left 
volatile bool drivingEnable; //allows driving
volatile bool hazState; //hazard light loop state
volatile int rASET = 0;
volatile int lASET = 0;
volatile int hazCounter = 0;

//constants
#define ENC_pf 1330 
#define ENC_pr 2500
#define RIGHT 1 //right turn macro
#define LEFT 0 //left turn macro
#define maxSpeed 150




void setup() {//---------------------------SETUP----------------------------------------------------------------------------------------------------
  Serial.begin(9600);
  pinMode(rFWD, OUTPUT);
  pinMode(rBCK, OUTPUT);
  pinMode(rPWM, OUTPUT);//----Analog Pin
  pinMode(lFWD, OUTPUT);
  pinMode(lBCK, OUTPUT);
  pinMode(lPWM, OUTPUT);//----Analog Pin
  pinMode(button, INPUT_PULLUP);
  pinMode(13, OUTPUT);//onboard LED
  pinMode(rBlink,OUTPUT);
  pinMode(lBlink,OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(objDetected, OUTPUT);
  pinMode(usInt,INPUT);

  digitalWrite(rFWD, LOW);
  digitalWrite(rBCK, LOW);
  digitalWrite(lFWD, LOW);
  digitalWrite(lBCK, LOW);
  digitalWrite(objDetected,LOW);
  analogWrite(rPWM,200);
  analogWrite(lPWM,200);


//Interrupts
  attachInterrupt(digitalPinToInterrupt(lENC), lENC_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(rENC), rENC_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(usInt), us_ISR, RISING);
}


//---------------------------------COMMANDS-----------------------------------------------------------------------------------------------
void loop() {//Do nothing
  //start wheels off, break lights on
  digitalWrite(lFWD,LOW);
  digitalWrite(rBCK,LOW);
  digitalWrite(rFWD,LOW);
  digitalWrite(lBCK,LOW);
  digitalWrite(lBlink,HIGH);
  digitalWrite(rBlink,HIGH);
  blinkRightEnable = 0;
  blinkLeftEnable = 0;
  
  do {digitalWrite(lBlink,HIGH); //No break lights
  digitalWrite(rBlink,HIGH);
  } while (digitalRead(button) == LOW); //wait for button press
  
  delay(1000); //wait one sec
  digitalWrite(lBlink,LOW); //No break lights
  digitalWrite(rBlink,LOW);

  usFunction();//---------------------------------------------------------------------------------------------->
  
  //-----------forward back test---------------
  driveStraight(2,1);
  turn(180, RIGHT);
  driveStraight(2,1);
  turn(180, LEFT);

  //-----------EC TEST---------------------
//  driveStraight(2,1);
//  turn(90, LEFT);
//  driveStraight(4,1);
//  turn(90, RIGHT);
//  driveStraight(2,1);
//  turn(90, RIGHT);
//  driveStraight(2,1);
//  turn(90,RIGHT);
//  driveStraight(4,1);
//  turn(90, LEFT);
//  driveStraight(2,1);
//  turn(90, LEFT);

}


//----------------------------------------------------------------------------------------------------------FUNCTIONS-------------------------------------------
void driveStraight(float dist, bool direc){//distance in feet, if direction is 1: fwd, if direction is zero: bckwd
  //reset the two encoder counters
  digitalWrite(lBlink,LOW);
  digitalWrite(rBlink,LOW);
  
  delay(250);
  rEC = 0;
  lEC = 0;
  int d = dist*ENC_pf; //calculates pulses needed for certain distance
  if (direc){//checks direction specified
    digitalWrite(lFWD,HIGH);
    digitalWrite(rFWD,HIGH);
    digitalWrite(lBCK,LOW);
    digitalWrite(rBCK,LOW);
  }
  else{
    digitalWrite(lFWD,LOW);
    digitalWrite(rFWD,LOW);
    digitalWrite(lBCK,HIGH);
    digitalWrite(rBCK,HIGH);

  }
  
  //start moving
  do {
      usFunction();
      if(drivingEnable){//DRIVE
          blinkLeftEnable = 0;
          blinkRightEnable = 0;
          if (direc){//checks direction specified
                digitalWrite(lFWD,HIGH);
                digitalWrite(rFWD,HIGH);
                digitalWrite(lBCK,LOW);
                digitalWrite(rBCK,LOW);
              }
              else{
                digitalWrite(lFWD,LOW);
                digitalWrite(rFWD,LOW);
                digitalWrite(lBCK,HIGH);
                digitalWrite(rBCK,HIGH);
                  }
          //speed up and slow down funcs-------------------------------------
          if (rEC == 0 && drivingEnable){ //speed up at beginning
              speedUp(0.8);
          }
          if(rEC >= d-200 && drivingEnable){ //slow down at end
            speedDown(RIGHT);
            if (rEC >= d){ 
              analogWrite(rPWM, 0);  
            }
            if (lEC >= d){
              analogWrite(lPWM, 0);
            }
          }
          if(lEC >= d-200 && drivingEnable){ //slow down at end
            speedDown(LEFT);
            if (rEC >= d){ 
              analogWrite(rPWM, 0);  
            }
            if (lEC >= d){
              analogWrite(lPWM, 0);
            }
          }
      }
      else{
        //HAZARD PROTOCOL
        //counter
        hazCounter += 1;
        hazCounter = hazCounter%100;
        if(hazCounter==0){
          hazState = digitalRead(rBlink);
          digitalWrite(rBlink,!hazState);
          digitalWrite(lBlink,!hazState); 
        }
      }
      
    } while(rEC < d && lEC < d); //wait for movement to finish
  
  //Turn direction control off
  digitalWrite(lFWD,LOW);
  digitalWrite(rFWD,LOW);
  digitalWrite(lBCK,LOW);
  digitalWrite(rBCK,LOW); 
  //Set the speed refrences to zero
  analogWrite(rPWM,0);
  analogWrite(lPWM,0);
}


//-----------------TURN----------------------------------------------
void turn(float tAngle, bool direc){ //turn some amount of degrees, 
  //direc=1: clockwise, 0:counterclockwise
  tAngle = tAngle/360;
  delay(250);
  lEC = 0;
  rEC = 0;
  float d = ENC_pr * tAngle;
  if (direc){//RIGHT TURN
    digitalWrite(lFWD,HIGH);
    digitalWrite(lBCK,LOW);
    digitalWrite(rFWD,LOW);
    digitalWrite(rBCK,HIGH);
    blinkRightEnable = 1;
    blinkLeftEnable = 0;
  }
  else{//LEFT TURN
    digitalWrite(lFWD,LOW);
    digitalWrite(lBCK,HIGH);
    digitalWrite(rFWD,HIGH);
    digitalWrite(rBCK,LOW);
    blinkRightEnable = 0;
    blinkLeftEnable = 1;
  }
  //speed up
  do {
      if (rEC == 0){ //speed up at beginning
              speedUp(0.5);
          }
          if(rEC >= d-100){ //slow down at end
            speedDown(RIGHT);
            if (rEC >= d){ 
              analogWrite(rPWM, 0);  
            }
          }
          if(lEC >= d-100){ //slow down at end
            speedDown(LEFT);
            if (lEC >= d){
              analogWrite(lPWM, 0);
            }
          }
    }while(rEC < d && lEC < d); //wait for movement to finish
  
  //Turn direction control off once condition met
  digitalWrite(lFWD,LOW);
  digitalWrite(rFWD,LOW);
  digitalWrite(lBCK,LOW);
  digitalWrite(rBCK,LOW);

  //Set the speed refrences to zero
  analogWrite(rPWM,0);
  analogWrite(lPWM,0);

  blinkRightEnable = 0;
  blinkLeftEnable = 0;
}

//----------------------------ISRs----------
void lENC_ISR(){
  lEC++;
}
void rENC_ISR(){
  rEC++;
  blinkers();
}
void us_ISR(){ 
  drivingEnable = 0; //driving disabled

  digitalWrite(rFWD,LOW); //stop moving
  digitalWrite(rBCK,LOW);
  digitalWrite(lFWD,LOW);
  digitalWrite(lBCK,LOW);
  
}


//----------------------SPEEDUP/SLOWDOWN-----------
void speedUp(float frac){
  rASET = 0;
  lASET = 0;
  for (int i = 0; i < maxSpeed * frac ; i+=5){
            //This loop speeds up the robot slowly
            analogWrite(rPWM,i);
            analogWrite(lPWM,i-2);
            rASET = i;
            lASET = i;
   }
}
void speedDown(bool wheel){
  if(wheel){//RIGHT
    rASET -= 5;
    analogWrite(rPWM,rASET);
  }
  else{//LEFT
    lASET -= 5;
    analogWrite(lPWM,lASET);
  }
}

//-------------------------------------------------------Blinkers------------------------------------------------
void blinkers(){
  if((rEC%150)==0){
    if(blinkLeftEnable==1){
      lState = digitalRead(lBlink);
      digitalWrite(lBlink,!lState);
    }
    else{
      digitalWrite(lBlink,LOW);
    }
    if(blinkRightEnable==1){
      rState = digitalRead(rBlink);
      digitalWrite(rBlink,!rState);
    }
    else{
      digitalWrite(rBlink,LOW);
    }
  }
}

//------------------------------------------------------------US Sensor----------------------------
void usFunction(){
  float echoTime;
  float calculatedDistance;
  //bool entityPresent;
  const float boundary = 12; //centimeters

  //send out an ultrasonic pulse for 10us
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);//pulsein returnsvlength of pulse                        
  calculatedDistance = echoTime*(0.017);//calculate the distance of the object
  if (calculatedDistance >= boundary){
    digitalWrite(objDetected, LOW); //do nothing
    drivingEnable = 1;
  }
  else{
    digitalWrite(objDetected, HIGH); //send to interrrupt pin
  }
  //Serial.println(calculatedDistance);
}
  

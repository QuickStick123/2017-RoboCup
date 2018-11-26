#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <Average.h>
#include <HMC5883L_Simple.h>
#include <EnableInterrupt.h> //enables lots of extra interupt pins
#include <Wire.h>  // Reference the I2C Library


volatile unsigned long temptimer;
int Switch;
// Compass
HMC5883L_Simple Compass;  // Store our compass as an object.
//Motors Varaibles
byte wheelNotActive;
// Set motor pins
/*#define IN1Motor1 27
#define IN2Motor1 29
#define IN1Motor2 33
#define IN2Motor2 31
#define IN1Motor3 35
#define IN2Motor3 37
// Motorpins for speed
#define SpeedPort1 6
#define SpeedPort2 5
#define SpeedPort3 4
*/
boolean CorrectRight;
boolean CorrectLeft;
boolean Straight;
unsigned long button[2];
boolean motorsOn = false;
boolean kicker = false;
boolean bypass[2];
Average<float> compassCalibrate(100);
unsigned long compassCalibrateTotal;
float compassCalibratedVal;

//Variables TSOPs
int pulsePin[16];  //Hold TSOP pin number
volatile unsigned long pulseTime[14];//Length of TSOP pulse, No pulse = 0.
volatile unsigned long pulseTimeCalc[14]; // Holds pulseTime[x] value for direction calculation
volatile boolean pinState[14];   //Stores TSOP connect pin state
unsigned long pulseTime1[14];  //Time when pin goies LOW
unsigned long pulseTime2[14];  //Time when pin goes HIGH
unsigned long tsopTime;  //Time TSOP power state
int maxTSOPValue;
boolean delayTSOPup;
int maxTSOPValueOUT;
boolean firstLOW[14];       //Indicate if a LOW has been received after the TSOP powerup before calculating pulseTime.
boolean tsopPowerState;     //Indicate if TSOP are ON or OFF.
int tsopPowerPin = A7;
int TSOPnumber;
int TSOPnumberOUT;
int Direction;
int DirectionGo =1;
byte directionGoing;
boolean haveBall;
volatile unsigned long haveBallTime;
//Variables Colour Snesor
unsigned long colorTimer;
int s0=43,s1=45,s2=47,s3=49;
int ColourPIN = 51;
volatile unsigned long ColourIN;
unsigned long ColourTime1;  //Time when pin goies LOW
unsigned long ColourTime2;  //Time when pin goes HIGH
boolean ColourPinState;
boolean colorSensorError = LOW;
boolean onWhite =LOW;
boolean onDarkGreen = LOW;
boolean onBlack = LOW;
int pastOnWhite =3;
int pastOnDarkGreen = 3;
int pastOnBlack = 3;
int pastOnLightGreenLeft = 3;
int pastOnLightGreenRight = 3;

//Compass Varibles
boolean runOnce = LOW;
float startCompassReading;
float calibratedCompassValue;
float compassValueCalc;
//Main Varibles
byte randomNumber1To3;
byte randomNumber1Or2;
boolean darkGreenLogic;
//ultrasonic Varibles
byte frontUltraSonic;
byte rightUltraSonic;
byte backUltraSonic;
byte leftUltraSonic;
volatile unsigned long UltraPingFront;
volatile unsigned long UltraPingRight;
volatile unsigned long UltraPingBack;
volatile unsigned long UltraPingLeft;
unsigned long UltraTime1[4];  //Time when pin goies LOW
unsigned long UltraTime2[4];  //Time when pin goes HIGH
boolean UltraEchoPinState[4];
int EchoPIN[4];
int TriggerPIN[4];
boolean FrontPing;
boolean RightPing;
boolean BackPing;
boolean LeftPing;
unsigned long PingSwitchTimer;
int PingState;
//Motor Comunication Varibles
  int drehM1;
  int drehM2;
  int drehM3;
  byte Identifier;

void setup(){
randomSeed(analogRead(0));
randomNumber1To3 = random(1, 4);
randomNumber1Or2 = random(1, 3);
Serial.begin(115200);
temptimer = millis();
Switch = 1;
//setup UltraSonics
  FrontPing = LOW;
  LeftPing = LOW;
  BackPing = LOW;
  RightPing = LOW;
  EchoPIN[0] = 15;
  EchoPIN[1] = 14;
  EchoPIN[2] = 2;
  EchoPIN[3] = 3;
  TriggerPIN[0] = 17; 
  TriggerPIN[1] = 16;
  TriggerPIN[2] = 4;
  TriggerPIN[3] = 5;
  pinMode (TriggerPIN[0],OUTPUT);
  pinMode (TriggerPIN[1],OUTPUT);
  pinMode (TriggerPIN[2],OUTPUT);
  pinMode (TriggerPIN[3],OUTPUT);
  pinMode (EchoPIN[0],INPUT);
  pinMode (EchoPIN[1],INPUT);
  pinMode (EchoPIN[2],INPUT);
  pinMode (EchoPIN[3],INPUT);
  PingSwitchTimer = micros();
  PingState = 1;
  
  enableInterrupt(15, ReadUltrasonicFront, CHANGE); 
  enableInterrupt(14, ReadUltrasonicRight, CHANGE); 
  enableInterrupt(2, ReadUltrasonicBack, CHANGE); 
  enableInterrupt(3, ReadUltrasonicLeft, CHANGE); 
//Setup Compass
  Wire.begin();
  Compass.SetDeclination(15, 0, 'E');  //Hobart's Magnetic Declination (Change For Nationals)
  Compass.SetSamplingMode(COMPASS_CONTINUOUS);//Gets Readings all the time since on contionuous mode.
  Compass.SetScale(COMPASS_SCALE_130);//Set the scale of how sentive it is.
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);

//Buttons and jumpers
  pinMode(22, INPUT_PULLUP); //Turning motors on and off
  pinMode(24, INPUT_PULLUP); //button for Kicker + Compass Calibration
  pinMode(23, INPUT_PULLUP); // Jumpers
  pinMode(25, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(29, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(35, INPUT_PULLUP);
  pinMode(37, INPUT_PULLUP);
  
//Set direction correction to LOW
 CorrectRight = LOW;
 CorrectLeft = LOW;
 Straight = HIGH;

//Setup ColourSensor
pinMode(s0,OUTPUT);
pinMode(s1,OUTPUT); 
pinMode(s2,OUTPUT);
pinMode(s3,OUTPUT);
digitalWrite(s1,HIGH);
digitalWrite(s0,LOW);
digitalWrite(s2,LOW);
digitalWrite(s3,HIGH);  
enableInterrupt(51, ReadColour, CHANGE);

//Setup TSOPS  
pulsePin[0] = A8;
pulsePin[1] = A9;
pulsePin[2] = A10;   
pulsePin[3] = A11;
pulsePin[4] = A12;
pulsePin[5] = A13;
pulsePin[6] = A14;
pulsePin[7] = A15;
pulsePin[8] = 19;
pulsePin[9] = 10;
pulsePin[10] = 11;
pulsePin[11] = 12;
pulsePin[12] = 13;
pulsePin[13] = 18;
pinMode(tsopPowerPin, OUTPUT);
digitalWrite(tsopPowerPin, HIGH);
tsopPowerState = HIGH;
tsopTime = micros();
  
  for (int i = 0; i <= 13; i++) {
      firstLOW[i] = LOW;
      pinMode(pulsePin[i], INPUT);
  }
  
enableInterrupt(A8, TSOP0, CHANGE); 
enableInterrupt(A9, TSOP1, CHANGE); 
enableInterrupt(A10, TSOP2, CHANGE); 
enableInterrupt(A11, TSOP3, CHANGE);
enableInterrupt(A12, TSOP4, CHANGE); 
enableInterrupt(A13, TSOP5, CHANGE);  
enableInterrupt(A14, TSOP6, CHANGE);  
enableInterrupt(A15, TSOP7, CHANGE);  
enableInterrupt(19, TSOP8, CHANGE);  
enableInterrupt(10, TSOP9, CHANGE);  
enableInterrupt(11, TSOP10, CHANGE);  
enableInterrupt(12, TSOP11, CHANGE);  
enableInterrupt(13, TSOP12, CHANGE);  
enableInterrupt(18, TSOP13, CHANGE);  
startCompassReading = EEPROM.readFloat(1);
}

void loop(){
  disableMotorsButton();
  calibrationKickerButton();
  IRCalc();
  nanoCom();
  pingUltraSonics();
  float compassValue = Compass.GetHeadingDegrees();
  compassValueCalc = compassValue;
  Serial.println();

  for (int c =0; c <= 13; c++) {
    Serial.print(pulseTime[c]);
    Serial.print("\t");
  }

  Serial.print(DirectionGo);
  /*Serial.print(" wheelNotActive: ");
  Serial.print(wheelNotActive);*/
  frontUltraSonic = UltraPingFront / 2 / 29.39;//diveded by 2 beacuse sound has to go their and back and we only want the distance there
  rightUltraSonic = UltraPingRight / 2 / 29.39;
  backUltraSonic = UltraPingBack / 2  / 29.39;//sound travels a centimeter every 29 microseconds so we divide by 29 to get the distance in centimeters.
  leftUltraSonic = UltraPingLeft / 2 / 29.39;
  detectColor();
  //lastColor();
  Serial.print(compassCalibratedVal);
  compassCalibration();
  CorrectDirection ();
  //haveBall = HIGH;
  haveBallCalc();
  //ChaseBall();
  if(onWhite == HIGH){
    //onWhiteFunc();
  }
  else if(haveBall == HIGH){
    //haveBallLogic();
  }
  else if(haveBall == LOW){
    //brakesALL();
    //ChaseBall();
  }
/*io
  Serial.print("\t");
  Serial.print(frontUltraSonic);
  Serial.print("\t");
  Serial.print(rightUltraSonic);
  Serial.print("\t");
  Serial.print(backUltraSonic);
  Serial.print("\t");
  Serial.print(leftUltraSonic);
  /*Serial.print(" HaveBall: ");
  Serial.print(haveBall);
  Serial.print(" ");*/
  Serial.print("\t");
  Serial.print(ColourIN);
  if (onWhite == HIGH) {
    Serial.print("\tWhite");
  }
  if (onDarkGreen == HIGH) {
    Serial.print("\tDarkGreen");
  }
  if (onBlack == HIGH) {
    Serial.print("\tBlack");
  }
  if (colorSensorError == HIGH) {
    Serial.print("\tError");
  }
  /*
  Serial.print("\tstartCompassReading: ");
  Serial.print(startCompassReading);
  Serial.print("\tcalibratedCompassValue: ");
  Serial.print(calibratedCompassValue);
  //Serial.print("UltraSonicsValue: ");
  //Serial.print(frontUltraSonic+backUltraSonic);
  Serial.print("\tcompassValueCalc: ");
  Serial.print(compassValueCalc);
  Serial.print("\tcompassValue: ");
  Serial.print(compassValue);
  */
  resetTSOP();
/*
  if (temptimer > millis()-700 && Switch ==1) {
    forward();
    Serial.print("Forwardtest");
  } 
  else if(Switch ==1) {
    Switch = 2;
    temptimer = millis();
  }
  if (temptimer > millis()-500 && Switch == 2) {
    brakesALL();
    Serial.print("Brakes");
  }
  else if (Switch ==2) {
    Switch = 3;
    temptimer = millis();
  }
  if (temptimer > millis()-500 && Switch ==3) {
    back();
  }
  else if(Switch ==3) {
    Switch = 4;
    temptimer = millis();
  }
  if (temptimer > millis()-500 && Switch ==4) {
    brakesALL();
  }
  else if(Switch ==4) {
    Switch = 1;
    temptimer = millis();
  }
*/
}

void nanoCom(){
  Wire.beginTransmission(0x30); // transmit to device #48
  Wire.write(DirectionGo * motorsOn); // sends one byte DirectionSend
  if (Straight == HIGH) {
    Identifier = 7; //7 Means don't adjust
    Wire.write(7);
  }
  else if (CorrectRight == HIGH) {
    Wire.write(9); // means correct to the right
    Identifier = 9;
  }
  else if (CorrectLeft == HIGH) {
    Wire.write(8); // correct to the left
    Identifier = 8;
  }
  Wire.write(kicker);
  kicker = false;
  Wire.write(Identifier + DirectionGo* motorsOn); //sum of numbers used to help filter out bad data
  Wire.endTransmission();    // stop transmitting
}

void disableMotorsButton(){
  if (button[0] <= millis() - 1000 && digitalRead(22)==LOW || bypass[0] == true){      // Button has been pressed
    if (bypass[0] == false){
      button[0] = millis();
    }
    bypass[0] = true;
    if (button[0] < millis() - 20 && digitalRead(22)==LOW){    // Really pressed
      button[0] = millis();
      bypass[0] = false;
      if (motorsOn == true){
        motorsOn = false;
        Serial.print("\tMotorsOff");
      }
      else {
        motorsOn = true;
        Serial.print("\tMotorsOn");
      }
    }
  }
}

void calibrationKickerButton(){
  if (digitalRead(24)==LOW || bypass[1] == true){      // Button has been pressed
    if(bypass[1] == false && button[1] < millis() - 50){
      button[1] = millis();
    }
    if (button[1] < millis() - 20 && digitalRead(24)==LOW || bypass[1] == true){// Really pressed
      bypass[1] = true;
      if (digitalRead(24)==HIGH){    // Realsed
        if (button[1] < millis() - 2000 ){
          kicker = true;
          Serial.print("\tKick");
          bypass[1] = false;
        }
        else {
          calibrate();
          Serial.print("\tCalibrate");          
          bypass[1] = false;
        }
      }
    }
  }
}

void jumpers(){


  
}
  
void calibrate(){
    compassCalibrate.clear();
    for (int i = 0; i < 100; i++) {
        compassCalibrate.push(Compass.GetHeadingDegrees());
        delay(10);
    }
    Serial.print(compassCalibrate.mean());
    startCompassReading = compassCalibrate.mean();
    EEPROM.writeFloat(1,startCompassReading);      

}

void detectColor(){
  if(ColourIN >= 0 && ColourIN < 200 && onWhite != HIGH){ //White
  colorSensorError = LOW;
  onWhite = HIGH;
  onDarkGreen = LOW;
  onBlack = LOW;
  colorTimer = micros();
  }
  else if(ColourIN >= 500 && ColourIN <= 1000 && onDarkGreen != HIGH){ //Dark Green
  colorSensorError = LOW;
  onWhite = LOW;
  onDarkGreen = HIGH;
  onBlack = LOW;
  colorTimer = micros();
  }
  else if(ColourIN >= 1001 && ColourIN <= 2000 && onBlack != HIGH){ //Black 
  colorSensorError = LOW;
  onWhite = LOW;
  onDarkGreen = LOW;
  onBlack = HIGH;
  colorTimer = micros();
  }
  else if(ColourIN >= 2001){ //Error
  onWhite = LOW;
  onDarkGreen = LOW;
  onBlack = LOW;
  colorSensorError = HIGH;
  }
}
/*
void lastColor(){
  if(onWhite == HIGH){//if on white
    pastOnWhite = 1; 
  }
  else if(pastOnWhite == 1 && onLightGreenLeft == HIGH || pastOnWhite == 1 && onLightGreenRight == HIGH){//if it was on white and know on ethier lightGreens
    pastOnWhite = 2;
  }
  else{//if its not on white for 3 colors.
    pastOnWhite = 3;
  }
  if(onDarkGreen == HIGH){//if on darkgreen.
    pastOnDarkGreen = 1; 
  }
  else if(pastOnDarkGreen== 1 && onLightGreenLeft == HIGH || pastOnDarkGreen== 1 && onLightGreenRight == HIGH || pastOnDarkGreen== 1 && onBlack == HIGH){//if it was on darkgreen and know on ethier lightGreens or black.
    pastOnDarkGreen = 2;
  }
  else{//if its not on darkgreen for 3 colors.
    pastOnDarkGreen = 3;
  }
  if(onBlack == HIGH){//if on black.
    pastOnBlack = 1; 
  }
  else if(pastOnBlack == 1 && onLightGreenLeft == HIGH || pastOnBlack == 1 && onLightGreenRight == HIGH || pastOnBlack == 1 && onDarkGreen == HIGH){//if it was on black and know on ethier lightGreens or darkgreen.
    pastOnBlack = 2;
  }
  else{//if its not on black for 3 colors.
    pastOnBlack = 3;
  }
  if(onLightGreenLeft == HIGH){//if on lightgreen left.
    pastOnLightGreenLeft = 1; 
  }
  else if(pastOnLightGreenLeft == 1 && onWhite == HIGH || pastOnLightGreenLeft == 1 && onBlack == HIGH || pastOnLightGreenLeft == 1 && onDarkGreen == HIGH){//if it was on lightGreen to the Left and know on ethier on white, black or darkgreen.
    pastOnLightGreenLeft = 2;
  }
  else{//if its not on LightGreenLeft for 3 colors.
    pastOnLightGreenLeft = 3;
  }
  if(onLightGreenRight == HIGH){//if on lightgreen right.
    pastOnLightGreenRight = 1; 
  }
  else if(pastOnLightGreenRight == 1 && onWhite == HIGH || pastOnLightGreenRight == 1 && onBlack == HIGH || pastOnLightGreenRight == 1 && onDarkGreen == HIGH){//if it was on lightGreen to the Right and know on ethier on white, black or darkgreen.
   pastOnLightGreenRight = 2;
  }
  else{//if its not on LightGreenRight for 3 colors.
    pastOnLightGreenRight = 3;
  }
}
*/
void compassCalibration(){
 if(compassValueCalc-startCompassReading>180){
 calibratedCompassValue=-1*(360-compassValueCalc+startCompassReading);
 }
 else if(compassValueCalc-startCompassReading<-180){
 calibratedCompassValue=360-startCompassReading+compassValueCalc;
 }
 else{
  calibratedCompassValue=compassValueCalc-startCompassReading;
 }
 /* if(startCompassReading - compassValueCalc){  
    calibratedCompassValue = startCompassReading - compassValueCalc;
    }
  else if(startCompassReading < compassValueCalc){    
    calibratedCompassValue = startCompassReading -360 + compassValueCalc ;
//   IF(compassValueCalc-startCompassReading>180,
// ((360-compassValueCalc)+startCompassReading)
//   IF(compassValueCalc-startCompassReading<-180
//   (360-startCompassReading+compassValueCalc),compassValueCalc-startCompassReading))
  }*/
}

void ReadColour(){    
  
    ColourPinState = digitalRead(ColourPIN);
      if ( (ColourPinState == LOW)  ) {
        ColourTime1 = micros(); //get time of pulse going down
        }
      else {
      ColourTime2 = micros();  //get time of pulse going up
      ColourIN = ColourTime2 - ColourTime1;  //measure time between down and up   
      }
}

void IRCalc() {
/*
////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
|||||||||||||||||||||||||||||||||||IR Reading|||||||||||||||||||||||||||||||||||
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////////
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////////
*/
for (int c =0; c <= 13; c++) {
  pulseTimeCalc[c] = pulseTime[c];
}
maxTSOPValue = pulseTimeCalc[0]; 
TSOPnumber = 0;
  for (int c = 0; c <= 13; c++){
    if (pulseTimeCalc[c] > maxTSOPValue) {
       maxTSOPValue  = pulseTimeCalc[c];
       TSOPnumber = c;
    }
  }    
if (maxTSOPValue >550) {   
    maxTSOPValue = maxTSOPValueOUT;
    TSOPnumber = TSOPnumberOUT;
}
else {
  maxTSOPValueOUT = maxTSOPValue;
TSOPnumberOUT = TSOPnumber;
}

//Direction and DirectionGo
//1 = Forward
//2 = Forward Left
//3 = Forward Right
//4 = Back Left
//5 = Back Right
//6 = Back

if (TSOPnumberOUT == 3 || TSOPnumberOUT == 4 || TSOPnumberOUT == 13){
  Direction = 1; //Forward
}
else if (TSOPnumberOUT == 10){
  Direction = 4; //Back Left/Back Right Dependent on field side or something.
}
else if (TSOPnumberOUT == 2) {
  Direction = 2; //Forward Left
}
else if (TSOPnumberOUT == 5) {
  Direction = 3; //Forward Right
}
else if (TSOPnumberOUT == 1 or TSOPnumberOUT == 0) {
  Direction = 4; //Back Left
}
else if (TSOPnumberOUT == 6 or TSOPnumberOUT == 7) {
  Direction = 5; //Back Right
}
else if (TSOPnumberOUT == 8 or TSOPnumberOUT == 9 or TSOPnumberOUT == 11 or TSOPnumberOUT == 12) {
  Direction = 6; //Back
}
if (Direction == 1 and (DirectionGo == 1 or DirectionGo == 2 or DirectionGo ==3)) {
  DirectionGo = Direction;
}
else if(Direction == 1 and DirectionGo == 5){
  DirectionGo = 3;
}
else if(Direction == 1 and DirectionGo == 4){
  DirectionGo = 2;
}
else if (Direction == 1 and DirectionGo == 6) {
  DirectionGo = 4;
}
if (Direction == 2 and (DirectionGo == 1 or DirectionGo == 2 or DirectionGo ==4)) {
  DirectionGo = Direction;
}
else if(Direction == 2 and DirectionGo == 3){
  DirectionGo = 1;
}
else if(Direction == 2 and DirectionGo == 6){
  DirectionGo = 4;
}
else if (Direction == 2 and DirectionGo == 5) {
  DirectionGo = 6;
}
if (Direction == 3 and (DirectionGo == 1 or DirectionGo == 3 or DirectionGo ==5)) {
  DirectionGo = Direction;
}
else if(Direction == 3 and DirectionGo == 2){
  DirectionGo = 1;
}
else if(Direction == 3 and DirectionGo == 6){
  DirectionGo = 5;
}
else if (Direction == 3 and DirectionGo == 4) {
  DirectionGo = 2;
}
if (Direction == 5 and (DirectionGo == 3 or DirectionGo == 5 or DirectionGo == 6)) {
  DirectionGo = Direction;
}
else if(Direction == 5 and DirectionGo == 4){
  DirectionGo = 6;
}
else if(Direction == 5 and DirectionGo == 1){
  DirectionGo = 3;
}
else if (Direction == 5 and DirectionGo == 2) {
  DirectionGo = 1;
}
if (Direction == 4 and (DirectionGo == 2 or DirectionGo == 4 or DirectionGo == 6)) {
  DirectionGo = Direction;
}
else if(Direction == 4 and DirectionGo == 1){
  DirectionGo = 2;
}
else if(Direction == 4 and DirectionGo == 5){
  DirectionGo = 6;
}
else if (Direction == 4 and DirectionGo == 3) {
  DirectionGo = 5;
}
if (Direction == 6 and (DirectionGo == 4 or DirectionGo == 5 or DirectionGo == 6)) {
  DirectionGo = Direction;
}
else if(Direction == 6 and DirectionGo == 3){
  DirectionGo = 5;
}
else if(Direction == 6 and DirectionGo == 2){
  DirectionGo = 4;
}
else if (Direction == 6 and DirectionGo == 1) {
  DirectionGo = 3;
}
for(int i=0;i<=13;i++){
//Serial.print (pulseTimeCalc[i]);
//Serial.print("  ");
if (pulseTime2[i] < micros()-833 && tsopPowerState ==HIGH){  //if 833increased to 3000 for power down microseconds have past with out a reading pulseTime =0. It's  is 833 microsecond between cycles. Pulses are merging together. 
    pulseTime[i] = 0;                  
  }
}
if (pulseTime2[14] <micros()-833) {
  pulseTime[14]=0;
}
if (pulseTime2[15] <micros()-833) {
  pulseTime[15]=0;
}
}

void ReadUltrasonicFront(){    
    UltraEchoPinState[0] = digitalRead(EchoPIN[0]);
      if ( (UltraEchoPinState[0] == HIGH && FrontPing == HIGH)  ) {
        UltraTime1[0] = micros(); //get time of pulse going up
        }
      else {
      UltraTime2[0] = micros();  //get time of pulse going down
      UltraPingFront = UltraTime2[0] - UltraTime1[0];  //measure time between up and down.  
      FrontPing = LOW;
      }
}

void ReadUltrasonicRight(){    
    UltraEchoPinState[1] = digitalRead(EchoPIN[1]);
      if ( (UltraEchoPinState[1] == HIGH && RightPing == HIGH)  ) {
        UltraTime1[1] = micros(); //get time of pulse going up
        }
      else {
      UltraTime2[1] = micros();  //get time of pulse going down
      UltraPingRight = UltraTime2[1] - UltraTime1[1];  //measure time between up and down.  
      RightPing = LOW;
      }
}

void ReadUltrasonicBack(){    
    UltraEchoPinState[2] = digitalRead(EchoPIN[2]);
      if ( (UltraEchoPinState[2] == HIGH && BackPing == HIGH)  ) {
        UltraTime1[2] = micros(); //get time of pulse going up
        }
      else {
      UltraTime2[2] = micros();  //get time of pulse going down
      UltraPingBack = UltraTime2[2] - UltraTime1[2];  //measure time between up and down.  
      BackPing = LOW;
      }
}

void ReadUltrasonicLeft(){    
    UltraEchoPinState[3] = digitalRead(EchoPIN[3]);
      if ( (UltraEchoPinState[3] == HIGH && LeftPing == HIGH)  ) {
        UltraTime1[3] = micros(); //get time of pulse going up
        }
      else {
      UltraTime2[3] = micros();  //get time of pulse going down
      UltraPingLeft = UltraTime2[3] - UltraTime1[3];  //measure time betweenup and down
      LeftPing = LOW;
      }
}

void pingUltraSonics(){
  if (PingState == 1 && PingSwitchTimer < (micros()-11600)){
    digitalWrite(TriggerPIN[0], HIGH);
    PingSwitchTimer =micros();
    PingState =2;
  }
  else if (PingState == 2 && PingSwitchTimer < micros()-10){
    FrontPing = HIGH;
    digitalWrite(TriggerPIN[0],LOW);  
    PingSwitchTimer =micros();
    PingState =3;
  }
  else if (PingState == 3 && PingSwitchTimer < micros()-11600){
    digitalWrite(TriggerPIN[1], HIGH);   
    PingSwitchTimer =micros();
    PingState =4;
  }
  else if (PingState == 4 && PingSwitchTimer < micros()-10){
    RightPing = HIGH;
    digitalWrite(TriggerPIN[1],LOW);  
    PingSwitchTimer =micros();
    PingState =5;
  }
  else if (PingState == 5 && PingSwitchTimer < (micros()-11600)){
    digitalWrite(TriggerPIN[2], HIGH);
    PingSwitchTimer =micros();
    PingState =6;
  }
  else if (PingState == 6 && PingSwitchTimer < micros()-10){
    BackPing = HIGH;
    digitalWrite(TriggerPIN[2],LOW);  
    PingSwitchTimer =micros();
    PingState =7;
  }
  else if (PingState == 7 && PingSwitchTimer < micros()-11600){
    digitalWrite(TriggerPIN[3], HIGH);
    PingSwitchTimer =micros();
    PingState =8;
  }
 else if (PingState == 8 && PingSwitchTimer < micros()-10){
    LeftPing = HIGH;
    digitalWrite(TriggerPIN[3],LOW);  
    PingSwitchTimer =micros();
    PingState =1;
  }
}
/*
void onWhiteFunc(){
  if(onWhite == HIGH){//if on white
    if(directionGoing == 1){//if it was going forwards
      brakesALL();
      delay(100);
      back();
      delay(350);
    }
    else if(directionGoing == 2){//if it was going forwardLeft
      brakesALL();
      delay(100);
      backRight();
      delay(350);
    }
    else if(directionGoing == 3){//if it was going forwardRight
      brakesALL();
      delay(100);
      backLeft();
      delay(350);
    }
    else if(directionGoing == 4){//if it was going backLeft
      brakesALL();
      delay(100);
      forwardRight();
      delay(350);
    }
    else if(directionGoing == 5){//if it was going backRight
      brakesALL();
      delay(100);
      forwardLeft();
      delay(350);
    }
    else if(directionGoing == 6){//if it was going back
      brakesALL();
      delay(100);
      forward();
      delay(350);
    }
  }
}

void haveBallLogic(){
  //forward();
    if(haveBall == HIGH){//if haveBall is HIGH
     if(onLightGreenLeft == HIGH){//If haveBall is HIGH and onLightGreenLeft is HIGH.
       if(pastOnDarkGreen == 2 && colorTimer >= micros()-200000){//contionus onto LightGreenLeft more.
         forwardLeft();
       }
       else if(frontUltraSonic+backUltraSonic >= 200 && frontUltraSonic+backUltraSonic <= 220){//If haveBall is HIGH, on lightGreenLeft and no robot on lightGreenLeft.
        if(frontUltraSonic <= 70){//if up oppenates goal's end.
          forwardRight();//goes forawrdRight onto the black to score.  
        }
        else{
          forward();//contines down lightGreenLeft. 
        }
       }
       else if(frontUltraSonic+backUltraSonic < 200){//Robot on onLightGreenLeft 
        forwardRight();
       }
     }
     else if(onLightGreenRight == HIGH){
      if(pastOnDarkGreen == 2 && colorTimer >= micros()-200000){//contionues onto LightGreenRight more.
        forwardRight();
      }
      else if(frontUltraSonic+backUltraSonic >= 200 && frontUltraSonic+backUltraSonic <= 220){//If haveBall is HIGH, on lightGreenRight and no robot on lightGreenRight.
        if(frontUltraSonic <= 70){//if up oppenates goal's end.
          forwardLeft();//goes forawrdLeft onto the black to score.  
        }
        else{
          forward();//contines down lightGreenRight. 
        }
       }
      else if(frontUltraSonic+backUltraSonic < 200){//Robot on onLightGreenRight
        forwardLeft();
       }
     }
     else if(onDarkGreen == HIGH){//If haveBall is HIGH and onDarkGreen is HIGH.
      if(pastOnLightGreenLeft == 2 && colorTimer >= micros()-100000){//centres it self a bit
        forwardRight();
      }
      else if(pastOnLightGreenRight == 2 && colorTimer >= micros()-100000){//centres it self a bit
        forwardLeft();  
      }
      else if(pastOnLightGreenLeft == 2){//Goes forwards if it came from the Left.
        forward();     
      }
      else if(pastOnLightGreenRight == 2){//Goes forwards if it came from the Right.
        forward();
      }
      else if(darkGreenLogic == LOW){
        forwardRight();
      }
      else if(darkGreenLogic == HIGH){
        forwardLeft();
      }
      else if(frontUltraSonic+backUltraSonic >= 75){//Goes forward if no oppenate stricker robot is on that plane.
        forward();
      }
      else if( randomNumber1Or2 == 2){//picks a random Number between 1-2 and goes that way.
        forwardRight();
        darkGreenLogic = LOW; 
      }
      else if( randomNumber1Or2 == 1){
        forwardLeft();
        darkGreenLogic = HIGH;        
      }
     }
     else if(onBlack == HIGH){
      if(pastOnLightGreenLeft == 2 && colorTimer >= micros()-50000){//center it a little when coming from the left
        forwardRight();              
      }
      else if(pastOnLightGreenRight == 2 && colorTimer >= micros()-50000){//center it a little when coming from the right
        forwardLeft();              
      }
      else{//goes forward if comes off DarkGreen or after it has finished going acros
        forward();
      }
     }
   }
}
*/
void haveBallCalc(){
  if(pulseTime[14] != 0 || pulseTime[15] != 0 ){ //if sensor 14 or 15 is getting a reading.
    haveBall = HIGH;//sets haveBall HIGH
    haveBallTime = micros();//starts haveballtimer
  }
  else if(haveBall == HIGH && haveBallTime <= micros()-333333 || directionGoing >= 4){//if the robot is going in a backwards direction or has not got a reading on the ball for a second. 
    haveBall = LOW;//sets haveBall LOW
  }
  
}

void CorrectDirection () {
  if (calibratedCompassValue >=-2.5 && calibratedCompassValue <= 2.5) {
  CorrectRight = LOW;
  CorrectLeft = LOW;
  Straight = HIGH;
  Serial.print("Straight");
}

else if (calibratedCompassValue < -2) {
  CorrectRight = HIGH;
  Straight = LOW;
  CorrectLeft = LOW;
  Serial.print("CorrectRight");
 }
else if (calibratedCompassValue > 2) {
  CorrectLeft = HIGH;
  Straight = LOW;
  CorrectRight = LOW;
  Serial.print("CorrectLeft");
  }
}
/*
void ChaseBall() { 

if (DirectionGo == 1) {
  //Serial.print("Foward");
  forward();
  }
else if (DirectionGo == 2) {
//  Serial.print("FowardLeft");
  forwardLeft();
}
else if (DirectionGo == 3) {
//  Serial.print("forwardRight");
  forwardRight();
}
else if (DirectionGo == 4) {
//  Serial.print("BackLeft");
  backLeft();
}
else if (DirectionGo == 5) {
//  Serial.print("BackRight");
  backRight();
}
else if (DirectionGo == 6) {
  //Serial.print("Back");
  back();
}
}*/
//////
//The 40 kHz carrier output of the ball shall be modulated with a trapezoidal (stepped) waveform of
//frequency 1.2 kHz. Each 833 microsecond cycle of the modulation waveform shall comprise 8 carrier
//pulses at full intensity, followed (in turn) by 4 carrier pulses at 1/4 of full intensity, four pulses at 1/16
//of full intensity and four pulses at 1/64 of full intensity, followed by a space (i.e. zero intensity) of
//about 346 microseconds. The peak current level in the LEDs shall be within the range of 45 to 55 mA.
//The radiant intensity of each LED shall be more than 20 mW/sr.
//So full intensity = 8+4+4+4=20 pulses, followed by 16, 12, and with 8 being furthest away. 
//All the pulses are merging together, so you are measuring from start of pulses to end.

void resetTSOP(){
if ((tsopTime+100000) < micros() and  tsopPowerState == HIGH){
    digitalWrite(tsopPowerPin, LOW);
    tsopPowerState = LOW;
    
    for(int i = 0; i <= 13; i++) {
      firstLOW[i] = LOW; 
    }
    tsopTime = micros();
    }
    
if (tsopTime+2000 < micros() && tsopPowerState == LOW && delayTSOPup == HIGH){
    digitalWrite(tsopPowerPin, HIGH);
    delayTSOPup = LOW;
    tsopTime = micros();
    }

if ((tsopTime+100 < micros()) and (delayTSOPup == LOW)) {
  delayTSOPup = HIGH;
  tsopPowerState = HIGH;
}
}

void TSOP0(){    
  if (tsopPowerState ==HIGH) {
    pinState[0] = digitalRead(pulsePin[0]);
      if ( (pinState[0] == LOW)  ) {
        pulseTime1[0] = micros(); //get time of pulse going down
        firstLOW[0] = HIGH;
        }
      else if( firstLOW[0] == HIGH) {
      pulseTime2[0] = micros();  //get time of pulse going up
      pulseTime[0] = pulseTime2[0] - pulseTime1[0];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[0] = micros();
  }
}   

void TSOP1(){    
  if (tsopPowerState ==HIGH) {
    pinState[1] = digitalRead(pulsePin[1]);
      if ( (pinState[1] == LOW)  ) {
        pulseTime1[1] = micros(); //get time of pulse going down
        firstLOW[1] = HIGH;
        }
      else if( firstLOW[1] == HIGH) {
      pulseTime2[1] = micros();  //get time of pulse going up
      pulseTime[1] = pulseTime2[1] - pulseTime1[1];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[1] = micros();
  }
}   

void TSOP2(){    
  if (tsopPowerState ==HIGH) {
    pinState[2] = digitalRead(pulsePin[2]);
      if ( (pinState[2] == LOW)  ) {
        pulseTime1[2] = micros(); //get time of pulse going down
        firstLOW[2] = HIGH;
        }
      else if( firstLOW[2] == HIGH) {
      pulseTime2[2] = micros();  //get time of pulse going up
      pulseTime[2] = pulseTime2[2] - pulseTime1[2];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[2] = micros();
  }
}   

void TSOP3(){    
  if (tsopPowerState ==HIGH) {
    pinState[3] = digitalRead(pulsePin[3]);
      if ( (pinState[3] == LOW)  ) {
        pulseTime1[3] = micros(); //get time of pulse going down
        firstLOW[3] = HIGH;
        }
      else if( firstLOW[3] == HIGH) {
      pulseTime2[3] = micros();  //get time of pulse going up
      pulseTime[3] = pulseTime2[3] - pulseTime1[3];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[3] = micros();
  }
}   

void TSOP4(){    
  if (tsopPowerState ==HIGH) {
    pinState[4] = digitalRead(pulsePin[4]);
      if ( (pinState[4] == LOW)  ) {
        pulseTime1[4] = micros(); //get time of pulse going down
        firstLOW[4] = HIGH;
        }
      else if( firstLOW[4] == HIGH) {
      pulseTime2[4] = micros();  //get time of pulse going up
      pulseTime[4] = pulseTime2[4] - pulseTime1[4];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[4] = micros();
  }
}   

void TSOP5(){    
  if (tsopPowerState ==HIGH) {
    pinState[5] = digitalRead(pulsePin[5]);
      if ( (pinState[5] == LOW)  ) {
        pulseTime1[5] = micros(); //get time of pulse going down
        firstLOW[5] = HIGH;
        }
      else if( firstLOW[5] == HIGH) {
      pulseTime2[5] = micros();  //get time of pulse going up
      pulseTime[5] = pulseTime2[5] - pulseTime1[5];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[5] = micros();
  }
}   

void TSOP6(){    
  if (tsopPowerState ==HIGH) {
    pinState[6] = digitalRead(pulsePin[6]);
      if ( (pinState[6] == LOW)  ) {
        pulseTime1[6] = micros(); //get time of pulse going down
        firstLOW[6] = HIGH;
        }
      else if( firstLOW[6] == HIGH) {
      pulseTime2[6] = micros();  //get time of pulse going up
      pulseTime[6] = pulseTime2[6] - pulseTime1[6];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[6] = micros();
  }
}   

void TSOP7(){    
  if (tsopPowerState ==HIGH) {
    pinState[7] = digitalRead(pulsePin[7]);
      if ( pinState[7] == LOW ) {
        pulseTime1[7] = micros(); //get time of pulse going down
        firstLOW[7] = HIGH;
        }
      else if( firstLOW[7] == HIGH) {
      pulseTime2[7] = micros();  //get time of pulse going up
      pulseTime[7] = pulseTime2[7] - pulseTime1[7];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[7] = micros();
  }
}   

void TSOP8(){    
  if (tsopPowerState == HIGH) {
    pinState[8] = digitalRead(pulsePin[8]);
      if ( (pinState[8] == LOW)  ) {
        pulseTime1[8] = micros(); //get time of pulse going down
        firstLOW[8] = HIGH;
        }
      else if( firstLOW[8] == HIGH) {
      pulseTime2[8] = micros();  //get time of pulse going up
      pulseTime[8] = pulseTime2[8] - pulseTime1[8];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[8] = micros();
  }
} 

void TSOP9(){    
  if (tsopPowerState ==HIGH) {
    pinState[9] = digitalRead(pulsePin[9]);
      if ( (pinState[9] == LOW)  ) {
        pulseTime1[9] = micros(); //get time of pulse going down
        firstLOW[9] = HIGH;
        }
      else if( firstLOW[9] == HIGH) {
      pulseTime2[9] = micros();  //get time of pulse going up
      pulseTime[9] = pulseTime2[9] - pulseTime1[9];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[9] = micros();
  }
}   

void TSOP10(){    
  if (tsopPowerState ==HIGH) {
    pinState[10] = digitalRead(pulsePin[10]);
      if ( (pinState[10] == LOW)  ) {
        pulseTime1[10] = micros(); //get time of pulse going down
        firstLOW[10] = HIGH;
        }
      else if( firstLOW[10] == HIGH) {
      pulseTime2[10] = micros();  //get time of pulse going up
      pulseTime[10] = pulseTime2[10] - pulseTime1[10];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[10] = micros();
  }
}  

 void TSOP11(){    
  if (tsopPowerState ==HIGH) {
    pinState[11] = digitalRead(pulsePin[11]);
      if ( (pinState[11] == LOW)  ) {
        pulseTime1[11] = micros(); //get time of pulse going down
        firstLOW[11] = HIGH;
        }
      else if( firstLOW[11] == HIGH) {
      pulseTime2[11] = micros();  //get time of pulse going up
      pulseTime[11] = pulseTime2[11] - pulseTime1[11];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[11] = micros();
  }
}   

void TSOP12(){    
  if (tsopPowerState ==HIGH) {
    pinState[12] = digitalRead(pulsePin[12]);
      if ( (pinState[12] == LOW)  ) {
        pulseTime1[12] = micros(); //get time of pulse going down
        firstLOW[12] = HIGH;
        }
      else if( firstLOW[12] == HIGH) {
      pulseTime2[12] = micros();  //get time of pulse going up
      pulseTime[12] = pulseTime2[12] - pulseTime1[12];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[12] = micros();
  }
}   

void TSOP13(){    
  if (tsopPowerState ==HIGH) {
    pinState[13] = digitalRead(pulsePin[13]);
      if ( (pinState[13] == LOW)  ) {
        pulseTime1[13] = micros(); //get time of pulse going down
        firstLOW[13] = HIGH;
        }
      else if( firstLOW[13] == HIGH) {
      pulseTime2[13] = micros();  //get time of pulse going up
      pulseTime[13] = pulseTime2[13] - pulseTime1[13];  //measure time between down and up   
      }
  }
  else { 
    pulseTime2[13] = micros();
  }
}     

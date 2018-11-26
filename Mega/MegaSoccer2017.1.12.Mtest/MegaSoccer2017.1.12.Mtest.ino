#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <Average.h>
#include <EnableInterrupt.h> //enables lots of extra interupt pins

//Compass from MagMaster
#include <Wire.h>
#include <HMC5883L.h>

#include <PixyI2C.h>

PixyI2C pixy;

HMC5883L compass; //Copy the folder "HMC5883L" in the folder "C:\Program Files\Arduino\libraries" and restart the arduino IDE.


int pixyX;


char fieldSide;
volatile char verifiedFieldSide;
char fieldSideFrontBack = 'B';
volatile char verifiedFieldSideFrontBack = 'B';

float xv, yv, zv;
volatile float headingDegrees;
float heading;
float heading_last;
float last_calibrated_values[3];

//calibrated_values[3] is the global array where the calibrated data will be placed
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
volatile float calibrated_values[3];
//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc

void transformation(float uncalibrated_values[3]) {
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] =
  {
    {1.069, 0.033, 0.032}, //M11-M13
    {0.019, 1.037, 0.011}, //M21-M23
    {0.021, 0.009, 1.118} //M31-M33
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] =
  {
    77.182,//X
    -75.096,//Y
    -326.966//Z
  };
  //calculation
  for (int i = 0; i < 3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i]; //Takes the appropiate bias of the compassval
  float result[3] = {0, 0, 0}; //resets data
  for (int i = 0; i < 3; ++i) // making loop for MX1-MX3 (Horizontal)
    for (int j = 0; j < 3; ++j) // making loop for vertical
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j]; //times
  for (int i = 0; i < 3; ++i) calibrated_values[i] = result[i];
}

//vector_length_stabilasation() - is the function of the magnetometer vector length stabilasation (stabilisation of the sphere radius)
float scaler;
boolean scaler_flag = false;
float normal_vector_length;
void vector_length_stabilasation() {
  //calculate the normal vector length
  if (scaler_flag == false)
  {
    getHeading();
    normal_vector_length = sqrt(calibrated_values[0] * calibrated_values[0] + calibrated_values[1] * calibrated_values[1] + calibrated_values[2] * calibrated_values[2]);
    scaler_flag = true;
  }
  //calculate the current scaler
  scaler = normal_vector_length / sqrt(calibrated_values[0] * calibrated_values[0] + calibrated_values[1] * calibrated_values[1] + calibrated_values[2] * calibrated_values[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  calibrated_values[0] = calibrated_values[0] * scaler;
  calibrated_values[1] = calibrated_values[1] * scaler;
  calibrated_values[2] = calibrated_values[2] * scaler;
}

unsigned long kickerForwardTimer;
unsigned long kickerCooldown;
boolean kickerForwardReset = true;
int kickerForwardsCount;
boolean canKick;

unsigned long I2CComCooldown;
volatile unsigned long temptimer;
int Switch;
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
byte correctAmount;
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
int LastDirectionGo = 1;
int t;
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
int DirectionGo = 1;
byte directionGoing;
boolean haveBall;
volatile unsigned long haveBallTime;
//Variables Colour Snesor
unsigned long colorTimer;
int s0 = 43, s1 = 45, s2 = 47, s3 = 49;
int ColourPIN = 51;
volatile unsigned long ColourIN;
unsigned long ColourTime1;  //Time when pin goies LOW
unsigned long ColourTime2;  //Time when pin goes HIGH
boolean ColourPinState;
boolean colorSensorError = LOW;
boolean onWhite = LOW;
boolean onDarkGreen = LOW;
boolean onBlack = LOW;
int pastOnWhite = 3;
int pastOnDarkGreen = 3;
int pastOnBlack = 3;
int pastOnLightGreenLeft = 3;
int pastOnLightGreenRight = 3;
int disableDirection = 0;
unsigned long disableDirectionTimer;
int directionSend;

//Compass Varibles Working
boolean runOnce = LOW;
volatile float headinDegrees;
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
//lasers
boolean laser1On = 1;
boolean laser2On = 1;

void setup() {
  Serial.begin(250000);
  pixy.init();
  randomSeed(analogRead(0));
  randomNumber1To3 = random(1, 4);
  randomNumber1Or2 = random(1, 3);
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
  pinMode (TriggerPIN[0], OUTPUT);
  pinMode (TriggerPIN[1], OUTPUT);
  pinMode (TriggerPIN[2], OUTPUT);
  pinMode (TriggerPIN[3], OUTPUT);
  pinMode (EchoPIN[0], INPUT);
  pinMode (EchoPIN[1], INPUT);
  pinMode (EchoPIN[2], INPUT);
  pinMode (EchoPIN[3], INPUT);
  PingSwitchTimer = micros();
  PingState = 1;
  //   enableInterrupt(15, ReadUltrasonicFront, CHANGE);  //moved to do individually
  //   enableInterrupt(14, ReadUltrasonicRight, CHANGE);
  //   enableInterrupt(2, ReadUltrasonicBack, CHANGE);
  enableInterrupt(3, ReadUltrasonicLeft, CHANGE);
  //Setup Compass
  Wire.begin();
  compass = HMC5883L();
  setupHMC5883L();
  //lasers
  pinMode(50, INPUT);
  pinMode(52, INPUT);
  enableInterrupt(50, Laser1, CHANGE);
  enableInterrupt(52, Laser2, CHANGE);

  //Buttons and jumpers
  pinMode(22, INPUT_PULLUP); //Turning motors on and off
  pinMode(24, INPUT_PULLUP); //button for Kicker + Compass Calibration
  pinMode(23, OUTPUT); // Jumpers
  pinMode(25, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(48, OUTPUT);
  digitalWrite(23, LOW);
  digitalWrite(25, LOW);
  digitalWrite(27, LOW);
  digitalWrite(29, LOW);
  digitalWrite(31, LOW);
  digitalWrite(33, LOW);
  digitalWrite(35, LOW);
  digitalWrite(37, LOW);
  digitalWrite(A6, LOW);
  digitalWrite(A5, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(26, LOW);
  digitalWrite(28, LOW);
  digitalWrite(30, LOW);
  digitalWrite(32, LOW);
  digitalWrite(34, LOW);
  digitalWrite(36, LOW);
  digitalWrite(38, LOW);
  digitalWrite(40, LOW);
  digitalWrite(42, LOW);
  digitalWrite(44, LOW);
  digitalWrite(46, LOW);
  digitalWrite(48, LOW);


  //Set direction correction to LOW
  CorrectRight = LOW;
  CorrectLeft = LOW;
  Straight = HIGH;

  //Setup ColourSensor
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  digitalWrite(s1, HIGH);
  digitalWrite(s0, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
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

void loop() {
  Serial.println(millis());
  disableMotorsButton();
  calibrationKickerButton();
  IRCalc();
  pingUltraSonics();
  frontUltraSonic = UltraPingFront / 2 / 29.39;//diveded by 2 beacuse sound has to go their and back and we only want the distance there
  rightUltraSonic = UltraPingRight / 2 / 29.39;
  backUltraSonic = UltraPingBack / 2  / 29.39;//sound travels a centimeter every 29 microseconds so we divide by 29 to get the distance in centimeters.
  leftUltraSonic = UltraPingLeft / 2 / 29.39;
  ultrasonicDistanceSide();
  ultrasonicDistanceFront();
  compassCalibration();
  float compassValue = headingDegrees;
  compassValueCalc = compassValue;
  GetHeadingDegrees();
  pixyCam();

  /////abcdefg123Serial.print(compassValue);
  /////abcdefg123Serial.println();
 //for (int c = 0; c <= 13; c++) {
    /////abcdefg123Serial.print(pulseTime[c]);
    /////abcdefg123Serial.print("\t");
  //  }
    ///////abcdefg123Serial.print(DirectionGo);
    ///////abcdefg123Serial.print(" wheelNotActive: ");
    ///////abcdefg123Serial.print(wheelNotActive);*/

  
  //lastColor();
  ///////abcdefg123Serial.print(compassCalibratedVal);
  
  
  //haveBall = HIGH;
  
  //ChaseBall();
  /////abcdefg123Serial.print("\tFrontUltraSonic: ");
  /////abcdefg123Serial.print(frontUltraSonic);
  /////abcdefg123Serial.print("\tRightUltrasonic: ");
  /////abcdefg123Serial.print(rightUltraSonic);
  /////abcdefg123Serial.print("\tBackUltrasonic: ");
  /////abcdefg123Serial.print(backUltraSonic);
  /////abcdefg123Serial.print("\tLeftUltrasonic: ");
  /////abcdefg123Serial.print(leftUltraSonic);
haveBallCalc();
detectColor();
CorrectDirection(); 
nanoCom();

  if (onWhite == HIGH) {
    onWhiteFunc();
  }
  else if (haveBall == HIGH) {
    Chaseball(); //haveBallLogic();
  }
  else if (haveBall == LOW) {
    Chaseball();
  }

  
  /*/////abcdefg123Serial.print(" HaveBall: ");
    /////abcdefg123Serial.print(haveBall);
    /////abcdefg123Serial.print(" ");
    /////abcdefg123Serial.print("\t");
    /////abcdefg123Serial.print(ColourIN);
    if (onWhite == HIGH) {
    /////abcdefg123Serial.print("\tWhite");
    }
    if (onDarkGreen == HIGH) {
    /////abcdefg123Serial.print("\tDarkGreen");
    }
    if (onBlack == HIGH) {
    /////abcdefg123Serial.print("\tBlack");
    }
    if (colorSensorError == HIGH) {
    /////abcdefg123Serial.print("\tError");
    }
    /////abcdefg123Serial.print("\tstartCompassReading: ");
    /////abcdefg123Serial.print(startCompassReading);
    /////abcdefg123Serial.print("\tcalibratedCompassValue: ");
    /////abcdefg123Serial.print(calibratedCompassValue);
    ///////abcdefg123Serial.print("UltraSonicsValue: ");
    ///////abcdefg123Serial.print(frontUltraSonic+backUltraSonic);
    /////abcdefg123Serial.print("\tcompassValueCalc: ");
    /////abcdefg123Serial.print(compassValueCalc);
    /////abcdefg123Serial.print("\tcompassValue: ");
    /////abcdefg123Serial.print(compassValue);*/
  resetTSOP();
  /*
    if (temptimer > millis()-700 && Switch ==1) {
      forward();
      /////abcdefg123Serial.print("Forwardtest");
    }
    else if(Switch ==1) {
      Switch = 2;
      temptimer = millis();
    }
    if (temptimer > millis()-500 && Switch == 2) {
      brakesALL();
      /////abcdefg123Serial.print("Brakes");
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

/*
int ColourDisableDirection(int targetDirection) {
  if (disableDirectionTimer > millis() - 1000) {
    if (targetDirection == 1) { //Forwards
      if (disableDirection == 1 || disableDirection == 2 || disableDirection == 3) {
        return 0;
      }
      else {
        return targetDirection;
      }
    }
    else if (targetDirection == 2) { //ForwardLeft
      if (disableDirection == 2 || disableDirection == 1 || disableDirection == 4) {
        return 0;
      }
      else {
        return targetDirection;
      }
    }
    else if (targetDirection == 3) { //ForwardRight
      if (disableDirection == 3 || disableDirection == 1 || disableDirection == 5) {
        return 0;
      }
      else {
        return targetDirection;
      }
    }
    else if (targetDirection == 4) { //BackLeft
      if (disableDirection == 4 || disableDirection == 2 || disableDirection == 6) {
        return 0;
      }
      else {
        return targetDirection;
      }
    }
    else if (targetDirection == 5) { //BackRight
      if (disableDirection == 5 || disableDirection == 3 || disableDirection == 6) {
        return 0;
      }
      else {
        return targetDirection;
      }
    }
    else if (targetDirection == 6) { //Back
      if (disableDirection == 6 || disableDirection == 4 || disableDirection == 5) {
        return 0;
      }
      else {
        return targetDirection;
      }
    }
  }
  else {
    disableDirection = 0;
    return targetDirection;
  }
}
*/

void nanoCom() {
  /*if(I2CComCooldown <= millis()-10){
    I2CComCooldown = millis();*/
  /////abcdefg123Serial.print("Test");
  Wire.beginTransmission(0x30); // transmit to device #48
  //DirectionGo
  //1 = Forward
  //2 = Forward Left
  //3 = Forward Right
  //4 = Back Left
  //5 = Back Right
  //6 = Back
 // directionSend = ColourDisableDirection(DirectionGo);
  /////abcdefg123Serial.print(directionSend);
  Wire.write(DirectionGo * motorsOn); // sends one byte DirectionSend
  if (Straight == HIGH) {
    Identifier = 7; //7 Means don't adjust
    Wire.write(7);
  }
  else if (CorrectRight == HIGH) {
    Wire.write(8); // means correct to the right
    Identifier = 8;
  }
  else if (CorrectLeft == HIGH) {
    Wire.write(9); // correct to the left
    Identifier = 9;
  }

  //Serial.println(calibratedCompassValue);
  
  correctAmount = byte(100 -(abs(calibratedCompassValue)/2));
  if (correctAmount < 85){
    correctAmount = 85; //Setting 30 as the maximum adjustemnt.
  //correctAmount = correctAmount + 2; //Adding 2 to override motor power differences.
  }
  //Serial.println(correctAmount);
  Wire.write(correctAmount);
  if (kicker == true && kickerCooldown < millis() - 5000) {
    canKick = true;
    kickerCooldown = millis();
  }
  else {
    canKick = false;
  }
  Wire.write(kicker * canKick);
  Wire.write(Identifier + correctAmount + directionSend * motorsOn + kicker * canKick); //sum of numbers used to help filter out bad dat
  kicker = false;
  Wire.endTransmission();    // stop transmitting
  //}
}

void disableMotorsButton() {
  if (button[0] <= millis() - 1000 && digitalRead(22) == LOW || bypass[0] == true) {   // Button has been pressed
    if (bypass[0] == false) {
      button[0] = millis();
    }
    bypass[0] = true;
    if (button[0] < millis() - 20 && digitalRead(22) == LOW) { // Really pressed
      button[0] = millis();
      bypass[0] = false;
      if (motorsOn == true) {
        motorsOn = false;
        /////abcdefg123Serial.print("\tMotorsOff");
      }
      else {
        motorsOn = true;
        /////abcdefg123Serial.print("\tMotorsOn");
      }
    }
  }
}

void calibrationKickerButton() {
  if (digitalRead(24) == LOW || bypass[1] == true) {   // Button has been pressed
    if (bypass[1] == false && button[1] < millis() - 50) {
      button[1] = millis();
    }
    if (button[1] < millis() - 20 && digitalRead(24) == LOW || bypass[1] == true) { // Really pressed
      bypass[1] = true;
      if (digitalRead(24) == HIGH) { // Realsed
        if (button[1] < millis() - 2000 ) {
          kicker = true;
          /////abcdefg123Serial.print("\tKick");
          bypass[1] = false;
        }
        else {
          calibrate();
          /////abcdefg123Serial.print("\tCalibrate");
          bypass[1] = false;

        }
      }
    }
  }
}

void jumpers() {


}

void calibrate() {
  compassCalibrate.clear();
  for (int i = 0; i < 50; i++) {
    GetHeadingDegrees();
    compassCalibrate.push(headingDegrees);
    delay(50);
  }
  /////abcdefg123Serial.print(compassCalibrate.mean());
  startCompassReading = compassCalibrate.mean();
  EEPROM.writeFloat(1, startCompassReading);
}

void GetHeadingDegrees() {
  float values_from_magnetometer[3];

  getHeading();
  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer);

  vector_length_stabilasation();

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  // float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  // heading += declinationAngle;


  heading = atan2(calibrated_values[0], calibrated_values[1]);


  if (heading < 0)
    heading += 2 * PI;
  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;
  // Convert radians to degrees for readability.
  if (heading_last == heading)
    headingDegrees = heading * 180 / M_PI;
  heading_last = heading;


  /////abcdefg123Serial.print("\t");
  /////abcdefg123Serial.print(calibrated_values[0]);
  /////abcdefg123Serial.print("\t");
  /////abcdefg123Serial.print(calibrated_values[1]);
  /////abcdefg123Serial.print("\t");
  /////abcdefg123Serial.print(calibrated_values[2]);
  /////abcdefg123Serial.print("\t");



}

void setupHMC5883L() {
  compass.SetScale(1.3);
  compass.SetMeasurementMode(Measurement_Continuous);
}

void getHeading() {
  MagnetometerRaw raw = compass.ReadRawAxis();
  xv = (float)raw.XAxis;
  yv = (float)raw.YAxis;
  zv = (float)raw.ZAxis;
}

void detectColor() {
  if (ColourIN >= 0 && ColourIN < 500 && onWhite != HIGH) { //White
    colorSensorError = LOW;
    onWhite = HIGH;
    onDarkGreen = LOW;
    onBlack = LOW;
    colorTimer = micros();
  }
  else if (ColourIN >= 500 && onDarkGreen != HIGH) { //Dark Green
    colorSensorError = LOW;
    onWhite = LOW;
    onDarkGreen = HIGH;
    onBlack = LOW;
    colorTimer = micros();
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

void compassCalibration() {
  if (compassValueCalc - startCompassReading > 180) {
    calibratedCompassValue = -1 * (360 - compassValueCalc + startCompassReading);
  }
  else if (compassValueCalc - startCompassReading < -180) {
    calibratedCompassValue = 360 - startCompassReading + compassValueCalc;
  }
  else {
    calibratedCompassValue = compassValueCalc - startCompassReading;
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

void ReadColour() {

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
  for (int c = 0; c <= 13; c++) {
    pulseTimeCalc[c] = pulseTime[c];
  }
  maxTSOPValue = pulseTimeCalc[0];
  TSOPnumber = 0;
  for (int c = 0; c <= 13; c++) {
    if (pulseTimeCalc[c] > maxTSOPValue) {
      maxTSOPValue  = pulseTimeCalc[c];
      TSOPnumber = c;
    }
  }
  if (maxTSOPValue > 550) {
    maxTSOPValue = maxTSOPValueOUT;
    TSOPnumber = TSOPnumberOUT;
  }
  else {
    maxTSOPValueOUT = maxTSOPValue;
    TSOPnumberOUT = TSOPnumber;
  }
  LastDirectionGo = DirectionGo;
  for (int i = 0; i <= 13; i++) {
    ///////abcdefg123Serial.print (pulseTimeCalc[i]);
    ///////abcdefg123Serial.print("  ");
    if (pulseTime2[i] < micros() - 833 && tsopPowerState == HIGH) { //if 833increased to 3000 for power down microseconds have past with out a reading pulseTime =0. It's  is 833 microsecond between cycles. Pulses are merging together.
      pulseTime[i] = 0;
    }
  }
}

void Chaseball() {
  if (TSOPnumberOUT == 3 || TSOPnumberOUT == 4 || TSOPnumberOUT == 13) {
    Direction = 1; //Forward
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
  else if (TSOPnumberOUT == 8 or TSOPnumberOUT == 9 or TSOPnumberOUT == 11 or TSOPnumberOUT == 12 || TSOPnumberOUT == 10) {
    Direction = 6; //Back
  }
  if (Direction == 1 and (DirectionGo == 1 or DirectionGo == 2 or DirectionGo == 3)) {
    DirectionGo = Direction;
  }
  else if (Direction == 1 and DirectionGo == 5) {
    DirectionGo = 3;
  }
  else if (Direction == 1 and DirectionGo == 4) {
    DirectionGo = 2;
  }
  else if (Direction == 1 and DirectionGo == 6) {
    DirectionGo = 4;
  }
  if (Direction == 2 and (DirectionGo == 1 or DirectionGo == 2 or DirectionGo == 4)) {
    DirectionGo = Direction;
  }
  else if (Direction == 2 and DirectionGo == 3) {
    DirectionGo = 1;
  }
  else if (Direction == 2 and DirectionGo == 6) {
    DirectionGo = 4;
  }
  else if (Direction == 2 and DirectionGo == 5) {
    DirectionGo = 6;
  }
  if (Direction == 3 and (DirectionGo == 1 or DirectionGo == 3 or DirectionGo == 5)) {
    DirectionGo = Direction;
  }
  else if (Direction == 3 and DirectionGo == 2) {
    DirectionGo = 1;
  }
  else if (Direction == 3 and DirectionGo == 6) {
    DirectionGo = 5;
  }
  else if (Direction == 3 and DirectionGo == 4) {
    DirectionGo = 2;
  }
  if (Direction == 5 and (DirectionGo == 3 or DirectionGo == 5 or DirectionGo == 6)) {
    DirectionGo = Direction;
  }
  else if (Direction == 5 and DirectionGo == 4) {
    DirectionGo = 6;
  }
  else if (Direction == 5 and DirectionGo == 1) {
    DirectionGo = 3;
  }
  else if (Direction == 5 and DirectionGo == 2) {
    DirectionGo = 1;
  }
  if (Direction == 4 and (DirectionGo == 2 or DirectionGo == 4 or DirectionGo == 6)) {
    DirectionGo = Direction;
  }
  else if (Direction == 4 and DirectionGo == 1) {
    DirectionGo = 2;
  }
  else if (Direction == 4 and DirectionGo == 5) {
    DirectionGo = 6;
  }
  else if (Direction == 4 and DirectionGo == 3) {
    DirectionGo = 5;
  }
  if (Direction == 6 and (DirectionGo == 4 or DirectionGo == 5 or DirectionGo == 6)) {
    DirectionGo = Direction;
  }
  else if (Direction == 6 and DirectionGo == 3) {
    DirectionGo = 5;
  }
  else if (Direction == 6 and DirectionGo == 2) {
    DirectionGo = 4;
  }
  else if (Direction == 6 and DirectionGo == 1) {
    DirectionGo = 3;
  }
  if (DirectionGo != LastDirectionGo) {
    t++;
    if (t == 3) {
      t = 0;
    }
    else {
      DirectionGo = LastDirectionGo;
    }
  }
}

void ReadUltrasonicFront() {
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

void ReadUltrasonicRight() {
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

void ReadUltrasonicBack() {
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

void ReadUltrasonicLeft() {
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

void pingUltraSonics() {
  if (PingState == 1 && PingSwitchTimer < (micros() - 11600)) {
    disableInterrupt(3);
    digitalWrite(TriggerPIN[0], HIGH);
    PingSwitchTimer = micros();
    PingState = 2;
  }
  else if (PingState == 2 && PingSwitchTimer < micros() - 10) {
    FrontPing = HIGH;
    enableInterrupt(15, ReadUltrasonicFront, CHANGE);
    digitalWrite(TriggerPIN[0], LOW);
    PingSwitchTimer = micros();
    PingState = 3;
  }
  else if (PingState == 3 && PingSwitchTimer < micros() - 11600) {
    disableInterrupt(15);
    digitalWrite(TriggerPIN[1], HIGH);
    PingSwitchTimer = micros();
    PingState = 4;
  }
  else if (PingState == 4 && PingSwitchTimer < micros() - 10) {
    RightPing = HIGH;
    enableInterrupt(14, ReadUltrasonicRight, CHANGE);
    digitalWrite(TriggerPIN[1], LOW);
    PingSwitchTimer = micros();
    PingState = 5;
  }
  else if (PingState == 5 && PingSwitchTimer < (micros() - 11600)) {
    disableInterrupt(14);
    digitalWrite(TriggerPIN[2], HIGH);
    PingSwitchTimer = micros();
    PingState = 6;
  }
  else if (PingState == 6 && PingSwitchTimer < micros() - 10) {
    BackPing = HIGH;
    enableInterrupt(2, ReadUltrasonicBack, CHANGE);
    digitalWrite(TriggerPIN[2], LOW);
    PingSwitchTimer = micros();
    PingState = 7;
  }
  else if (PingState == 7 && PingSwitchTimer < micros() - 11600) {
    disableInterrupt(2);
    digitalWrite(TriggerPIN[3], HIGH);
    PingSwitchTimer = micros();
    PingState = 8;
  }
  else if (PingState == 8 && PingSwitchTimer < micros() - 10) {
    LeftPing = HIGH;
    enableInterrupt(3, ReadUltrasonicLeft, CHANGE);
    digitalWrite(TriggerPIN[3], LOW);
    PingSwitchTimer = micros();
    PingState = 1;
  }
}

void delayOnWhite (int delayTime, int targetDirection) {
  unsigned long delayTimer = millis();
  while(delayTimer+delayTime > millis()){
    DirectionGo = targetDirection;
    GetHeadingDegrees();
    float compassValue = headingDegrees;
    compassValueCalc = compassValue;
    compassCalibration();
    CorrectDirection ();
    nanoCom();
  }
}

void onWhiteFunc() {
  if (onWhite == HIGH) { //if on white
    /*if (DirectionGo == 1) { //if it was going forwards
      DirectionGo = 0;
      nanoCom();
      delay(100);
      DirectionGo = 6; //6
      disableDirection = 1;
      disableDirectionTimer = millis();
      Straight = HIGH;
      CorrectRight = LOW;
      CorrectLeft = LOW;
      nanoCom();
      detectColor();
      ReadColour();
      delay(400);
    }
    else if (DirectionGo == 2) { //if it was going forwardLeft
      DirectionGo = 0;
      nanoCom();
      delay(100);
      DirectionGo = 5; //5
      disableDirection = 2;
      disableDirectionTimer = millis();
      Straight = HIGH;
      CorrectRight = LOW;
      CorrectLeft = LOW;
      nanoCom();
      detectColor();
      ReadColour();
      delay(400);
    }
    else if (DirectionGo == 3) { //if it was going forwardRight
      DirectionGo = 0;
      nanoCom();
      delay(100);
      DirectionGo = 4; //4
      disableDirection = 3;
      disableDirectionTimer = millis();
      Straight = HIGH;
      CorrectRight = LOW;
      CorrectLeft = LOW;
      nanoCom();
      detectColor();
      ReadColour();
      delay(500);
    }
    else if (DirectionGo == 4) { //if it was going backLeft
      DirectionGo = 0;
      nanoCom();
      delay(100);
      DirectionGo = 3; //3
      disableDirection = 4;
      disableDirectionTimer = millis();
      Straight = HIGH;
      CorrectRight = LOW;
      CorrectLeft = LOW;
      nanoCom();
      detectColor();
      ReadColour();
      delay(400);
    }
    else if (DirectionGo == 5) { //if it was going backRight
      DirectionGo = 0;
      nanoCom();
      delay(100);
      DirectionGo = 2; //2
      disableDirection = 5;
      disableDirectionTimer = millis();
      Straight = HIGH;
      CorrectRight = LOW;
      CorrectLeft = LOW;
      nanoCom();
      detectColor();
      ReadColour();
      delay(400);
    }
    else if (DirectionGo == 6) { //if it was going back
      DirectionGo = 0;
      nanoCom();
      delay(100);
      DirectionGo = 1; //1
      disableDirection = 6;
      disableDirectionTimer = millis();
      Straight = HIGH;
      CorrectRight = LOW;
      CorrectLeft = LOW;
      nanoCom();
      detectColor();
      ReadColour();
      delay(400);
    }*/
    if(verifiedFieldSideFrontBack == 'F' && verifiedFieldSide == 'L'){
      delayOnWhite(100, 0); //Break For 100 milliseconds
      delayOnWhite(400, 5); //Goes BackRight For 400 milliseconds
    }
    else if(verifiedFieldSideFrontBack == 'F' && verifiedFieldSide == 'R'){
      delayOnWhite(100, 0); //Break For 100 milliseconds
      delayOnWhite(400, 4); //Goes BackLeft For 400 milliseconds
    }
    else if(verifiedFieldSideFrontBack == 'B' && verifiedFieldSide == 'L'){
      delayOnWhite(100, 0); //Break For 100 milliseconds
      delayOnWhite(400, 3); //Goes ForwardRight For 400 milliseconds
    }
    else if(verifiedFieldSideFrontBack == 'B' && verifiedFieldSide == 'R'){
      delayOnWhite(100, 0); //Break For 100 milliseconds
      delayOnWhite(400, 2); //Goes ForwardLeft For 400 milliseconds
    }
  }
}

void haveBallLogic() {
  DirectionGo = 1;
  nanoCom();
  /////abcdefg123Serial.print("HaveballLogic: ");
  if (DirectionGo == 1) {
    kickerForwardsCount = 0;
    /////abcdefg123Serial.print("Forwards\t");
    if (kickerForwardReset == true) {
      kickerForwardReset = false;
      kickerForwardTimer = millis();
    }
  }
  else {
    kickerForwardsCount = kickerForwardsCount + 1;
    if (kickerForwardsCount == 4) {
      kickerForwardTimer = millis();
      kickerForwardReset = true;
      /////abcdefg123Serial.print("NotForwards\t");
      kickerForwardsCount = 0;
    }
  }
  if (DirectionGo == 1 && kickerForwardTimer < millis() - 500) {
    if (pixyX >= 158 && pixyX <= 162) {
      kickerForwardTimer = millis();
      kickerForwardReset = true;
      kicker = true;
      nanoCom();
      /////abcdefg123Serial.print("KICK");
    }
  }
  //Kicker Code for Robot
  /*
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
    }*/
}

void haveBallCalc() {
  /////abcdefg123Serial.print("\tlaser1On: ");
  /////abcdefg123Serial.print(laser1On);
  /////abcdefg123Serial.print("\tlaser2On: ");
  /////abcdefg123Serial.print(laser2On);
  if (laser1On == false && pulseTime[13] > 300 && laser2On == false) { //if sensor 14 or 15 is getting a reading. pulseTime[13] > 350 ||
    haveBall = HIGH;//sets haveBall HIGH
    haveBallTime = micros();//starts haveballtimer
    /////abcdefg123Serial.print("HaveBall");
  }
  else if (haveBall == HIGH && haveBallTime <= micros() - 333333 || directionGoing >= 4) { //if the robot is going in a backwards direction or has not got a reading on the ball for a second.
    haveBall = LOW;//sets haveBall LOW
  }
}

void CorrectDirection () {
  if (false) {
    if (pixyX <= 165 && pixyX >= 155) {
      CorrectLeft = LOW;
      Straight = HIGH;
      CorrectRight = LOW;
      /////abcdefg123Serial.print("Straight");
    //  correctAmount = 75;
    }
    else if (pixyX >= 165) {
      CorrectLeft = HIGH;
      Straight = LOW;
      CorrectRight = LOW;
      /////abcdefg123Serial.print("CorrectRight");
      //correctAmount = 75;
    }
    else if (pixyX <= 155) {
      CorrectLeft = LOW;
      Straight = LOW;
      CorrectRight = HIGH;
      /////abcdefg123Serial.print("CorrectLeft");
     // correctAmount = 75;
    }
  }
  else {
    if (calibratedCompassValue >= -1 && calibratedCompassValue <= 1) {
      CorrectRight = LOW;
      CorrectLeft = LOW;
      Straight = HIGH;
      /////abcdefg123Serial.print("Straight");
      //correctAmount = 85;
    }

    else if (calibratedCompassValue < 0) {
      CorrectRight = LOW;
      Straight = LOW;
      CorrectLeft = HIGH;
      /////abcdefg123Serial.print("CorrectLeft");
     // correctAmount = 85;
    }
    else if (calibratedCompassValue > 0) {
      CorrectLeft = LOW;
      Straight = LOW;
      CorrectRight = HIGH;
      /////abcdefg123Serial.print("CorrectRight");
     // correctAmount = 85;
    }
  }
}
/*
  void ChaseBall() {

  if (DirectionGo == 1) {
  ///////abcdefg123Serial.print("Foward");
  forward();
  }
  else if (DirectionGo == 2) {
  //  /////abcdefg123Serial.print("FowardLeft");
  forwardLeft();
  }
  else if (DirectionGo == 3) {
  //  /////abcdefg123Serial.print("forwardRight");
  forwardRight();
  }
  else if (DirectionGo == 4) {
  //  /////abcdefg123Serial.print("BackLeft");
  backLeft();
  }
  else if (DirectionGo == 5) {
  //  /////abcdefg123Serial.print("BackRight");
  backRight();
  }
  else if (DirectionGo == 6) {
  ///////abcdefg123Serial.print("Back");
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

void resetTSOP() {
  if ((tsopTime + 100000) < micros() and  tsopPowerState == HIGH) {
    digitalWrite(tsopPowerPin, LOW);
    tsopPowerState = LOW;

    for (int i = 0; i <= 13; i++) {
      firstLOW[i] = LOW;
    }
    tsopTime = micros();
  }

  if (tsopTime + 2000 < micros() && tsopPowerState == LOW && delayTSOPup == HIGH) {
    digitalWrite(tsopPowerPin, HIGH);
    delayTSOPup = LOW;
    tsopTime = micros();
  }

  if ((tsopTime + 100 < micros()) and (delayTSOPup == LOW)) {
    delayTSOPup = HIGH;
    tsopPowerState = HIGH;
  }
}

void TSOP0() {
  if (tsopPowerState == HIGH) {
    pinState[0] = digitalRead(pulsePin[0]);
    if ( (pinState[0] == LOW)  ) {
      pulseTime1[0] = micros(); //get time of pulse going down
      firstLOW[0] = HIGH;
    }
    else if ( firstLOW[0] == HIGH) {
      pulseTime2[0] = micros();  //get time of pulse going up
      pulseTime[0] = pulseTime2[0] - pulseTime1[0];  //measure time between down and up
    }
  }
  else {
    pulseTime2[0] = micros();
  }
}

void TSOP1() {
  if (tsopPowerState == HIGH) {
    pinState[1] = digitalRead(pulsePin[1]);
    if ( (pinState[1] == LOW)  ) {
      pulseTime1[1] = micros(); //get time of pulse going down
      firstLOW[1] = HIGH;
    }
    else if ( firstLOW[1] == HIGH) {
      pulseTime2[1] = micros();  //get time of pulse going up
      pulseTime[1] = pulseTime2[1] - pulseTime1[1];  //measure time between down and up
    }
  }
  else {
    pulseTime2[1] = micros();
  }
}

void TSOP2() {
  if (tsopPowerState == HIGH) {
    pinState[2] = digitalRead(pulsePin[2]);
    if ( (pinState[2] == LOW)  ) {
      pulseTime1[2] = micros(); //get time of pulse going down
      firstLOW[2] = HIGH;
    }
    else if ( firstLOW[2] == HIGH) {
      pulseTime2[2] = micros();  //get time of pulse going up
      pulseTime[2] = pulseTime2[2] - pulseTime1[2];  //measure time between down and up
    }
  }
  else {
    pulseTime2[2] = micros();
  }
}

void TSOP3() {
  if (tsopPowerState == HIGH) {
    pinState[3] = digitalRead(pulsePin[3]);
    if ( (pinState[3] == LOW)  ) {
      pulseTime1[3] = micros(); //get time of pulse going down
      firstLOW[3] = HIGH;
    }
    else if ( firstLOW[3] == HIGH) {
      pulseTime2[3] = micros();  //get time of pulse going up
      pulseTime[3] = pulseTime2[3] - pulseTime1[3];  //measure time between down and up
    }
  }
  else {
    pulseTime2[3] = micros();
  }
}

void TSOP4() {
  if (tsopPowerState == HIGH) {
    pinState[4] = digitalRead(pulsePin[4]);
    if ( (pinState[4] == LOW)  ) {
      pulseTime1[4] = micros(); //get time of pulse going down
      firstLOW[4] = HIGH;
    }
    else if ( firstLOW[4] == HIGH) {
      pulseTime2[4] = micros();  //get time of pulse going up
      pulseTime[4] = pulseTime2[4] - pulseTime1[4];  //measure time between down and up
    }
  }
  else {
    pulseTime2[4] = micros();
  }
}

void TSOP5() {
  if (tsopPowerState == HIGH) {
    pinState[5] = digitalRead(pulsePin[5]);
    if ( (pinState[5] == LOW)  ) {
      pulseTime1[5] = micros(); //get time of pulse going down
      firstLOW[5] = HIGH;
    }
    else if ( firstLOW[5] == HIGH) {
      pulseTime2[5] = micros();  //get time of pulse going up
      pulseTime[5] = pulseTime2[5] - pulseTime1[5];  //measure time between down and up
    }
  }
  else {
    pulseTime2[5] = micros();
  }
}

void TSOP6() {
  if (tsopPowerState == HIGH) {
    pinState[6] = digitalRead(pulsePin[6]);
    if ( (pinState[6] == LOW)  ) {
      pulseTime1[6] = micros(); //get time of pulse going down
      firstLOW[6] = HIGH;
    }
    else if ( firstLOW[6] == HIGH) {
      pulseTime2[6] = micros();  //get time of pulse going up
      pulseTime[6] = pulseTime2[6] - pulseTime1[6];  //measure time between down and up
    }
  }
  else {
    pulseTime2[6] = micros();
  }
}

void TSOP7() {
  if (tsopPowerState == HIGH) {
    pinState[7] = digitalRead(pulsePin[7]);
    if ( pinState[7] == LOW ) {
      pulseTime1[7] = micros(); //get time of pulse going down
      firstLOW[7] = HIGH;
    }
    else if ( firstLOW[7] == HIGH) {
      pulseTime2[7] = micros();  //get time of pulse going up
      pulseTime[7] = pulseTime2[7] - pulseTime1[7];  //measure time between down and up
    }
  }
  else {
    pulseTime2[7] = micros();
  }
}

void TSOP8() {
  if (tsopPowerState == HIGH) {
    pinState[8] = digitalRead(pulsePin[8]);
    if ( (pinState[8] == LOW)  ) {
      pulseTime1[8] = micros(); //get time of pulse going down
      firstLOW[8] = HIGH;
    }
    else if ( firstLOW[8] == HIGH) {
      pulseTime2[8] = micros();  //get time of pulse going up
      pulseTime[8] = pulseTime2[8] - pulseTime1[8];  //measure time between down and up
    }
  }
  else {
    pulseTime2[8] = micros();
  }
}

void TSOP9() {
  if (tsopPowerState == HIGH) {
    pinState[9] = digitalRead(pulsePin[9]);
    if ( (pinState[9] == LOW)  ) {
      pulseTime1[9] = micros(); //get time of pulse going down
      firstLOW[9] = HIGH;
    }
    else if ( firstLOW[9] == HIGH) {
      pulseTime2[9] = micros();  //get time of pulse going up
      pulseTime[9] = pulseTime2[9] - pulseTime1[9];  //measure time between down and up
    }
  }
  else {
    pulseTime2[9] = micros();
  }
}

void TSOP10() {
  if (tsopPowerState == HIGH) {
    pinState[10] = digitalRead(pulsePin[10]);
    if ( (pinState[10] == LOW)  ) {
      pulseTime1[10] = micros(); //get time of pulse going down
      firstLOW[10] = HIGH;
    }
    else if ( firstLOW[10] == HIGH) {
      pulseTime2[10] = micros();  //get time of pulse going up
      pulseTime[10] = pulseTime2[10] - pulseTime1[10];  //measure time between down and up
    }
  }
  else {
    pulseTime2[10] = micros();
  }
}

void TSOP11() {
  if (tsopPowerState == HIGH) {
    pinState[11] = digitalRead(pulsePin[11]);
    if ( (pinState[11] == LOW)  ) {
      pulseTime1[11] = micros(); //get time of pulse going down
      firstLOW[11] = HIGH;
    }
    else if ( firstLOW[11] == HIGH) {
      pulseTime2[11] = micros();  //get time of pulse going up
      pulseTime[11] = pulseTime2[11] - pulseTime1[11];  //measure time between down and up
    }
  }
  else {
    pulseTime2[11] = micros();
  }
}

void TSOP12() {
  if (tsopPowerState == HIGH) {
    pinState[12] = digitalRead(pulsePin[12]);
    if ( (pinState[12] == LOW)  ) {
      pulseTime1[12] = micros(); //get time of pulse going down
      firstLOW[12] = HIGH;
    }
    else if ( firstLOW[12] == HIGH) {
      pulseTime2[12] = micros();  //get time of pulse going up
      pulseTime[12] = pulseTime2[12] - pulseTime1[12];  //measure time between down and up
    }
  }
  else {
    pulseTime2[12] = micros();
  }
}

void TSOP13() {
  if (tsopPowerState == HIGH) {
    pinState[13] = digitalRead(pulsePin[13]);
    if ( (pinState[13] == LOW)  ) {
      pulseTime1[13] = micros(); //get time of pulse going down
      firstLOW[13] = HIGH;
    }
    else if ( firstLOW[13] == HIGH) {
      pulseTime2[13] = micros();  //get time of pulse going up
      pulseTime[13] = pulseTime2[13] - pulseTime1[13];  //measure time between down and up
    }
  }
  else {
    pulseTime2[13] = micros();
  }
}

void Laser1() {
  laser1On = digitalRead(50);
}

void Laser2() {
  laser2On = digitalRead(52);
}

void pixyCam() {
  /////abcdefg123Serial.print("Pixy:");
  int q;
  uint16_t blocks;
  char buf[32];
  blocks = pixy.getBlocks();

  if (blocks) {
    sprintf(buf, "Detected %d:\n", blocks);
    sprintf(buf, "  block %d: ", q);
    /////abcdefg123Serial.print(buf);
    pixyX = pixy.blocks[q].x;
  }
  /////abcdefg123Serial.print("pixyX: ");
  /////abcdefg123Serial.print(pixyX);
}

void ultrasonicDistanceSide() {
  if (rightUltraSonic + leftUltraSonic >= 155 && rightUltraSonic + leftUltraSonic <= 175) {
    if (rightUltraSonic > leftUltraSonic) {
      fieldSide = 'L';
      verifiedFieldSide = 'L';
    }
    else if (rightUltraSonic < leftUltraSonic) {
      fieldSide = 'R';
      verifiedFieldSide = 'R';
    }
  } 
  else {
    if (rightUltraSonic > leftUltraSonic) {
      fieldSide = 'L';
    } 
    else if (rightUltraSonic < leftUltraSonic) {
      fieldSide = 'R';
    }
  }
  /////abcdefg123Serial.print("Verified Field Side: ");
  /////abcdefg123Serial.println(verifiedFieldSide);
  /////abcdefg123Serial.print("Field Side: ");
  /////abcdefg123Serial.print(fieldSide);
}
  
void ultrasonicDistanceFront() {
    // field side 0 is left
    // field side 1 is right
    if (frontUltraSonic + backUltraSonic >= 160 && frontUltraSonic + backUltraSonic <= 180) {
      if (frontUltraSonic > backUltraSonic) {
        fieldSideFrontBack = 'B';
        verifiedFieldSideFrontBack = 'B';
      }
      else if (frontUltraSonic < backUltraSonic) {
        fieldSideFrontBack = 'F';
        verifiedFieldSideFrontBack = 'F';
      }
    } 
    else if (frontUltraSonic + backUltraSonic >= 195 && frontUltraSonic + backUltraSonic <= 215 ) {
      if (frontUltraSonic > backUltraSonic) {
        fieldSideFrontBack = 'B';
        verifiedFieldSideFrontBack = 'B';
      }
      else if (frontUltraSonic < backUltraSonic) {
        fieldSideFrontBack = 'F';
        verifiedFieldSideFrontBack = 'F';
      }
    } 
    else {
      if (frontUltraSonic > backUltraSonic) {
        fieldSideFrontBack = 'B';
      }
      else if (frontUltraSonic < backUltraSonic) {
        fieldSideFrontBack = 'F';
      }
    }
    /////abcdefg123Serial.print("Verified Field Side FrontBack: ");
    /////abcdefg123Serial.println(verifiedFieldSideFrontBack);
    /////abcdefg123Serial.print("Field Side FrontBack: ");
    /////abcdefg123Serial.print(fieldSideFrontBack);
  }

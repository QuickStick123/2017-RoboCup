/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////////////////////// Public Setup /////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

#include <Wire.h>

// Set motor pins
#define DirectionPort11 4
#define DirectionPort12 5
#define DirectionPort21 6
#define DirectionPort22 7
#define DirectionPort31 2
#define DirectionPort32 3

// Motorpins for speed
#define SpeedPort1 10
#define SpeedPort2 11
#define SpeedPort3 9

// Set kicker pin
#define kickerPin 12

byte Direction;
byte correct;
byte verify;
int error;
boolean kicker = false;
int speedMultiply = 8;


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////////////////////////// Main /////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void setup() {

  // Motors
  // PWM ports
  pinMode (SpeedPort1, OUTPUT);
  pinMode (SpeedPort2, OUTPUT);
  pinMode (SpeedPort3, OUTPUT);
  // Direction ports
  pinMode (DirectionPort11, OUTPUT);
  pinMode (DirectionPort12, OUTPUT);
  pinMode (DirectionPort21, OUTPUT);
  pinMode (DirectionPort22, OUTPUT);
  pinMode (DirectionPort31, OUTPUT);
  pinMode (DirectionPort32, OUTPUT);
  Wire.begin(0x30);                // Join i2c bus with address 0x30
  Serial.begin(115200);           // Start serial for output
}

void loop() {
  Wire.onReceive(i2cPacket); // On recive of packet run i2cPacket
}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////////////////////////// I2C //////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void i2cPacket(int howMany) {
  Direction = Wire.read(); // Read the Direction variable sent from the mega via i2c
  correct = Wire.read();   // Read the Motor_X variable sent from the mega via i2c and minus 100
  kicker = Wire.read();
  verify = Wire.read();    // Read the verify variable sent from the mega via i2c
  
  Serial.println();
  Serial.print(Direction);
  Serial.print("\t Correct: ");
  Serial.print(correct); 
  Serial.print("\t Kicker: ");
  Serial.print(kicker);  
  Serial.print("\t Verify: ");
  Serial.print(verify);  
  Serial.print("\t");
  
  
  if (Direction + correct + kicker  == verify){ // Check if the sum of all of the variables sent is the same as the verify variable
    checkDirection(); // Move in the coresponding direction
    kick(); // Check to see if it the kicker should fire
  }else{
    Serial.print("Check is invalid");
    Serial.print(error);
    error++;
  }
}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
//////////////////////// Motors /////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void turnM1(int velo){
  if(velo == 0){ // If input is 0, power to motors is set to full but both in directions to act as brakes
    analogWrite(SpeedPort1, 255); 
    digitalWrite(DirectionPort11, HIGH);
    digitalWrite(DirectionPort12, HIGH);
  }
  else if(velo > 0){ // If input is grater than 0, power to motors is set to input and set to turn in one direction
    velo = velo * 2.55;
    analogWrite(SpeedPort1, velo); 
    digitalWrite(DirectionPort11, LOW);
    digitalWrite(DirectionPort12, HIGH);         
  }
  else{
    velo = abs(velo) * 2.55; // Otherwise , power to motors is set to the absolute of the input and set to turn in the other direction
    analogWrite(SpeedPort1, velo); 
    digitalWrite(DirectionPort11, HIGH);
    digitalWrite(DirectionPort12, LOW);
  }
}

void turnM2(int velo){
  if(velo == 0){
    analogWrite(SpeedPort2, 255); 
    digitalWrite(DirectionPort22, HIGH);
    digitalWrite(DirectionPort21, HIGH);
  }
  else if(velo > 0){
    velo = velo * 2.55;
    analogWrite(SpeedPort2, velo); 
    digitalWrite(DirectionPort22, LOW);
    digitalWrite(DirectionPort21, HIGH);         
  }
  else{
    velo = abs(velo) * 2.55;
    analogWrite(SpeedPort2, velo); 
    digitalWrite(DirectionPort22, HIGH);
    digitalWrite(DirectionPort21, LOW);
  }
}

void turnM3(int velo){
  if(velo == 0){
    analogWrite(SpeedPort3, 255); 
    digitalWrite(DirectionPort32, HIGH);
    digitalWrite(DirectionPort31, HIGH);
  }
  else if(velo > 0){
    velo = velo * 2.55;
    analogWrite(SpeedPort3, velo); 
    digitalWrite(DirectionPort32, LOW);
    digitalWrite(DirectionPort31, HIGH);         
  }
  else{
    velo = abs(velo) * 2.55;
    analogWrite(SpeedPort3, velo); 
    digitalWrite(DirectionPort32, HIGH);
    digitalWrite(DirectionPort31, LOW);
  }
}

void checkDirection(){
    if(Direction == 0){
    brake();
  }else if (Direction == 1){
    forward();
  }else if (Direction == 2){
    forwardLeft();
  }else if (Direction == 3){
    forwardRight();
  }else if (Direction == 4){
    backLeft();
  }else if (Direction == 5){
    backRight();
  }else if (Direction == 6){
    back();
  }
}

void brake() {
  turnM1 (0);
  turnM2 (0);
  turnM3 (0);
}

void forward() {
 if (correct == 7) {// Go forward 
    turnM2 (10*speedMultiply);
    turnM1 (10*speedMultiply);
  }else if (correct == 9) { // Correct Left, puts less power to motor 1
    turnM1 (8*speedMultiply);
    turnM2 (10*speedMultiply);
  }else if (correct == 8) {// Correct Right, puts less power to motor 2
    turnM1 (10*speedMultiply);
    turnM2 (8*speedMultiply);
  }
  turnM3 (0);
}

void forwardLeft() {
  if (correct == 7){
    turnM2 (10*speedMultiply);
    turnM3 (-10*speedMultiply);
  }else if (correct == 8){
    turnM2 (8*speedMultiply);
    turnM3 (-10*speedMultiply);
  }else if (correct == 9){
    turnM2 (10*speedMultiply);
    turnM3 (-8*speedMultiply);
  }
  turnM1 (0);
}

void forwardRight() {
  if (correct == 7){
    turnM1 (10*speedMultiply);
    turnM3 (10*speedMultiply);
  }else if (correct == 8){
    turnM1 (10*speedMultiply);
    turnM3 (8*speedMultiply);
  }else if (correct == 9){
    turnM1 (8*speedMultiply);
    turnM3 (10*speedMultiply);
  }
  turnM2 (0);
}

void backLeft() {
  if (correct == 7){
    turnM1 (-10*speedMultiply);
    turnM3 (-10*speedMultiply);
  }else if (correct == 8){
    turnM1 (-8*speedMultiply);
    turnM3 (-10*speedMultiply);
  }else if (correct == 9){
    turnM1 (-10*speedMultiply);
    turnM3 (-8*speedMultiply);
  }
  turnM2 (0);
}

void backRight() {
  if (correct == 7){
    turnM2 (-10*speedMultiply);
    turnM3 (10*speedMultiply);
  }else if (correct == 8){
    turnM2 (-10*speedMultiply);
    turnM3 (8*speedMultiply);
  }else if (correct == 9){
    turnM2 (-8*speedMultiply);
    turnM3 (10*speedMultiply);
  }
  turnM1 (0);
}

void back() {
  if (correct == 7) {// Go back
    turnM1 (-10*speedMultiply);
    turnM2 (-10*speedMultiply);
  }else if (correct == 9) { // Correct Left, puts more power to motor 1
    turnM1 (-10*speedMultiply);
    turnM2 (-8*speedMultiply);
  }else if (correct == 8) {// Correct Right, puts more power to motor 2
    turnM1 (-8*speedMultiply);
    turnM2 (-10*speedMultiply);
  }
  turnM3 (0);
}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
//////////////////////// Kicker /////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void kick(){
  if (kicker == true && Direction == 0){ // Check if kicker boolean is true then kick
    digitalWrite(kickerPin, HIGH);// Set kicker trigger to high
    delay(80);// Wait 80 ms
    digitalWrite(kickerPin, LOW);// Set kicker trigger to low
    kicker = false;
  }
  
}



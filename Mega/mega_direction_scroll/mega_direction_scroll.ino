#include <Wire.h>

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
}

byte x = 0;

void loop() {
  for(int x = 1; x < 6;x++;){
  Wire.beginTransmission(0x30); // transmit to device #8
  Wire.write(x);              // sends one byte
  Wire.endTransmission();    // stop transmitting
  delay(1000);
  }
}

#include <Wire.h> // specify use of Wire.h library
#define ASD1115 0x48

unsigned int val = 0;
byte writeBuf[3];
byte buffer[3];
float t_micro;

const float VPS = 4.096 / 32768.0; // volts per step

void setup()   {

  Serial.begin(115200);
  Wire.begin(); // begin I2C

  writeBuf[0] = 1;
 
  writeBuf[1] = 0b11010010; // 0xC2 single shot off <== ORIGINAL - single conversion/ AIN1 & GND/ 4.096V/ Continuous (0)

  writeBuf[2] = 0b11100101; // bits 7-0  0x85 //869 SPS


  // setup ADS1115
  Wire.beginTransmission(ASD1115);  // ADC
  Wire.write(writeBuf[0]);
  Wire.write(writeBuf[1]);
  Wire.write(writeBuf[2]);  
  Wire.endTransmission();  

  delay(500);
pinMode(6,OUTPUT);
analogWrite(6,128);
}  // end setup

void loop() {

  buffer[0] = 0; // pointer
  Wire.beginTransmission(ASD1115);  // DAC
  Wire.write(buffer[0]);  // pointer
  Wire.endTransmission();

  Wire.requestFrom(ASD1115, 2);
  buffer[1] = Wire.read();  //
  buffer[2] = Wire.read();  //
  Wire.endTransmission();  

  // convert display results
  val = buffer[1] << 8 | buffer[2];

t_micro = micros();

  if (val > 32768) val = 0;
  Serial.print(t_micro/1000000.0,6);
  Serial.print("\t");
  Serial.print(val * VPS);
  Serial.print("\t");
  Serial.println(val);
 
} // end loop

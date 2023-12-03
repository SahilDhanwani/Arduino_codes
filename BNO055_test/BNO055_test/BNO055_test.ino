#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {

Serial.begin(115200);
myIMU.begin();
delay(1000);
int8_t temp=myIMU.getTemp();
myIMU.setExtCrystalUse(true);
}

void loop() {
  // put your main code here, to run repeatedly:
imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);//to get Acceleration//
imu::Vector<3> gyro =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);//to get Angular velocity//
imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);//to get Angle//

// Serial.print("Acceleration: ");
// Serial.print(" x: ");
// Serial.print(acc.x());
// Serial.print(" y: ");
// Serial.print(acc.y());
// Serial.print(" z: ");
// Serial.println(acc.z());

// Serial.print("Angular velocity: ");
// Serial.print(" x: ");
// Serial.print(gyro.x());
// Serial.print(" y: ");
// Serial.print(gyro.y());
// Serial.print(" z: ");
// Serial.println(gyro.z());

Serial.print("Angle: ");
Serial.print(" x: ");
Serial.print(euler.x());
Serial.print(" y: ");
Serial.print(euler.y());
Serial.print(" z: ");
Serial.println(euler.z());
Serial.println("");

delay(BNO055_SAMPLERATE_DELAY_MS);
}

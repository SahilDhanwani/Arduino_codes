#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {

Serial.begin(115200);
myIMU.begin();
myIMU.setExtCrystalUse(true);
}

void loop() {
imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);

Serial.print("Angle: ");
Serial.print(" x: ");
Serial.print(euler.x());
Serial.print(" y: ");
Serial.print(euler.y());
Serial.print(" z: ");
Serial.println(euler.z());
Serial.println("");

delay(100);
}

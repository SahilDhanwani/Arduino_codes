#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
int z;
byte l, m;
void setup(void)
{
  Serial.begin(115200);

  while (!Serial)
    delay(10);  // wait for serial port to open!

  //  Serial.println("Orientation Sensor Test"); Serial.println("");
  if (!bno.begin())
    while (1);
}

void loop(void)
{
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  printEvent(&orientationData);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}

void printEvent(sensors_event_t* event) {
  delay(1);
  z = event->orientation.x;
  if (digitalRead(5) == HIGH)
  {
    l = z;
    m = z >> 8;
    Serial.write(m);
    Serial.write(l);
  }
//    Serial.println(z);
}

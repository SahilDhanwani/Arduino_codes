#include <Encoder.h>
#define ea 5
float error,opv,p=0.1;
Encoder myEnc(3, 2);

void setup() {
  Serial.begin(9600);
  pinMode(ea,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
   error = 360 - newPosition;
   if(error > 0)
   {
    digitalWrite(7,HIGH);
    digitalWrite(8,LOW);
    opv = map(error * p,0,36,0,255);
    analogWrite(ea,opv);
   }
   else
   {
    digitalWrite(8,HIGH);
    digitalWrite(7,LOW);
    opv = map(error * p,0,36,0,255);
    analogWrite(ea,opv);
   }
   
}

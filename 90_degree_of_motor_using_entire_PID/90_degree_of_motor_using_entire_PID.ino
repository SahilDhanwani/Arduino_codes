#include <Encoder.h>
//<SAHIL.DHANWANI>
#define ea 5
#define pk 0.1
#define dk 0.1
#define ik 0.01
#define hv 40

float error_p,opv,correction,pv_error=0,error_sum=0,error_d = 0;
int i = 0;
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
   error_p = 360 - newPosition;
   if(error_p > 0)
   {
    digitalWrite(7,HIGH);
    digitalWrite(8,LOW);
    if(i!=0)
      error_d = pv_error - error_p;
      
    correction = (error_p * pk) + (error_d * dk) + (error_sum * ik);
    
    opv = map(correction,0,hv,0,255);
    analogWrite(ea,opv);
    
    pv_error = error_p;
    error_sum += error_p;
   }
   else
   {
    error_d = pv_error - error_p;
    error_p = error_p * -1;
    digitalWrite(8,HIGH);
    digitalWrite(7,LOW);
    
    correction = (error_p * pk) + (error_d * dk) + (error_sum * ik);
    
    opv = map(correction,0,hv,0,255);
    analogWrite(ea,opv);

    pv_error = error_p * -1;
    error_sum += error_p;
   }
}

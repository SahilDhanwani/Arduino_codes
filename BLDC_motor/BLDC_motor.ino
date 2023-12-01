#include <Servo.h>

Servo esc;

void setup() {
Serial.begin(9600);
esc.attach(8, 1000, 2000);
esc.write(0);
for(int i=0;i<=25;i++){esc.write(i);}
}

void loop() {
}

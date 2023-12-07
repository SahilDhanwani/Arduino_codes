
//BTS7960 motor driver sketch 

int R_IS = 40;
int R_EN = 41;
int R_PWM = 9;
int L_IS = 42;
int L_EN = 43;
int L_PWM = 10;
int piston_pin1 = 6;
int piston_pin2 = 7;
int value;
void setup() {
  // put your setup code here, to run once:
 pinMode(R_IS, OUTPUT);
 pinMode(R_EN, OUTPUT);
 pinMode(R_PWM, OUTPUT);
 pinMode(L_IS, OUTPUT);
 pinMode(L_EN, OUTPUT);
 pinMode(L_PWM, OUTPUT);
 pinMode(piston_pin1,OUTPUT);
 pinMode(piston_pin2,OUTPUT);
 digitalWrite(R_IS, LOW);
 digitalWrite(L_IS, LOW);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);
 Serial.begin(9600);
}

void loop() {

//  digitalWrite(piston_pin1,HIGH);
//  digitalWrite(piston_pin2,HIGH);
  // put your main code here, to run repeatedly:
  int i;
//  for(i = 0; i <= 255; i++){ //clockwise rotation
//   analogWrite(R_PWM, i);
//   analogWrite(L_PWM, 0);
//   delay(20);
//  }
//  delay(500);

  
//  for(i = 0; i<= 120; i++)
//  { //counter clockwise rotation
    value = analogRead(A0);
    Serial.println(value);
//    if(value < 255)
//    {
//    analogWrite(R_PWM , i);
//    analogWrite(L_PWM, 0);
//    delay(20);
//    }
//    else return;
//Serial.println(i);
//  }
  
//  delay(500);
}

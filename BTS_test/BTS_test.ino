  

const int ledPin = 13; 


void setup() {
  // put your setup code here, to run once:
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(36,OUTPUT);
  pinMode(37,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
   pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(ledPin, HIGH);   
delay(1000);        
digitalWrite(ledPin, LOW);    
delay(1000); 
analogWrite(2,50);
analogWrite(3,0);
analogWrite(4,50);
analogWrite(5,0);
analogWrite(10,50);
analogWrite(11,0);
analogWrite(36,50);
analogWrite(37,0);
}

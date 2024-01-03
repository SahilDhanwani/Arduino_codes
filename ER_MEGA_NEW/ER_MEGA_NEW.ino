#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include<Encoder.h>


//#define piston_l 41   //striker
//#define piston_r 43
//#define piston_dj 50 //f
//#define piston_gp 52 //
#define rpwm_dj 10
#define lpwm_dj 9


#define l1 'b'
#define r1 'a'
#define options 'k'
#define up 'i'
#define down 'g'
#define left1 'h'
#define right1 'j'
#define triangle 'e'
#define cross 'f'
#define square 'd'
#define circle 'c'
#define share '1'
#define touch_top_left 'm'
#define touch_top_mid 'q'
#define touch_top_right 'n'
#define touch_down_left 'o'
#define touch_down_mid 'q'
#define touch_down_right 'p'



HardwareSerial& odrive_serial = Serial3;
ODriveArduino odrive(odrive_serial);

int potRef=800 ,current = 0, right =0 , left = 0,i=0;
int a[]={6,-60,-30,0,30,60};
bool s = false;
bool strike_bool = true;

int mini = 300;   //757
int maxi = 900;   //905
int turret_left_angle = 734;//765;//754
int turret_right_angle = 890;//901;//895
int turret_mid_angle = 808;//831;//810
int turret_r_left_angle = 760;
int turret_r_mid_angle = 816;

//mid 362 - 370
int mid_angle;
int motor_pos_dj;
int target_pos_dj = (mini + maxi) / 2;
int PWMValue_dj = 0;
float previousTime_dj = 0; //for calculating delta
float previousError_dj = 0; //for calculating the derivative (edot)
float errorIntegral_dj = 0; //integral error
float currentTime_dj = 0; //time at the moment of calculation
float deltaTime_dj = 0; //time difference
float errorValue_dj = 0; //error
float edot_dj = 0; //derivative (de/dt)
float controlsignal_dj = 0; //u - Also called as process variable (PV)
bool ms = 0;
int motorDirection = 0;
float proportional = 0.5; //k_p = 0.5,0.4,3,0.4,0.4
float integral = 0.1; //k_i = 3,0.7,0.55,0,0.08
float derivative = 0; //k_d = 1,0.45,0,0
float P = 0.35;
float I = 0.2;
float D = 0;
float ct = 0;
float pt = 0;
int flag2 = 1;
char ch;
int f = 0;
bool dj_bool = true;
float dj_speed, dj_angle; //store double jugs speed
float add_speed; // for controlling speed of double jugs
bool dj_sep_jugs;


//Gripper vars//
int gripper_count=0;
int vertical_count=0,drop_count=0,angle_count=0,ramp_count=0;

int grip_piston=42,  //31
    vertical_pistons=40,   //38
    drop_piston=32;     //42

//RR Ramp
int ramp_pistons=38;   //32

//Turret & double jugs
int angle_pistons=33,//50
    shoot_piston=31;  //33

HardwareSerial& odrive1 = Serial1;
HardwareSerial& odrive2 = Serial3;

ODriveArduino o1(odrive1);
ODriveArduino o2(odrive2);
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  odrive1.begin(115200);
  odrive2.begin(115200);

  pinMode(21, INPUT);

  pinMode(lpwm_dj, OUTPUT);
  pinMode(rpwm_dj, OUTPUT);
//  pinMode(piston_dj, OUTPUT);

//  pinMode(piston_l, OUTPUT);
//  pinMode(piston_r, OUTPUT);
    pinMode(31, OUTPUT);
    pinMode(32, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(38, OUTPUT);
    pinMode(42, OUTPUT);
    pinMode(50, OUTPUT);
   
  digitalWrite(31, LOW);
  digitalWrite(38, LOW);
  digitalWrite(42, LOW);
  digitalWrite(32, LOW);
  digitalWrite(50, LOW);
  digitalWrite(26, LOW);


}

void loop() {
  //Serial.println(analogRead(A0));
  onAngle();
  previousTime_dj = 0; //for calculating delta t
  previousError_dj = 0; //for calculating the derivative (edot)
  errorIntegral_dj = 0; //integral error
  currentTime_dj = 0; //time in the moment of calculation
  deltaTime_dj = 0;
// Serial.println("HELLO TEAM");

  ch = 'z';
  if (Serial1.available() > 0 )
  {
    Serial.println("HELLO TEAM");
    ch = Serial1.read();
//    Serial.println(ch);
  }
 
  switch (ch)
  {
    case 'k' : //right piston     //R1
      Serial.println(ch);
      setv(0);
      break;

    case 'c' : //right position of Double Jugs //circle
     Serial.println(ch);
      motor_pos_dj = analogRead(A1);
     
      gettarget(380);
      current=right;
      setv(31);
      do
      {
        motor_pos_dj = analogRead(A1);
        calculatePID();
        driveMotor();

      } while (fabs(errorValue_dj) > 3 && PWMValue_dj != 0);
      gettarget(380);
      Serial.println(String("Speeddddd : ") + dj_speed);
      break;

    case 'd' : //left position of Double Jugs  //square
      Serial.println(ch);
      motor_pos_dj = analogRead(A1);
     
      gettarget(820);
      current=left;
      setv(27);
      do
      {
        motor_pos_dj = analogRead(A1);
        calculatePID();
        driveMotor();

      } while (fabs(errorValue_dj) > 3 && PWMValue_dj != 0);
      gettarget(820);
      Serial.println(String("Speeddddd : ") + dj_speed);
      break;
     
    case 'b' ://middle position
     Serial.println(ch);
    motor_pos_dj = analogRead(A1);
      if (dj_bool)
      {
        setv(11);//trial
        dj_bool = false;
      }
      else
      {
          setv(25.5);//trial
        dj_bool = true;
      }
      motor_pos_dj = analogRead(A1);
      gettarget(600);
      do
      {
        if(errorValue_dj<30)
           gettarget(dj_angle);
        motor_pos_dj = analogRead(A1);
        calculatePID();
        driveMotor();
      } while (fabs(errorValue_dj)>3 || PWMValue_dj!=0);
      delay(500);
      gettarget(dj_angle);
      Serial.println(String("Speeddddd : ") + dj_speed);
      break;

    case 'l' :  //share

      break;

    case 'm' ://speed up of double jugs/using Touchpad (top left corner)
      Serial.println(ch);
      dj_speed -= 0.1;
      setv(dj_speed);
      Serial.println("decrease speed");
      Serial.println(dj_speed);
      break;

    case 'n' ://speed down of double jugs //using Touchpad (top right corner)
      Serial.println(ch);
      dj_speed += 0.1;
      setv(dj_speed);
      Serial.println("increase speed");
      Serial.println(dj_speed);
      break;

    case 'o' ://angle down (towards left) of double jugs/using Touchpad (bottom left corner)
      Serial.println(ch);
      dj_angle -= 3;
      gettarget(dj_angle);
      Serial.println("decrease angle");
      Serial.println(dj_angle);
      do
      {
        motor_pos_dj = analogRead(A1);
        calculatePID();
        driveMotor();
      } while (fabs(errorValue_dj) > 3 && PWMValue_dj != 0);
      break;

    case 'p' ://angle up of double jugs/using Touchpad (bottom right corner)
      Serial.println(ch);
      dj_angle += 3;
      gettarget(dj_angle);
      Serial.println("increase angle");
      Serial.println(dj_angle);
      do
      {
        motor_pos_dj = analogRead(A1);
        calculatePID();
        driveMotor();
      } while (fabs(errorValue_dj) > 3 && PWMValue_dj != 0);
      break;

    case 'g': //DOWN
      Serial.println(ch);
   
      if(gripper_count%2==0)
      {
       digitalWrite(grip_piston,HIGH);
       gripper_count++;
      }
      else if(gripper_count%2==1)
      {
       digitalWrite(grip_piston,LOW);
       gripper_count++;  
      }
      break;
     
    case 'i': //UP
      Serial.println(ch);
      if(vertical_count%2==0)
      {
       digitalWrite(vertical_pistons,HIGH);
       vertical_count++;
      }
      else if(vertical_count%2==1)
      {
       digitalWrite(vertical_pistons,LOW);
       vertical_count++;  
      }
      break;
    case 'h'://LEFT
      Serial.println(ch);
      if(drop_count%2==0)
      {
       digitalWrite(drop_piston,HIGH);
       drop_count++;
      }
      else if(drop_count%2==1)
      {
       digitalWrite(drop_piston,LOW);
       drop_count++;  
      }
      break;

    case 'f'://cross
      Serial.println(ch);
      digitalWrite(shoot_piston,HIGH);
      delay(500);
      digitalWrite(shoot_piston,LOW);
      break;  

    case 'e'://TRIANGLE
      Serial.println(ch);
     if(angle_count%2==0)
      {
       digitalWrite(angle_pistons,HIGH);
       angle_count++;
      }
      else if(angle_count%2==1)
      {
       digitalWrite(angle_pistons,LOW);
       angle_count++;  
      }  
     break;

    case 'y'://option
      Serial.println(ch);
     if(ramp_count%2==0)
      {
       digitalWrite(ramp_pistons,HIGH);
       ramp_count++;
      }
      else if(ramp_count%2==1)
      {
       digitalWrite(ramp_pistons,LOW);
       ramp_count++;  
      }
     
     break;
    default:
      //Serial.println("Default;");
     break;
  }

}

void calculatePID()
{
  Serial.println("in PID");
  Serial.println(millis());

  currentTime_dj = millis();
  deltaTime_dj = (currentTime_dj - previousTime_dj) / 1000.0;
  previousTime_dj = currentTime_dj;
  //---
  errorValue_dj = motor_pos_dj - target_pos_dj;

  edot_dj = (errorValue_dj - previousError_dj) / deltaTime_dj;

  errorIntegral_dj = errorIntegral_dj + (errorValue_dj * deltaTime_dj);

  controlsignal_dj = (proportional * errorValue_dj) + (derivative * edot_dj) + (integral * errorIntegral_dj);

  previousError_dj = errorValue_dj;

}

void driveMotor()
{
  if ((errorValue_dj <= 3 && errorValue_dj >= -3))
  {
    PWMValue_dj = 0;
    controlsignal_dj = 0;
    Serial.println(String("PWMVALUE")+PWMValue_dj);
  }

  if (controlsignal_dj < 0) //negative value: CCW
    motorDirection = -1;
  else if (controlsignal_dj > 0) //positive: CW
    motorDirection = 1;
  else
    motorDirection = 0;

  PWMValue_dj = (int)fabs(controlsignal_dj);

  if (PWMValue_dj > 35)
    PWMValue_dj = 35; //keep 8.2 perfect
    else if(PWMValue_dj<9)
    PWMValue_dj=9;

  if (motor_pos_dj < mini)
  {
    PWMValue_dj = 8;
    motorDirection = -1;
    previousTime_dj = 0; //for calculating delta t
    previousError_dj = 0; //for calculating the derivative (edot)
    errorIntegral_dj = 0; //integral error
    currentTime_dj = 0; //time in the moment of calculation
    deltaTime_dj = 0;
  }
  else if (motor_pos_dj > maxi)
  {
    PWMValue_dj = 8;
    motorDirection = 1;
    previousTime_dj = 0; //for calculating delta t
    previousError_dj = 0; //for calculating the derivative (edot)
    errorIntegral_dj = 0; //integral error
    currentTime_dj = 0; //time in the moment of calculation
    deltaTime_dj = 0;
  }
  if (motorDirection == 1) //-1 == CCW
  {
    analogWrite(lpwm_dj, PWMValue_dj);
    analogWrite(rpwm_dj, 0);
  }
  else if (motorDirection == -1) // == 1, CW
  {
    analogWrite(rpwm_dj, PWMValue_dj);
    analogWrite(lpwm_dj, 0);
  }
  else
  {
    analogWrite(rpwm_dj, 0);
    analogWrite(lpwm_dj, 0);
    PWMValue_dj = 0;
  }

  Serial.print(errorValue_dj);
  Serial.print("  ");
  Serial.println(PWMValue_dj);
  Serial.print("  ");
  Serial.println(controlsignal_dj);
  Serial.print("  ");
  Serial.println(target_pos_dj);
  Serial.print("  ");
  Serial.println(motor_pos_dj);
  Serial.print("  ");


}

void gettarget(float a)
{
 

  if (a > mini && a < maxi)
  {
    target_pos_dj = a;
    Serial.println(target_pos_dj);
    errorValue_dj = motor_pos_dj - target_pos_dj;
    if(fabs(errorValue_dj)<30)
        {
          ct=0;
          pt=0;
          errorIntegral_dj=0;
          previousError_dj=0;
          deltaTime_dj=0;
          flag2=0;
          while((fabs(errorValue_dj)<30)&&(fabs(errorValue_dj)>1 || PWMValue_dj!=0)&&controlsignal_dj<30)
          {
            motor_pos_dj=analogRead(A1);
            cp();
            dm();
 
          }
        }
       f=1;
  }
}

void setv(float a)
{
  Serial.println("iin setv ");

  o1.SetVelocity(0, -a);
  o2.SetVelocity(0,a);

}

void onAngle(){

for(i=1;i<=a[0];i++)
    {
      if(a[i]>current)
      {
        break;
      }
    }
    if(i==1)
    {
      right=current;
      left=a[i];
    }
    else if(i==(a[0]+1))
    {
      if(a[i-1]==current)
      {
        if(i>2)
        {
          right=a[i-2];
          left=a[i];
        }
      }
      left=current;
      right=a[i-1];
     
    }
    else
    {
      if(a[i-1]==current)
      {
        if(i>2)
        {
          right=a[i-2];
          left=a[i];
        }
      }
      left=a[i];
      right=a[i-1];
    }
  }

  int ref()
  {
      return (potRef+1024*current/270);
  }
  void cp()
{
  ct = micros();
  if (flag2)
  {
    deltaTime_dj = (ct - pt) / 1000000.0;
  }
  flag2 = 1;
  pt = ct;
  errorValue_dj = motor_pos_dj - target_pos_dj;
  edot_dj = (errorValue_dj - previousError_dj) / deltaTime_dj;
  errorIntegral_dj = errorIntegral_dj + (errorValue_dj * deltaTime_dj);
  controlsignal_dj = (P * errorValue_dj) + (D * edot_dj) + (I * errorIntegral_dj);
  previousError_dj = errorValue_dj;
  Serial.print(errorIntegral_dj);
  Serial.println("incp");
}

void dm()
{
  Serial.println("indm");

  if (controlsignal_dj < 0) //negative value: CCW
  {
    motorDirection = -1;
  }
  else if (controlsignal_dj > 0) //positive: CW
  {
    motorDirection = 1;
  }
  else
  {
    motorDirection = 0;
  }

  PWMValue_dj = (int)fabs(controlsignal_dj);

  if ((errorValue_dj <= 1 && errorValue_dj >= -1))
  {
    PWMValue_dj = 0;
    controlsignal_dj = 0;
  }
  if (PWMValue_dj > 24)
  {
    PWMValue_dj = 24;//9 best
  }

  if (motor_pos_dj < mini)
  {
    PWMValue_dj = 8;
    motorDirection = -1;
  }
  else if (motor_pos_dj > maxi)
  {
    PWMValue_dj = 8;
    motorDirection = 1;
  }

  if (motorDirection == 1) //1 == CCW
  {

    analogWrite(lpwm_dj, PWMValue_dj);
    analogWrite(rpwm_dj, 0);
  }
  else if (motorDirection == -1) // == -1, CW
  {

    analogWrite(rpwm_dj, PWMValue_dj);
    analogWrite(lpwm_dj, 0);
  }
  else
  {
    analogWrite(rpwm_dj, 0);
    analogWrite(lpwm_dj, 0);
    PWMValue_dj = 0;

  }
}

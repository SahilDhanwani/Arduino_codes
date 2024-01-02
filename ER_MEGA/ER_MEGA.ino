#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include<Encoder.h>

HardwareSerial& odrive_serial = Serial3;
ODriveArduino odrive(odrive_serial);

char ch;
int f = 0;
float dj_speed, dj_angle; //store double jugs speed
float add_speed; // for controlling speed of double jugs

char a;
void setv(float);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  odrive_serial.begin(115200);

  pinMode(21,INPUT);
  pinMode(32,INPUT);
}

void loop() {

  ch = 'z';
  if (Serial1.available() > 0 )
  {
    Serial.println("HELLO TEAM");
    ch = Serial1.read();
    Serial.println(ch);
  }
  switch (ch)
  {
    case 'k' ://odrive stop  //OPTIONS
      setv(0);
      Serial.println("velocity 0");
      break;

    case 'l' :  //share
      left_hitter_servo();
      break;

    case 'm' ://speed up of double jugs/using Touchpad (top left corner)
      dj_speed -= 0.1;
      setv(dj_speed);
      Serial.println("decrease speed");
      Serial.println(dj_speed);
      break;

    case 'n' ://speed down of double jugs //using Touchpad (top right corner)
      dj_speed += 0.1;
      setv(dj_speed);
      Serial.println("increase speed");
      Serial.println(dj_speed);
      break;

    case 'o' ://angle down (towards left) of double jugs/using Touchpad (bottom left corner)
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
 
         
    default :
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
  }

  if (controlsignal_dj < 0) //negative value: CCW
    motorDirection = -1;
  else if (controlsignal_dj > 0) //positive: CW
    motorDirection = 1;
  else
    motorDirection = 0;

  PWMValue_dj = (int)fabs(controlsignal_dj);

  if (PWMValue_dj > 19)
    PWMValue_dj = 19; //keep 8.2 perfect

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
  if (motorDirection == -1) //-1 == CCW
  {
    analogWrite(lpwm_dj, PWMValue_dj);
    analogWrite(rpwm_dj, 0);
  }
  else if (motorDirection == 1) // == 1, CW
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
  if (a >= 1 && a <= 35)
  {
    //      if(ms==0)
    //      {
    //        odrive.SetVelocity(0,-a);
    //      }
    //      else
    //      {
    //         odrive.SetVelocity(1,a);
    //      }
    //      ms=!ms;
    setv(a - 1);
  }
  if (a > mini && a < maxi)
  {
    target_pos_dj = a;
    Serial.println(target_pos_dj);
    errorValue_dj = motor_pos_dj - target_pos_dj;
    //    if (fabs(errorValue_dj) < 30)
    //    {
    //      ct = 0;
    //      pt = 0;
    //      errorIntegral_dj = 0;
    //      previousError_dj = 0;
    //      deltaTime_dj = 0;
    //      flag2 = 0;
    //      while ((fabs(errorValue_dj) < 30) && (fabs(errorValue_dj) > 1 || PWMValue_dj != 0))
    //      {
    //        motor_pos_dj = analogRead(A0);
    //        cp();
    //        dm();
    //        //            gettarget();
    //      }
    //    }
    //    f = 1;
    //  }
  }
}

void setv(float a)
{
  Serial.println("iin setv ");

  odrive.SetVelocity(0, -a);
  odrive.SetVelocity(1, a);
  delay(1000);
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
  if (PWMValue_dj > 9)
  {
    PWMValue_dj = 9;//9 best
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

  if (motorDirection == -1) //-1 == CCW
  {

    analogWrite(lpwm_dj, PWMValue_dj);
    analogWrite(rpwm_dj, 0);
  }
  else if (motorDirection == 1) // == 1, CW
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

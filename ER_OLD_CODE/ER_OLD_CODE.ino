//                                 || श्री गणेशाय नमः ||

#include <Adafruit_ADS1X15.h>
#include <PS4BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
     
Adafruit_ADS1115 ads;
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

#define RPWM_2 7
#define RPWM_3 11
#define RPWM_4 9
#define RPWM_1 46
#define LPWM_2 6
#define LPWM_3 12
#define LPWM_4 10
#define LPWM_1 44

int x, y, w, v, v_x, v_y, v_w;
int v1, v2, v3, v4;
int targated_angle = 0, curr_angle, error, prev_error = 0, correction = 0, diff = 0;
int sick_right, sick_left;
int vertical, horizontal;
int prev_error_sick, diff_sick;
int targated_v, targated_h;
long error_v = 0, error_h = 0;
long error_sick = 0;
float angle;
float KP = 2.5, KD = 1;
char ch;
byte m, l;
int sick_count = 0;

int touch_x, touch_y;
unsigned long int stopping_time = 6000, curr_time;
unsigned long int rotation_time = 1000;
unsigned long int curr_rot_time, start_rot_time;

bool right_arena = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);
  Serial2.begin(115200);

//  while(1)
//  {
//    MPU();
//  }

#if !defined(_MIPSEL_)
  while (!Serial);
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1) { }; // Halt
  }

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
  pinMode(RPWM_1, OUTPUT);
  pinMode(RPWM_2, OUTPUT);
  pinMode(RPWM_3, OUTPUT);
  pinMode(RPWM_4, OUTPUT);
  pinMode(LPWM_1, OUTPUT);
  pinMode(LPWM_2, OUTPUT);
  pinMode(LPWM_3, OUTPUT);
  pinMode(LPWM_4, OUTPUT);
  pinMode(26, OUTPUT); 
  digitalWrite(26, LOW);

  ch = 'z';

//      while (1)
//      {
//        delay(200);
//        vertical = ads.readADC_SingleEnded(1);
//        sick_right = ads.readADC_SingleEnded(2);
//        sick_left = ads.readADC_SingleEnded(3);
//        vertical = map(vertical, 80, 26720, 5 , 6070); //done
//        sick_right = map(sick_right, 50  , 26700, 5, 6070); //done
//        sick_left = map(sick_left, 330, 26660, 5, 6070); //done
//  
//        Serial.println(String("Vertical - ") + vertical);
//        Serial.println(String("Left     - ") + sick_left);
//        Serial.println(String("RIGHT    - ") + sick_right);
//      }
}

void loop() {
  Usb.Task();

  if (PS4.connected())
  {
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117 || PS4.getAnalogButton(L2) > 15 || PS4.getAnalogButton(R2) > 15)
    {
      KP = 3;//2.5
      KD = 1;//1

      if (PS4.getAnalogHat(LeftHatX) < 138 && PS4.getAnalogHat(LeftHatX) > 116)
        x = 0;
      else
        x  = map(PS4.getAnalogHat(LeftHatX), 0, 255, -127, 128);

      if (PS4.getAnalogHat(LeftHatY) < 138 && PS4.getAnalogHat(LeftHatY) > 116)
        y = 0;
      else
        y = map(PS4.getAnalogHat(LeftHatY), 0, 255, 128, -127);

      w = PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2);

      angle = atan2(y, x);

      v = sqrt( (x * x) + (y * y) );
      #define Sahil_Dhanwani 29
      v = map(v, 0, 181 , 0, 150);   //100

      v_w = map(w, 16, 255, 0, 43);

      if (PS4.getAnalogButton(L2) > 4 || PS4.getAnalogButton(R2) > 4)
        targated_angle = curr_angle;

      movement(v, v_w, angle, KP, KD);

    }

    else
    {
      v = 0;
      v_w = 0;
      KP = 0;
      KD = 0;
      movement(v, v_w, angle, KP, KD);
    }

    if (PS4.getButtonClick(UP))    //(Piston assembly up on turret)
      Serial1.write('i');
    if (PS4.getButtonClick(DOWN))   //(Grip Rings)
      Serial1.write('g');
    if (PS4.getButtonClick(LEFT))   //(gripper piston vertical motion)
      Serial1.write('h');
    if (PS4.getButtonClick(RIGHT))     //(rotation to zero)
    { right_arena = false;
        PS4.setLed(Blue);
      Serial1.write('y');
      rotate(0);
    }
    if (PS4.getButtonClick(CIRCLE))   //(Turret angle)
        Serial1.write('e');
    if (PS4.getButtonClick(CROSS))     //(shoot piston)
        Serial1.write('f');
        
  
    if (right_arena == false)
    {
      if (PS4.getButtonClick(TRIANGLE))    //Centre speed away fence and type3
        {Serial1.write('j');
        }
      if (PS4.getButtonClick(SQUARE)) //(Speed type 1 near fence)
        Serial1.write('x');

      if (PS4.getButtonClick(L1))     // (rotation Left type 2)
      {
        Serial1.write('d'); //type2 speed
        rotate(-22);
      }

      if (PS4.getButtonClick(R1))     // (rotation right type 2)
      {
        Serial1.write('c'); //type2 speed
        rotate(19);
      }
    }
    else if (right_arena == true)//RIGHT ARENA POINT
    { PS4.setLed(140, 40, 233);
      if (PS4.getButtonClick(TRIANGLE))    //type2 CENTRE
        Serial1.write('b');
      
      if (PS4.getButtonClick(SQUARE)) //(Speed type 2 RIGHT)
        Serial1.write('a');     

      if (PS4.getButtonClick(L1))     // (rotation Left type 2)
      {
        Serial1.write('a'); //type1 speed
        rotate(-110);
      }

      if (PS4.getButtonClick(R1))     // (rotation right type 2)
      {
        Serial1.write('u'); //type2 right speed
        rotate(-70);
      }

    }
    if (PS4.getButtonClick(SHARE))   //(Stop Jugs)
      Serial1.write('k');

    if (PS4.getButtonClick(OPTIONS))  // Ramp
      Serial1.write('w');


    if (PS4.getButtonClick(R3))  //VERTICAL,HORIZONTAL,TOLERABLE ERROR,MIN SPEED, MAX SPEED, TIME, inc_dist
    {
      //VERTICAL,HORIZONTAL,TOLERABLE ERROR,MIN START SPEED,MIN SPEED, MAX SPEED, TIME, inc_dist
      //      SICKR(810,610); //right with rotate
      //      SICKR(670, 150); //right without rotate
      //      SICKL(535, 5715); //Center left
      //      SICKR(535,5510); //Center Right
      //      SICKL(1230, 5520); //Center Left stick to front
      //      SICKR(1230,5720); //center right stick to front
      //      SICKL(640, 510); //Left with rotate
      //      SICKL(640, 210); //Left without rotate

      // Type1 pole left  - SICKL(1000,2150,10,40,20,100,2000,500,800);
      // Type2 pole right - SICKR(1000,2500,10,40,20,100,2000,500,800);

      PS4.setLed(Red);
      if (sick_count % 6 == 0)
      {
        //VERTICAL,HORIZONTAL,TOLERABLE ERROR,MIN PWMs, MIN SPEED,MAX SPEED,TIME,inc_dist,dec_dist
        SICKL(600, 500, 10, 40, 20, 150, 5000, 500, 2500);  //Left picking with rotate

        sick_count++;
      }
      else if (sick_count % 6 == 1)
      {
        Serial1.write('x');//pole one speed (10.8)
        SICKL(1030, 2150, 10, 40, 20, 120, 2000, 500, 800); //Pole one left shooting point
        sick_count++;
      }
      else if (sick_count % 6 == 2)
      {
        SICKL(700, 5715, 10, 40, 20, 150, 5000, 500, 3500); //Center point far from fence
        sick_count++;
      }
      else if (sick_count % 6 == 3)
      {
        SICKR(600, 410, 10, 40, 20, 150, 5000, 500, 3500);  //right ring picking point
        sick_count++;
      }
      else if (sick_count % 6 == 4)
      { Serial1.write('x');//pole one speed (10.8)
        SICKR(1030, 2500, 10, 40, 20, 150, 2000, 500, 800); //Right pole one shooting point
        sick_count++;
      }
      else if (sick_count % 6 == 5)
      {
        SICKR(535, 5510, 10, 40, 20, 150, 5000,  500, 3500); //centre point away fence
        sick_count++;
      }

    }

    if (PS4.isTouching(0) && PS4.getButtonClick(TOUCHPAD))
    {
      touch_x = PS4.getX(0);
      touch_y = PS4.getY(0);
      if (touch_x < 1000 && touch_y < 500) //(SPEED DECREMENT)
        Serial1.write('m');
      else if (touch_x > 1000 && touch_y < 500) //(SPEED INCREMENT)
        Serial1.write('n');
      else if (touch_x < 1000 && touch_y > 500) //NOT ASSIGNED
        Serial1.write('z');
      else if (touch_x > 1000 && touch_y > 500) //Sick right point throwing point
      { Serial1.write('u');//right point type two speed (20.2)
        right_arena = true;
      }
      //SICKL(950, 5715, 8, 40, 20, 70, 2000, 0, 500); //Center

      else
        Serial1.write('z');
    }
    if (PS4.getButtonClick(PS))
      PS4.disconnect();
  }
}

void MPU()
{
  digitalWrite(26, HIGH);
  if (Serial2.available() >= 2)
  {
    m = Serial2.read();
    l = Serial2.read();
    digitalWrite(26, LOW);
    curr_angle = m << 8 | l ;
    if (curr_angle > 180)
      curr_angle = curr_angle - 360;
    Serial.println(String ("Angle---------------------- - ") + curr_angle);
  }
}

void movement(int v, int v_w, float angle, float KP, float KD)
{
  //    KP = 0; KD = 0; v = 0; v_w = 0;

  if (v > 150)
    v = 150;
  //Serial.println(String ("TAN ANGLE ") + angle);
  MPU();

  v_x = v * cos(angle);
  v_y = v * sin(angle);

  error = curr_angle - targated_angle;
  if (error < -180)
    error = error + 360;
  else if (error > 180)
    error = error - 360;
  diff = error - prev_error;
  correction = (error * KP) + (diff * KD);
  prev_error = error;

  v2 = (v_x - v_y) - v_w - correction;
  v1 = (v_x + v_y) + v_w + correction;//
  v3 = (v_y - v_x) - v_w - correction;
  v4 = (-v_x - v_y) + v_w + correction;//

  if (v1 >= 0)
  {
    analogWrite(RPWM_1, v1);
    analogWrite(LPWM_1, 0);
  }
  else
  {
    analogWrite(RPWM_1, 0);
    analogWrite(LPWM_1, abs(v1));
  }

  if (v2 >= 0)
  {
    analogWrite(RPWM_2, v2);
    analogWrite(LPWM_2, 0);
  }
  else
  {
    analogWrite(RPWM_2, 0);
    analogWrite(LPWM_2, abs(v2));
  }

  if (v3 >= 0)
  {
    analogWrite(RPWM_3, v3);
    analogWrite(LPWM_3, 0);
  }
  else
  {
    analogWrite(RPWM_3, 0);
    analogWrite(LPWM_3, abs(v3));
  }

  if (v4 >= 0)
  {
    analogWrite(RPWM_4, v4);
    analogWrite(LPWM_4, 0);
  }
  else
  {
    analogWrite(RPWM_4, 0);
    analogWrite(LPWM_4, abs(v4));
  }
}

void SICKR(int targated_v, int targated_h, int tolerable_error, int min_PWMs, int min_PWM, int max_PWM, unsigned long int stopping_time, int inc_dist, int dec_dist)
{
  rotate(0);
  KP = 2.8;
  KD = 0.8;


  curr_time = 0;
  curr_time = millis();

  vertical = ads.readADC_SingleEnded(1);
  sick_right = ads.readADC_SingleEnded(2);
  vertical = map(vertical, 80, 26720, 5 , 6070); //done
  sick_right = map(sick_right, 50  , 26700, 5, 6070); //done

  horizontal = sick_right;
  error_v = targated_v - vertical;
  error_h = horizontal - targated_h;

  error_sick = sqrt((error_h * error_h) + (error_v * error_v));

  prev_error_sick = 0;
  long temp_error_sick = error_sick;

  while (error_sick > tolerable_error && abs(millis() - curr_time) <= stopping_time)
  {
    Usb.Task();
    if (PS4.getButtonClick(CROSS))
    {
      movement(0, 0, angle, 0, 0);
      break;
    }

    angle = atan2(error_v, error_h);

    if (error_sick >= (temp_error_sick - inc_dist))
    {
      v = map(error_sick, temp_error_sick, (temp_error_sick - inc_dist), min_PWMs, max_PWM);
      movement(v, 0, angle, KP, KD);
    }
    else if (error_sick >= dec_dist)
      movement(max_PWM, 0, angle, KP, KD);
    else
    {
      v = map(error_sick, dec_dist, (tolerable_error + 300), max_PWM, min_PWM);
      movement(v, 0, angle, KP, KD);
    }

    vertical = ads.readADC_SingleEnded(1);
    sick_right = ads.readADC_SingleEnded(2);
    vertical = map(vertical, 80, 26720, 5 , 6070); //done
    sick_right = map(sick_right, 50  , 26700, 5, 6070); //done
    horizontal = sick_right;
    Serial.println(String("Vertical : ") + vertical);
    Serial.println(String("right     : ") + sick_right);

    error_v = targated_v - vertical;
    error_h = horizontal - targated_h;
    error_sick = sqrt(error_h * error_h + error_v * error_v);
  }


  rotate(0);

  movement(0, 0, angle, KP, KD);
}

void SICKL(int targated_v, int targated_h, int tolerable_error, int min_PWMs, int min_PWM, int max_PWM, unsigned long int stopping_time, int inc_dist, int dec_dist)
{
  rotate(0);
  KP = 2.8;
  KD = 0.8;


  curr_time = 0;
  curr_time = millis();

  vertical = ads.readADC_SingleEnded(1);
  sick_left = ads.readADC_SingleEnded(3);
  vertical = map(vertical, 80, 26720, 5 , 6070); //done
  sick_left = map(sick_left, 330, 26660, 5, 6070); //done

  horizontal = sick_left;
  error_v = targated_v - vertical;
  error_h = targated_h - horizontal;

  error_sick = sqrt((error_h * error_h) + (error_v * error_v));

  prev_error_sick = 0;
  long temp_error_sick = error_sick;

  while (error_sick > tolerable_error && abs(millis() - curr_time) <= stopping_time)
  {
    Usb.Task();
    if (PS4.getButtonClick(CROSS))
    {
      movement(0, 0, angle, 0, 0);
      break;
    }

    angle = atan2(error_v, error_h);

    if (error_sick >= (temp_error_sick - inc_dist))
    {
      v = map(error_sick, temp_error_sick, (temp_error_sick - inc_dist), min_PWMs, max_PWM);
      movement(v, 0, angle, KP, KD);
    }
    else if (error_sick >= dec_dist)
      movement(max_PWM, 0, angle, KP, KD);
    else
    {
      v = map(error_sick, dec_dist, (tolerable_error + 300), max_PWM, min_PWM);
      movement(v, 0, angle, KP, KD);
    }

    vertical = ads.readADC_SingleEnded(1);
    sick_left = ads.readADC_SingleEnded(3);
    vertical = map(vertical, 80, 26720, 5 , 6070); //done
    sick_left = map(sick_left, 330, 26660, 5, 6070); //done
    horizontal = sick_left;
    Serial.println(String("Vertical : ") + vertical);
    Serial.println(String("Left     : ") + sick_right);

    error_v = targated_v - vertical;
    error_h = targated_h - horizontal;
    error_sick = sqrt(error_h * error_h + error_v * error_v);
  }
  if (sick_count % 4 == 1)
    rotate(0);

  movement(0, 0, angle, KP, KD);
}

void rotate(int ANGLE)
{
  start_rot_time = millis();
  KP = 1.5;
  KD = 0.25;
  targated_angle = ANGLE;
  while (curr_angle >= (targated_angle + 3) || curr_angle <= (targated_angle - 3))
  {
    curr_rot_time = millis();
    if (curr_rot_time - start_rot_time > rotation_time)
    {
      curr_angle = targated_angle;
      curr_rot_time = start_rot_time = 0;
    }
    movement(0, 0, angle, KP, KD);

  }
}

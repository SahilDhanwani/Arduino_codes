#include <PS4BT.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;

BTD Btd(&Usb);

PS4BT PS4(&Btd);

int RPWM_1 = 23;
int RPWM_2 = 24;
int RPWM_3 = 25;
int RPWM_4 = 26;
int LPWM_1 = 27;
int LPWM_2 = 28;
int LPWM_3 = 29;
int LPWM_4 = 30;

int REN_1 = 31;
int REN_2 = 32;
int REN_3 = 33;
int REN_4 = 34;
int LEN_1 = 35;
int LEN_2 = 36;
int LEN_3 = 37;
int LEN_4 = 38;

int x,y,w;
int a = 100;
float angle;
bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value; 

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
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

  pinMode(REN_1, OUTPUT);
  pinMode(REN_2, OUTPUT);
  pinMode(REN_3, OUTPUT);
  pinMode(REN_4, OUTPUT);
  pinMode(LEN_1, OUTPUT);
  pinMode(LEN_2, OUTPUT);
  pinMode(LEN_3, OUTPUT);
  pinMode(LEN_4, OUTPUT);

  
}
void loop() {
  Usb.Task();

  if (PS4.connected()) {
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117 || PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)) {
      Serial.print(F("\r\nLeftHatX: "));
      Serial.print(PS4.getAnalogHat(LeftHatX));
      Serial.print(F("\tLeftHatY: "));
      Serial.print(PS4.getAnalogHat(LeftHatY));

      Serial.print(F("\tRightHatX: "));
      Serial.print(PS4.getAnalogHat(RightHatX));
      Serial.print(F("\tRightHatY: "));
      Serial.print(PS4.getAnalogHat(RightHatY));

        x = map(PS4.getAnalogHat(LeftHatX),0,255,-127,128);
        y = map(PS4.getAnalogHat(LeftHatY),255,0,-127,128);

      Serial.print(F("\r\nX: "));
      Serial.print(x);
      Serial.print(F("\t\nY: "));
      Serial.print(y);
        if(y>=0)
        {
          angle = atan2(y,x);
          Serial.print("angle : ");
          Serial.println(angle);
        }
        else
        {
          angle = atan2(y,x) + 360;
          Serial.print("angle : ");
          Serial.println(angle);
        }

        

        
    }

//    if (PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)) { // These are the only analog buttons on the PS4 controller
//      Serial.print(F("\r\nL2: "));
//      Serial.print(PS4.getAnalogButton(L2));
//      Serial.print(F("\tR2: "));
//      Serial.print(PS4.getAnalogButton(R2));
//    }
//    if (PS4.getAnalogButton(L2) != oldL2Value || PS4.getAnalogButton(R2) != oldR2Value) // Only write value if it's different
//      PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
//    oldL2Value = PS4.getAnalogButton(L2);
//    oldR2Value = PS4.getAnalogButton(R2);
//
//    if (PS4.getButtonClick(PS)) {
//      Serial.print(F("\r\nPS"));
//      PS4.disconnect();
//    }
//    else {
//      if (PS4.getButtonClick(TRIANGLE)) {
//        Serial.print(F("\r\nTriangle"));
//        PS4.setRumbleOn(RumbleLow);
//      }
//      if (PS4.getButtonClick(CIRCLE)) {
//        Serial.print(F("\r\nCircle"));
//        PS4.setRumbleOn(RumbleHigh);
//      }
//      if (PS4.getButtonClick(CROSS)) {
//        Serial.print(F("\r\nCross"));
//        PS4.setLedFlash(10, 10); // Set it to blink rapidly
//      }
//      if (PS4.getButtonClick(SQUARE)) {
//        Serial.print(F("\r\nSquare"));
//        PS4.setLedFlash(0, 0); // Turn off blinking
//      }
//
//      if (PS4.getButtonClick(UP)) {
//        Serial.print(F("\r\nUp"));
//        PS4.setLed(Red);
//      } if (PS4.getButtonClick(RIGHT)) {
//        Serial.print(F("\r\nRight"));
//        PS4.setLed(Blue);
//      } if (PS4.getButtonClick(DOWN)) {
//        Serial.print(F("\r\nDown"));
//        PS4.setLed(Yellow);
//      } if (PS4.getButtonClick(LEFT)) {
//        Serial.print(F("\r\nLeft"));
//        PS4.setLed(Green);
//      }
//
//      if (PS4.getButtonClick(L1))
//        Serial.print(F("\r\nL1"));
//      if (PS4.getButtonClick(L3))
//        Serial.print(F("\r\nL3"));
//      if (PS4.getButtonClick(R1))
//        Serial.print(F("\r\nR1"));
//      if (PS4.getButtonClick(R3))
//        Serial.print(F("\r\nR3"));
//
//      if (PS4.getButtonClick(SHARE))
//        Serial.print(F("\r\nShare"));
//      if (PS4.getButtonClick(OPTIONS)) {
//        Serial.print(F("\r\nOptions"));
//        printAngle = !printAngle;
//      }
//      if (PS4.getButtonClick(TOUCHPAD)) {
//        Serial.print(F("\r\nTouchpad"));
//        printTouch = !printTouch;
//      }
//
//      if (printAngle) { // Print angle calculated using the accelerometer only
//        Serial.print(F("\r\nPitch: "));
//        Serial.print(PS4.getAngle(Pitch));
//        Serial.print(F("\tRoll: "));
//        Serial.print(PS4.getAngle(Roll));
//      }
//
//      if (printTouch) { // Print the x, y coordinates of the touchpad
//        if (PS4.isTouching(0) || PS4.isTouching(1)) // Print newline and carriage return if any of the fingers are touching the touchpad
//          Serial.print(F("\r\n"));
//        for (uint8_t i = 0; i < 2; i++) { // The touchpad track two fingers
//          if (PS4.isTouching(i)) { // Print the position of the finger if it is touching the touchpad
//            Serial.print(F("X")); Serial.print(i + 1); Serial.print(F(": "));
//            Serial.print(PS4.getX(i));
//            Serial.print(F("\tY")); Serial.print(i + 1); Serial.print(F(": "));
//            Serial.print(PS4.getY(i));
//            Serial.print(F("\t"));
//          }
//        }
//      }
//    }
  }
}

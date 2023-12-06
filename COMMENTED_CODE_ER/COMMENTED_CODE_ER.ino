// ANGULAR motion 

//    int i = 0;
//    Serial.println(String("angle : ") + angl);
//    Serial.println(String("error : ") + error);
//Serial.println(String("pwm : ") + p);
//    if (error > 3)
//    {
//      p = (error * KP) + (KD * (error - prev_error)) + (KI * (error + prev_error));
//      Serial.println(String("pwm : ") + p);
//
//      if (error >= 30 )
//      {
//        analogWrite(R_PWM_gp, p);
//        analogWrite(L_PWM_gp, 0);
//      }
//      else if (error >= 15)
//      {
//        i = 30 - error; p = p + i;
//        analogWrite(R_PWM_gp, p);
//        analogWrite(L_PWM_gp, 0);
//      }
//      else if (error > 3)
//      {
//        if (error >= 5) {
//          i = 15 - error;
//          p = p + i;
//        }
//        analogWrite(R_PWM_gp, p);
//        analogWrite(L_PWM_gp, 0);
//      }
//    }//el-if for 90degree
//
//    else if (error < 0)
//    {
//      p = error * KPd + KD * (error - prev_error) + KI * (error + prev_error);
//      p = abs(p);
//
//      if (error <= -30)
//      {
//        analogWrite(R_PWM_gp, 0);
//        analogWrite(L_PWM_gp, p);
//      }
//      else if (error <= -15)
//      {
//        i = 30 + error; p = p + i;
//        analogWrite(R_PWM_gp, 0);
//        analogWrite(L_PWM_gp, p);
//      }
//      else if (error <= 0)
//      {
//        if (error <= -5) {
//          i = 15 + error;
//          p = p + i;
//        }
//        analogWrite(R_PWM_gp, 0);
//        analogWrite(L_PWM_gp, p);
//      }

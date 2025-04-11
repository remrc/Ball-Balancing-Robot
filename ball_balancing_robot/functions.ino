void beep(int btime) {
  digitalWrite(BUZZER, HIGH);
  delay(btime);
  digitalWrite(BUZZER, LOW);
  delay(70);
}

void XY_to_threeWay(float speed_X, float speed_Y) {

  int16_t m1 = round(0.5 * speed_X + 0.866 * speed_Y);
  int16_t m2 = round(0.5 * speed_X - 0.866 * speed_Y);
  int16_t m3 = -speed_X;  
  
  Motor1_control(constrain(m1, -255, 255));
  Motor2_control(constrain(m2, -255, 255));
  Motor3_control(constrain(m3, -255, 255));
}

void threeWay_to_XY(int m1, int m2, int m3) {
  float d = (m1 + m2 + m3) / 3.0;
  speed_Y_enc = (m1 - m2) / 1.732;
  speed_X_enc = -(d - m3);
}

float doublePID_X(float measured_speed, float measured_angle) {

  float error_speed = -measured_speed;
  integral_PI_X += error_speed;
  
  if (integral_PI_X > integralSpeedMax)
    integral_PI_X = integralSpeedMax;
  else if (integral_PI_X < -integralSpeedMax)
    integral_PI_X = -integralSpeedMax;

  float desired_angle = Kp_PI * error_speed + Ki_PI * integral_PI_X;
  
  float error_angle = desired_angle - measured_angle;
  float derivative_angle = error_angle - prev_error_angle_X;
  float output = Kp_PD * error_angle + Kd_PD * derivative_angle;
  prev_error_angle_X = error_angle;
  
  output += -Kp_pos * pos_X;
  
  output = constrain(output, -255, 255);
  return output;
}

float doublePID_Y(float measured_speed, float measured_angle) {
  
  float error_speed = -measured_speed;
  integral_PI_Y += error_speed;
  
  if (integral_PI_Y > integralSpeedMax)
    integral_PI_Y = integralSpeedMax;
  else if (integral_PI_Y < -integralSpeedMax)
    integral_PI_Y = -integralSpeedMax;

  float desired_angle = Kp_PI * error_speed + Ki_PI * integral_PI_Y;
  
  float error_angle = desired_angle - measured_angle;
  float derivative_angle = error_angle - prev_error_angle_Y;
  float output = Kp_PD * error_angle + Kd_PD * derivative_angle;
  prev_error_angle_Y = error_angle;
  
  output += -Kp_pos * pos_Y;
  
  output = constrain(output, -255, 255);
  return output;
}

void battVoltage(double voltage) {
  //SerialBT.print("batt: "); SerialBT.println(voltage); //debug
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void Motor1_control(int sp) {
  if (sp < 0) 
    digitalWrite(DIR1, LOW);
  else 
    digitalWrite(DIR1, HIGH);
  pwmSet(PWM1_CH, 255 - abs(sp));
}

void Motor2_control(int sp) {
  if (sp < 0) 
    digitalWrite(DIR2, LOW);
  else 
    digitalWrite(DIR2, HIGH);
  pwmSet(PWM2_CH, 255 - abs(sp));
}

void Motor3_control(int sp) {
  if (sp < 0) 
    digitalWrite(DIR3, LOW);
  else 
    digitalWrite(DIR3, HIGH);
  pwmSet(PWM3_CH, 255 - abs(sp));
}

void ENC1_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}

void ENC2_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}

void ENC3_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}

int Tuning() {
  if (!SerialBT.available())  return 0;
  char param = SerialBT.read();               // get parameter byte
  if (!SerialBT.available()) return 0;
  char cmd = SerialBT.read();                 // get command byte
  switch (param) {
    case 'p':
      if (cmd == '+')    Kp_PI += 0.001;
      if (cmd == '-')    Kp_PI -= 0.001;
      printGValues();
      break;
    case 'i':
      if (cmd == '+')    Ki_PI += 0.001;
      if (cmd == '-')    Ki_PI -= 0.001;
      printGValues();
      break;  
    case 's':
      if (cmd == '+')    Kp_PD += 0.5;
      if (cmd == '-')    Kp_PD -= 0.5;
      printGValues();
      break;  
    case 'd':
      if (cmd == '+')    Kd_PD += 1;
      if (cmd == '-')    Kd_PD -= 1;
      printGValues();
      break; 
    case 'k':
      if (cmd == '+')    Kp_pos += 0.01;
      if (cmd == '-')    Kp_pos -= 0.01;
      printGValues();
      break;  

   case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        SerialBT.println("calibrating on");
      }
      if (cmd == '-' && calibrating)  {
        if (abs(robot_angle_X) < 10 && abs(robot_angle_Y) < 10) {
          SerialBT.print("X: "); SerialBT.print(robot_angle_X); SerialBT.print(" Y: "); SerialBT.println(robot_angle_Y); 
          offsets.ID = 95;
          offsets.X_angle = robot_angle_X;
          offsets.Y_angle = robot_angle_Y;
          EEPROM.put(0, offsets);
          EEPROM.commit();
          EEPROM.get(0, offsets);
          beep(70);
          beep(70);
          calibrating = false;
          calibrated = true;
        } else {
          SerialBT.println("The angles are wrong!!!");
          beep(70);
          beep(500);
        }
        SerialBT.println("calibrating off");
      }
      break;              
   }
   return 1;
}

void printGValues() {
  SerialBT.print("P_I: "); SerialBT.print(Kp_PI,3);
  SerialBT.print(" I_I: "); SerialBT.print(Ki_PI,3);
  SerialBT.print(" P_D: "); SerialBT.print(Kp_PD);
  SerialBT.print(" D_D: "); SerialBT.println(Kd_PD);
  SerialBT.print("Pos: "); SerialBT.println(Kp_pos,3);
}

#include "ESP32.h"
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include "MPU6050_6Axis_MotionApps612.h"

BluetoothSerial SerialBT;

MPU6050     mpu;
Quaternion  q;           // quaternion container
VectorFloat gravity;     // gravity vector

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-ball"); 
  
  EEPROM.begin(EEPROM_SIZE);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  
  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);

  EEPROM.get(0, offsets);
  if (offsets.ID == 95) calibrated = true;
    else calibrated = false;

  beep(70);

  Wire.begin();
  Wire.setClock(400000);
  
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    //mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    //Serial.println(); mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus); Serial.println(F(")"));
    
    beep(60);
    beep(1000);
    while(1);
  }
  
  beep(70);
  beep(70);
}

void loop() {

  currentT = millis();

  if (currentT - previousT_1 >= loop_time) {
    Tuning();  // derinimui

    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
     }

    robot_angle_Y = -ypr[1] * 180 / M_PI;
    robot_angle_Y_cor = robot_angle_Y - offsets.Y_angle;
    robot_angle_X = -ypr[2] * 180 / M_PI;
    robot_angle_X_cor = robot_angle_X - offsets.X_angle;

    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    speed_X_filtered = speed_filter * speed_X_enc + (1 - speed_filter) * speed_X_filtered;
    speed_Y_filtered = speed_filter * speed_Y_enc + (1 - speed_filter) * speed_Y_filtered;
    
    pos_X += speed_X_filtered;
    pos_Y += speed_Y_filtered;

    pos_X = constrain(pos_X, -MAX_POS, MAX_POS);
    pos_Y = constrain(pos_Y, -MAX_POS, MAX_POS);

    if (abs(robot_angle_X_cor) > 15 || abs(robot_angle_Y_cor) > 15) 
      vertical = false;
    if (abs(robot_angle_X_cor) < 0.6 && abs(robot_angle_Y_cor) < 0.6) 
      vertical = true;

    if (vertical && calibrated && !calibrating) {    
      int pwm_X = doublePID_X(speed_X_filtered, robot_angle_X_cor);
      int pwm_Y = doublePID_Y(speed_Y_filtered, robot_angle_Y_cor);
      XY_to_threeWay(pwm_X, -pwm_Y);
    } else {
      XY_to_threeWay(0, 0);
      pos_X = 0;
      pos_Y = 0;
    }
    previousT_1 = currentT;
  }
  
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / batt_div); 
    if (!calibrated && !calibrating) {
      SerialBT.println("first you need to calibrate the balancing point...");
      Serial.println("first you need to calibrate the balancing point...");
    }
    previousT_2 = currentT;
  }
  
}

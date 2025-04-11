#define BUZZER      27
#define VBAT        34
#define INT_LED     2

#define BRAKE       26

#define DIR1        15
#define PWM1        25
#define PWM1_CH     0
#define ENC1_1      13
#define ENC1_2      14

#define DIR2        4
#define PWM2        32
#define PWM2_CH     1
#define ENC2_1      35
#define ENC2_2      33

#define DIR3        5
#define PWM3        18
#define PWM3_CH     2
#define ENC3_1      16
#define ENC3_2      17

#define TIMER_BIT  8
#define BASE_FREQ  20000

#define MAX_POS     60

#define EEPROM_SIZE 64

uint8_t   devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t  packetSize;       // expected DMP packet size (default is 42 bytes)
uint8_t   fifoBuffer[64];   // FIFO storage buffer
float     ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

int loop_time = 5;
int batt_div = 299;

float Kp_PI = 0.006;
float Ki_PI = 0.008;
float Kp_PD = 10.0;
float Kd_PD = 20.0;
float Kp_pos = 0.25;

const float integralSpeedMax = 200.0;

float integral_PI_X = 0.0, integral_PI_Y = 0.0;;
float prev_error_angle_X = 0.0, prev_error_angle_Y = 0.0;

struct AngleOffsetsObj {
  int ID;
  int16_t X_angle;
  int16_t Y_angle;
};
AngleOffsetsObj offsets;

float robot_angle_X, robot_angle_Y, robot_angle_X_cor, robot_angle_Y_cor;
   
float pos_X, pos_Y;
float speed_X_enc, speed_Y_enc; 
float speed_X_filtered, speed_Y_filtered; 

float speed_filter = 0.5;

long currentT, previousT_1, previousT_2 = 0, previousT_3 = 0;

volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int16_t motorY_speed, motorX_speed;
int32_t motorY_pos, motorX_pos;

int motor1_counter = 0;
int16_t motor1_speed;         
int motor2_counter = 0;
int16_t motor2_speed;         
int motor3_counter = 0;
int16_t motor3_speed;   
  

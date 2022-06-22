// 라이브러리 호출구간 시작

#include <I2Cdev.h>
#include <math.h>
#include <MPU9250_9Axis_MotionApps41.h>

#include "pid.h"
#include "flyControler.h"
#include "commonLib.h"
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

//////////////////////// Gyro data 불러오기 위한 부분 시작 ////////////////////////
#include "MPU9250.h"
MPU9250 accelgyro;
I2Cdev   I2C_M;
void getGyro_Data();
//////////////////////// Gyro data 불러오기 위한 부분 끝 ////////////////////////


//////////////////////// timeDiff 함수 관련 변수 선언 부분 시작 ////////////////////////
float timeDiff = 0.f;
float tv;
float last_tv = 0;
//////////////////////// timeDiff 함수 관련 변수 선언 부분 끝 ////////////////////////


//////////////////////// 라디오 채널관련 전역변수 및 함수형식 선언구간 시작 //////////////////////

#define fromLow 900
#define fromHigh 1900
#define toLow 1000
#define toHigh 2000

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int channel_1, channel_2, channel_3, channel_4;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;

//////////////////////// 라디오 채널관련 전역변수 및 함수형식 선언구간 끝 ////////////////////////


//////////////////////// MPU9250 전역변수 및 함수형식 선언구간 시작 ////////////////////////

MPU9250 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  

// MPU control/status vars
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;          
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;   
float euler[3];        
float ypr[3];
float Gxyz[3];           

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void MPU9250_INIT();
void MPU9250_Stabilized();

bool stable = false;

float ypr0_prev = 0;
float ypr0_sum = 0;

float ref_roll = 0;
float ref_pitch = 0;
float ref_yaw = 0;

//////////////////////// MPU9250 전역변수 및 함수형식 선언구간 끝 ////////////////////////


//////////////////////// PID 관련 전역변수 선언구간 시작 ////////////////////////

PID_STRUCT rollAttitudePidSettings_main;
PID_STRUCT pitchAttitudePidSettings_main;
PID_STRUCT rollRatePidSettings_main;
PID_STRUCT pitchRatePidSettings_main;
PID_STRUCT yawRatePidSettings_main;

float rollAttitudeOutput;
float pitchAttitudeOutput;

float rollRateOutput;
float pitchRateOutput;
float yawRateOutput;

//////////////////////// PID 관련 전역변수 선언구간 끝 ////////////////////////


//////////////////////// 모터 관련 전역변수 선언구간 시작 ////////////////////////
#define CW1 3
#define CCW1 4
#define CW2 5
#define CCW2 6

Servo cw1;
Servo ccw1;
Servo cw2;
Servo ccw2;

float throttleOutput;

int cw1_out;
int ccw1_out;
int cw2_out;
int ccw2_out;
float divider;
//////////////////////// 모터 관련 전역변수 선언구간 시작 ////////////////////////


void setup() {
    MPU9250_INIT();
    Serial.println("Done MPU9250 initializing");
    
    MPU9250_Stabilized();
    Serial.println("Done MPU9250 stabilizing");

    pidInit();
    Serial.println("Done PID initializing");
    
    //flyControlerInit();
    Serial.println("Done flyControler initializing");

    cw1.attach(CW1);
    ccw1.attach(CCW1);
    cw2.attach(CW2);
    ccw2.attach(CCW2);
    Serial.println("Done Motor attach");
    
    PCICR |= (1 << PCIE2); //Set PCIE0 to enable PCMSK2 scan.
    PCMSK2 |= (1 << PCINT16); //Set PCINT16 (Analog input A8) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT17); //Set PCINT17 (Analog input A9) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT18); //Set PCINT18 (Analog input A10) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT19); //Set PCINT19 (Analog input A11) to trigger an interrupt on state change.
    Serial.println("Done Radio initializing");

    pidTune(&rollAttitudePidSettings_main, 1.2, 0.02, 0.02, 0.0, 0.0, 1000.0, 4.0);
    pidTune(&pitchAttitudePidSettings_main, 1.2, 0.02, 0.02, 0.0, 0.0, 1000.0, 4.0);
    pidTune(&rollRatePidSettings_main, 3.6, 0.01, 0.07, 0.0, 0.0, 400.0, 2.0);
    pidTune(&pitchRatePidSettings_main, 3.6, 0.01, 0.07, 0.0, 0.0, 400.0, 2.0);
    pidTune(&yawRatePidSettings_main, 2.0, 0.01, 0.08, 0.0, 0.0, 400.0, 2.0);
    Serial.println("Done PID initializing");

    MotorInit();
    Serial.println("Initialized...");
}

void loop() {
  channel_1 = map(receiver_input_channel_1, fromLow, fromHigh, toLow, toHigh);
  channel_2 = map(receiver_input_channel_2, fromLow, fromHigh, toLow, toHigh);
  channel_3 = map(receiver_input_channel_3, fromLow, fromHigh, toLow, toHigh);
  channel_4 = map(receiver_input_channel_4, fromLow, fromHigh, toLow, toHigh);

  if (channel_1 <= 1100 & channel_2 <= 1100 & channel_3 <= 1100 & channel_4 >= 2000)
  {
    MPU9250_Stabilized();
    while(channel_1 < 1900)
    {
      cw1.writeMicroseconds(0);
      ccw1.writeMicroseconds(0);
      cw2.writeMicroseconds(0);
      ccw2.writeMicroseconds(0);
      delay(100);

      channel_1 = map(receiver_input_channel_1, fromLow, fromHigh, toLow, toHigh);
      channel_2 = map(receiver_input_channel_2, fromLow, fromHigh, toLow, toHigh);
      channel_3 = map(receiver_input_channel_3, fromLow, fromHigh, toLow, toHigh);
      channel_4 = map(receiver_input_channel_4, fromLow, fromHigh, toLow, toHigh);
    }
  }
  
  setPidSp(&rollAttitudePidSettings_main, ((float)channel_1 - 1596)/25.0);
  setPidSp(&pitchAttitudePidSettings_main, ((float)channel_2 - 1596)/25.0);

  get_timeDiff();
  get_Angle_MPU_9250();
  getGyro_Data();
  getAttitudePidOutput(&rollAttitudeOutput, &pitchAttitudeOutput, &rollAttitudePidSettings_main, &pitchAttitudePidSettings_main, (ypr[2] - ref_roll)*180/M_PI, (ypr[1] - ref_pitch) * 180/M_PI, timeDiff);
  
  Serial.println(timeDiff);
  Serial.print("set point = ");
  Serial.print(pitchAttitudePidSettings_main.sp);
  Serial.print('\t');
  Serial.print("ref = ");
  Serial.print(ref_pitch*180/M_PI);
  Serial.print('\t');
  Serial.print("pitch = ");
  Serial.print(ypr[1]*180/M_PI);
  Serial.print('\t');
  Serial.print("pv = ");
  Serial.println((ypr[1] - ref_pitch)*180/M_PI);


  Serial.print("err = ");
  Serial.print(pitchAttitudePidSettings_main.err);
  Serial.print('\t');
  Serial.print("set point = ");
  Serial.print(pitchAttitudePidSettings_main.sp);
  Serial.print("\tspShift = ");
  Serial.print(pitchAttitudePidSettings_main.spShift);
  Serial.print('\t');
  Serial.print("pv = ");
  Serial.println(pitchAttitudePidSettings_main.pv);
  
  Serial.print("roll att = ");
  Serial.print(rollAttitudeOutput);
  Serial.print('\t');
  Serial.print("pitch att = ");
  Serial.println(pitchAttitudeOutput);

  setPidSp(&rollRatePidSettings_main, rollAttitudeOutput);
  setPidSp(&pitchRatePidSettings_main, pitchAttitudeOutput);
  setPidSp(&yawRatePidSettings_main, ((float)channel_4 - 1596)/3.0);

  Serial.print("gyro_roll = ");
  Serial.print(Gxyz[0]);
  Serial.print('\t');
  Serial.print("gyro_pitch = ");
  Serial.print(Gxyz[1]);
  Serial.print('\t');
  Serial.print("gyro_yaw = ");
  Serial.println(Gxyz[2]);

  getRatePidOutput(&rollRateOutput, &pitchRateOutput, &yawRateOutput, &rollRatePidSettings_main, &pitchRatePidSettings_main, &yawRatePidSettings_main, Gxyz[0], Gxyz[1], Gxyz[2], timeDiff);

  Serial.print("err = ");
  Serial.print(pitchRatePidSettings_main.err);
  Serial.print('\t');
  Serial.print("set point = ");
  Serial.print(pitchRatePidSettings_main.sp);
  Serial.print("\tspShift = ");
  Serial.print(pitchRatePidSettings_main.spShift);
  Serial.print('\t');
  Serial.print("pv = ");
  Serial.println(pitchRatePidSettings_main.pv);
  
  Serial.print("roll rate = ");
  Serial.print(rollRateOutput);
  Serial.print('\t');
  Serial.print("pitch rate = ");
  Serial.print(pitchRateOutput);
  Serial.print('\t');
  Serial.print("yaw rate = ");
  Serial.println(yawRateOutput);

  throttleOutput = (float)channel_3 - 1100.0;
  MotorControl();
  Serial.print("cw1 = ");
  Serial.print(cw1_out);
  Serial.print('\t');
  Serial.print("ccw1 = ");
  Serial.print(ccw1_out);
  Serial.print('\t');
  Serial.print("cw2 = ");
  Serial.print(cw2_out);
  Serial.print('\t');
  Serial.print("ccw2 = ");
  Serial.println(ccw2_out);

  Serial.print("cw1 = ");
  Serial.print(constrain(cw1_out + 1050, 1050, 2154));
  Serial.print('\t');
  Serial.print("ccw1 = ");
  Serial.print(constrain(ccw1_out + 1050, 1050, 2159));
  Serial.print('\t');
  Serial.print("cw2 = ");
  Serial.print(constrain(cw2_out + 1050, 1050, 2152));
  Serial.print('\t');
  Serial.print("ccw2 = ");
  Serial.println(constrain(ccw2_out + 1050, 1050, 2152));
  
  Serial.println("");
  delay(1);
}

void get_timeDiff()
{
  tv = micros();

  if (tv != last_tv)
  {
    timeDiff = (tv - last_tv)/1000.0;
    last_tv = tv;
  }
}

ISR(PCINT2_vect)
{
  current_time = micros();
  //Channel 1=========================================
  if(PINK & B00000001)
  {                                        //Is input 8 high?
  if(last_channel_1 == 0)
  {                                   //Input 8 changed from 0 to 1
    last_channel_1 = 1;                                      //Remember current input state
    timer_1 = current_time;                                  //Set timer_1 to current_time
  }
  }
  else if(last_channel_1 == 1)
  {                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINK & B00000010 )
  {                                       //Is input 9 high?
    if(last_channel_2 == 0)
    {                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1)
  {                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINK & B00000100 )
  {                                       //Is input 10 high?
    if(last_channel_3 == 0)
    {                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1)
  {                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINK & B00001000 )
  {                                       //Is input 11 high?
    if(last_channel_4 == 0)
    {                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1)
  {                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}

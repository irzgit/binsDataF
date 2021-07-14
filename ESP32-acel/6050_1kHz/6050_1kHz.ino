#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "BluetoothSerial.h"

#define FK 0.5 // коэффициент комплементарного фильтра
#define statKoeff 0.04

BluetoothSerial SerialBT;

float q0, q1, q2, q3;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float axf, ayf, azf;
float gxf, gyf, gzf;
float angleX = 0;
float angleY = 0;
float angleZ = 0;
float xVel=0, yVel=0, zVel=0;           //Скорости
float xVelOld=0, yVelOld=0, zVelOld=0;  //Скорости в предыдущий момент времени
float xPos=0, yPos=0, zPos=0;           //Координаты


static float filValX = 0, filValY=0, filValZ=0;


const float toDeg = 180.0 / M_PI;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

MPU6050 mpu;

int samplePeriod = 0;

void initDMP(); 
void getAngles();
void expRunningAverage();
void setup() 
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  while(!Serial) {}
  if (!SerialBT.begin("ESP32")) Serial.println("An error occurred initializing Bluetooth");
  else Serial.println("Bluetooth initialized");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  initDMP();
  delay(50);
  Serial.println("Calibration...");
  mpu.CalibrateAccel(10);
  mpu.CalibrateGyro(10);
  fifoCount = mpu.getFIFOCount();
}

int count = 0;
//аккумуляторы
float q0_a=0, q1_a=0, q2_a=0, q3_a=0;
float ax_a=0, ay_a=0, az_a=0;
float gx_a=0, gy_a=0, gz_a=0;

long int start = millis();
void loop() 
{
  unsigned long int st = micros();

  getAngles();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//переводим в g

axf = ax / 16384.0f;
ayf = ay / 16384.0f;
azf = az / 16384.0f;

if(abs(axf)<0.01) axf=0;
if(abs(ayf)<0.01) ayf=0;
if(abs(azf)<0.01) azf=0;

gxf=gx/32768.0f * 250;
gyf=gy/32768.0f * 250;
gzf=gz/32768.0f * 250;

if(abs(gxf)<0.01) gxf=0;
if(abs(gyf)<0.01) gyf=0;
if(abs(gzf)<0.01) gzf=0;

//Удаляем гравитацию

axf-=gravity.x;
ayf-=gravity.y;
azf-=gravity.z;

if(abs(axf)<0.01) axf=0;
if(abs(ayf)<0.01) ayf=0;
if(abs(azf)<0.01) azf=0;

if(count==10)
{
  q0_a/=10; q1_a/=10; q2_a/=10; q3_a/=10;
  ax_a/=10; ay_a/=10; az_a/=10;
  gx_a/=10; gy_a/=10; gz_a/=10;
  char buf[100];
  int len=sprintf(buf,"%c%+02.2f,%+02.2f,%+02.2f,%+02.2f,%+04.2f,%+04.2f,%+04.2f,%+03.2f,%+03.2f,%+03.2f%c",'#',q0_a,q1_a,q2_a,q3_a,gx_a,gy_a,gz_a,ax_a,ay_a,az_a,'#');
    //  String str='+'+String(q0_a)+','+String(q1_a)+','+String(q2_a)+','+String(q3_a)+','+String(gx_a)+','+String(gy_a)+','+String(gz_a)+','+String(ax_a)+','+String(ay_a)+','+String(az_a)+'+';

      Serial.println(buf);
      SerialBT.println(buf);
 q0_a=0; q1_a=0; q2_a=0; q3_a=0;
 ax_a=0; ay_a=0; az_a=0;
 gx_a=0; gy_a=0; gz_a=0;
 count=0;
}
else
{
  count++;
  q0_a+=q.w; q1_a+=q.x; q2_a+=q.y; q3_a+=q.z;
  ax_a+=axf; ay_a+=ayf; az_a+=azf;
  gx_a+=gxf; gy_a+=gyf; gz_a+=gzf;    
}
     
}



void initDMP() 
{
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
}
void getAngles() 
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleX = ypr[2] * toDeg;
    angleY = ypr[1] * toDeg;
    angleZ = ypr[0] * toDeg;
  }
}

void expRunningAverage() 
{
    filValX+=(axf-filValX)*FK;
    filValY+=(ayf-filValY)*FK;
    filValZ+=(azf-filValZ)*FK;
}

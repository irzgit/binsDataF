#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "BluetoothSerial.h"
#include "GyverFilters.h"

#define statKoeff 0.04
#define LED 25

BluetoothSerial SerialBT;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float axf, ayf, azf;
float gxf, gyf, gzf;
float angleX = 0;
float angleY = 0;
float angleZ = 0;



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


uint8_t filter_select=0; // 0-без фильра, 1-среднее, 2-медиана, 3-калман

void sort(float *mas, int size);
void initDMP(); 
void getAngles();
float mediana(float mas[10]);       //медианный
float average(float mas[10]);  //среднее (принимает аккумулятор и колличесвто аккумулированныйх измерений, после аккумулятор обнуляется)
int comp (const void * a, const void * b)
{
  return ( *(float*)a - *(float*)b );
}
void setup() 
{
  pinMode(LED,OUTPUT);
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
  //digitalWrite(LED,HIGH);
}

GKalman gx_k(40,40,0.5), gy_k(40,40,0.5), gz_k(40,40,0.5), ax_k(40,40,0.5), ay_k(40,40,0.5), az_k(40,40,0.5); 

int count = 0;
//аккумуляторы
float q0_a=0, q1_a=0, q2_a=0, q3_a=0;
float ax_a=0, ay_a=0, az_a=0;
float gx_a=0, gy_a=0, gz_a=0;
//накопители для медианного фильтра
float q0_m[10], q1_m[10], q2_m[10], q3_m[10];
float ax_m[10], ay_m[10], az_m[10];
float gx_m[10], gy_m[10], gz_m[10];

//окончательные данные, которые пойдут навыход
float q0_o=0, q1_o=0, q2_o=0, q3_o=0;
float ax_o=0, ay_o=0, az_o=0;
float gx_o=0, gy_o=0, gz_o=0;

float i_angx=0, i_angy=0, i_angz=0;
float i_velx=0, i_vely=0, i_velz=0;
float i_posx=0, i_posy=0, i_posz=0;
float samplePeriod = 0.01;

void loop() 
{
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
  if(filter_select==0 || filter_select>3)
  {
      q0_o=q0_m[9]; q1_o=q1_m[9]; q2_o=q2_m[9]; q3_o=q3_m[9];
      ax_o=ax_m[9]; ay_o=ay_m[9]; az_o=az_m[9];
      gx_o=gx_m[9]; gy_o=gy_m[9]; gz_o=gz_m[9];
      digitalWrite(LED,HIGH);
  }
  else if(filter_select==1)
  {
      q0_o=average(q0_m); q1_o=average(q1_m); q2_o=average(q2_m); q3_o=average(q3_m);
      ax_o=average(ax_m); ay_o=average(ay_m); az_o=average(az_m);
      gx_o=average(gx_m); gy_o=average(gy_m); gz_o=average(gz_m);
      digitalWrite(LED,LOW);
  }
  else if(filter_select==2)
  {
      q0_o=mediana(q0_m); q1_o=mediana(q1_m); q2_o=mediana(q2_m); q3_o=mediana(q3_m);
      ax_o=mediana(ax_m); ay_o=mediana(ay_m); az_o=mediana(az_m);
      gx_o=mediana(gx_m); gy_o=mediana(gy_m); gz_o=mediana(gz_m);
      digitalWrite(LED,HIGH);
  }
  else if(filter_select==3)
  {
       q0_o=q0_m[9]; q1_o=q1_m[9]; q2_o=q2_m[9]; q3_o=q3_m[9];
       ax_o=ax_k.filtered(ax_m[9]); ay_o=ay_k.filtered(ay_m[9]); az_o=az_k.filtered(az_m[9]);
       gx_o=gx_k.filtered(gx_m[9]); gy_o=gy_k.filtered(gy_m[9]); gz_o=gz_k.filtered(gz_m[9]);
       digitalWrite(LED,LOW);
  }

  //интегрирование

  
  //get velocity
  i_velx = i_velx + ax_o * samplePeriod;
  i_vely = i_vely + ay_o * samplePeriod;
  i_velz = i_velz + az_o * samplePeriod;
  //get position
  i_posx = i_posx + i_velx * samplePeriod;
  i_posy = i_posy + i_vely * samplePeriod;
  i_posz = i_posz + i_velz * samplePeriod;
  //get angles
  i_angx = i_angx + (gx_o * samplePeriod);
  i_angy = i_angy + (gy_o * samplePeriod);
  i_angz = i_angz + (gz_o * samplePeriod);

  gx_o = i_angx;
  gy_o = i_angy;
  gz_o = i_angz;
  ax_o = i_posx;
  ay_o = i_posy;
  az_o = i_posz;

  
  
  char buf[100];
  int len=sprintf(buf,"%c%+02.2f,%+02.2f,%+02.2f,%+02.2f,%+04.2f,%+04.2f,%+04.2f,%+03.2f,%+03.2f,%+03.2f%c",'#',q0_o,q1_o,q2_o,q3_o,gx_o,gy_o,gz_o,ax_o,ay_o,az_o,'#');
  Serial.println(buf);
  SerialBT.println(buf);
  count=0;
  if(Serial.available()>0)
  {
    filter_select=Serial.read();
  }
}
else
{
  q0_m[count]=q.w; q1_m[count]=q.x; q2_m[count]=q.y; q3_m[count]=q.z;
  ax_m[count]=axf; ay_m[count]=ayf; az_m[count]=azf;
  gx_m[count]=gzf; gy_m[count]=gyf; gz_m[count]=gzf;
  count++;    
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

float average(float mas[10])
{
  float av =0.0;
  for(int i=0; i<10; i++) av+=mas[i];
  av/=10.0;
  return av;
}
float mediana(float mas[10])
{
  float med;
  sort(mas,10);
  return (mas[4]+mas[5])/2;
}

void sort(float *mas, int size) 
{
  qsort (mas, size, sizeof(float), comp);
}

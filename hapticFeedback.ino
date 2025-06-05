#include <Wire.h>

float rateRoll, ratePitch, rateYaw;
float rateRollCali, ratePitchCali, rateYawCali;

const int rateCaliIterations = 2000;
const int accCaliIterations = 2000;


float accX, accY, accZ;
float accX_Cali {0} 
float accY_Cali {0} 
float accZ_Cali {0};
float angleRoll, anglePitch, angleYaw;

float loopTimer;

void acc_signals(void){
  Wire.beginTransmission(0x68); //start I2C communication with gyro

  //Low pass filter to filter out motor vibration noise
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  //Configure the accelerometer output
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //Read from acceleromter, sensitivity to 4096 LSB/g
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //register of reading starting point
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t accX_LSB = Wire.read() <<8 | Wire.read();
  int16_t accY_LSB = Wire.read() <<8 | Wire.read();
  int16_t accZ_LSB = Wire.read() <<8 | Wire.read();

  accX = (float)accX_LSB/4096;
  accY = (float)accY_LSB/4096;
  accZ = (float)accZ_LSB/4096;

  //angleRoll = atan(accY/sqrt(accX*accX + accZ*accZ))*180/3.142;
  //anglePitch = atan(-accX/sqrt(accY*accY + accZ*accZ))*180/3.142;
  //anglePitch1 = atan(accZ/sqrt(accX*accX + accY*accY))*180/3.142;
}

void acc_cali(iterations){
  for(int i = 0; i<iterations; ++i){
    acc_signals();
    accX_Cali += accX;
    accY_Cali += accY;
    accZ_Cali += accZ;
  }
  accX_Cali /=iterations;
  accY_Cali /=iterations;
  accZ_Cali /=iterations;
}



//Read gyro signals
void gyro_signals(void){
  Wire.beginTransmission(0x68); //start I2C communication with gyro

  //Low pass filter to filter out motor vibration noise
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  //Set the sensitivity scale factor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);  //choose 65.5LSB/degree/s: reading can be as granular as 65.5 counts per degree/second
  Wire.endTransmission();

  //Access registers storing gyro measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //first register to use for gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  //one reading per axis, two registers per reading, in total 6 requesting registers

  //two registers(in LSB) per reading, store in one 2-byte var
  int16_t gyroX = Wire.read()<<8 | Wire.read(); 
  int16_t gyroY = Wire.read()<<8 | Wire.read(); 
  int16_t gyroZ = Wire.read()<<8 | Wire.read(); 

  //retrieve degree from LSB readings
  rateRoll = (float)gyroX/65.5; 
  ratePitch = (float)gyroY/65.5; 
  rateYaw = (float)gyroZ/65.5; 
}

void setup(void) {
  Serial.begin(57600);
  
  //Set the clock speed of I2C
  Wire.setClock(400000);  //MPU6050 supports I2C communications at up to 400kHz
  Wire.begin();
  delay(250); //give MPU6050 time to start

  //Start the gyro in power mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0X00);
  Wire.endTransmission();

  /*
  for(int i = 0; i < rateCaliIterations; ++i){
    gyro_signals();
    rateRollCali += rateRoll;
    ratePitchCali += ratePitch;
    rateYawCali += rateYaw;
    delay(1);
  }
  rateRollCali /= rateCaliIterations;
  ratePitchCali /= rateCaliIterations;
  rateYawCali /= rateCaliIterations;
  */

  for(int i = 0; i < accCaliIterations; ++i){
    acc_signals
  }
}

void loop() {
  acc_signals();
  /*
  rateRoll -= rateRollCali;
  ratePitch -= ratePitchCali;
  rateYaw -= rateYawCali;
  //print results in degree/s
  Serial.print("Roll rate [degree/s] = ");
  Serial.print(rateRoll);
  Serial.print("Pitch rate [degree/s] = ");
  Serial.print(ratePitch);
  Serial.print("Yaw rate [degree/s] = ");
  Serial.print(rateYaw);
  Serial.println();
  */

  Serial.print(" Acceleration X [g] = ");
  Serial.print(accX);

  Serial.print(" Acceleration Y [g] = ");
  Serial.print(accY);

  Serial.print(" Acceleration Z [g] = ");
  Serial.print(accZ);
  Serial.println();
  delay(50);
}
#include <Wire.h>

float rateRoll, ratePitch, rateYaw;
float rateRollCali, ratePitchCali, rateYawCali;

const int rateCaliIterations = 2000;
const int accCaliIterations = 2000;


float accX, accY, accZ;
float accX_Cali, accY_Cali, accZ_Cali;
float angleRoll, anglePitch,anglePitch1, angleYaw;

float kalmanAngleRoll = 0, 
      kalmanUncertaintyAngleRoll = 2*2;

float kalmanAnglePitch = 0, 
      kalmanUncertaintyAnglePitch = 2*2;

float kalman1DOutput[] = {0,0}; //{angle prediction, uncertainty}

void kalman_1d(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement){
  kalmanState += 0.004*kalmanInput;
  kalmanUncertainty += 0.004 * 0.004 * 4 * 4;
  float kalmanGain = kalmanUncertainty * 1/(1*kalmanUncertainty + 3*3);
  kalmanState += kalmanGain*(kalmanMeasurement-kalmanState);
  kalmanUncertainty = (1-kalmanGain) * kalmanUncertainty;
  kalman1DOutput[0] = kalmanState;
  kalman1DOutput[1] = kalmanUncertainty;

}


uint32_t loopTimer;

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
}

void gyro_cali(int iterations){
  for(int i = 0; i < iterations; ++i){
    gyro_signals();
    rateRollCali += rateRoll;
    ratePitchCali += ratePitch;
    rateYawCali += rateYaw;
    delay(1);
  }
  rateRollCali /= rateCaliIterations;
  ratePitchCali /= rateCaliIterations;
  rateYawCali /= rateCaliIterations;
}

void acc_readings(float accX_Cali, float accY_Cali,float accZ_Cali ){
  acc_signals();
  accX-=accX_Cali;
  accY-=accY_Cali;
  accZ-=accZ_Cali;
}

void get_angles(void){
  angleRoll = atan(accY/sqrt(accX*accX + accZ*accZ))*180/3.142;
  anglePitch = atan(-accX/sqrt(accY*accY + accZ*accZ))*180/3.142;
  anglePitch1 = atan(accZ/sqrt(accX*accX + accY*accY))*180/3.142;
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
  gyro_cali(rateCaliIterations);

  loopTimer = micros();
}

void loop() {
  gyro_signals();

  
  rateRoll -= rateRollCali;
  ratePitch -= ratePitchCali;
  rateYaw -= rateYawCali;

  acc_readings(0.12, -0.03, 0.01);
  get_angles();

  kalman_1d(kalmanAngleRoll, kalmanUncertaintyAngleRoll, rateRoll, angleRoll);
  kalmanAngleRoll = kalman1DOutput[0];
  kalmanUncertaintyAngleRoll - kalman1DOutput[1];
  kalman_1d(kalmanAnglePitch, kalmanUncertaintyAnglePitch, ratePitch, anglePitch);
  kalmanAnglePitch = kalman1DOutput[0];
  kalmanUncertaintyAnglePitch - kalman1DOutput[1];




  //print results in degree/s
  Serial.print("Roll angle [degree] = ");
  Serial.print(kalmanAngleRoll);
  Serial.print("Pitch angle [degree] = ");
  Serial.print(kalmanAnglePitch);

  Serial.println();
  while(micros() - loopTimer < 4000);
  loopTimer = micros();
  
  /*
  Serial.print(" Roll angle[degree] = ");
  Serial.print(angleRoll);

  Serial.print(" Pitch angle[degree] = ");
  Serial.print(anglePitch);

  Serial.print(" Pitch1 angle[degree] = ");
  Serial.print(anglePitch1);

  Serial.println();
  */
  //delay(50);
}
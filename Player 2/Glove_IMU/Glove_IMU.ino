#include <Wire.h>

//const int flex1 = A0;
const int flex2 = A1;
//const int flex3 = A2;
//const int flex4 = A3;
int flex1val, flex2val, flex3val, flex4val;

const int MPU = 0X68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

void setup() 
{
  Serial.begin(115200);

  //Serial.println("Initialize MPU6050");

  Wire.begin(); //Initialise communication
  Wire.beginTransmission(MPU); //Start comm. with MPU6050
  Wire.write(0x6B); //Talk to register 6B => MPU = 0x68
  Wire.write(0x00); //Make reset - place 0 into 6B register
  Wire.endTransmission(true); //end transmission
  
  //Configure Acc Sensitivity - Full Scale Range(default +/- 8g full scale):
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00); //Set the register bits as 00000000 (+-250 deg/s full scale) 
  Wire.endTransmission(true);
  delay(20);

  //Configure Gyroscope sensitvity - full scale range:
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x08); //Set the register bits as 00001000 (+- 4g full scale)
  Wire.endTransmission(true);
  delay(20);

  //DLPF(Digital Low-pass filter):
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x06); //Set the register bits as 00000110 (5Hz bandwidth, 0.19ms delay)
  Wire.endTransmission(true);
  delay(20);
  
  //Call this function to get IMU errors for module:
  //calculate_IMU_error();
  delay(20);
}

//following function is called in setup function to calc the accel and gyro readings
//IMU should be mounted flat, to get proper values
//read accelerometer values 200 times
void calculate_IMU_error()
{
  while(c<200){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    //Sum all readings:
    AccErrorX = AccErrorX + ((atan(AccY) / sqrt(pow((AccX),2) + pow((AccZ),2))) * 180 / PI);
    AccErrorY = AccErrorY + ((atan(-1 * AccX) / sqrt(pow((AccY),2) + pow((AccZ),2))) * 180 / PI);
    
    c++;
  }

  //Divide the sum by 200 to get the average error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;

  c=0;
  while(c<200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    //Sum all readings:
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);

    c++;
  }
  //Divide the sum by 200 to get the average error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  //Print the error values on the Serial Monitor:
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

void loop()
{ 
  
  //Read flex sensor values:
  //flex1val = analogRead(flex1);
  flex2val = analogRead(flex2);
  //flex3val = analogRead(flex3);
  //flex4val = analogRead(flex4);
  //print values on serial monitor:
  //Serial.println(flex1val);
  Serial.println(flex2val);
  //Serial.println(flex3val);
  //Serial.println(flex4val);
  delay(1000);
  
  //Read accelerometer data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Starts with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true); //read 6 registers in total, each axis value is stored 
  //for a range of +/- 2g, value of 16384 is to be used
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; //unit: g
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; //divide by 16384 LSB/g
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  
  //Calculating Roll and Pitch from Accel data:
  accAngleX = (atan(AccY / sqrt((pow(AccX,2)) + pow(AccZ,2))) * 180 / PI) - 0.57; //Roll - AccError
  accAngleY = (atan(-1*AccX / sqrt(pow(AccY,2) + pow(AccZ,2))) * 180 / PI) + 2.90; //Pitch - AccError
  //extra step for converting g to ms^-2:
  AccX = AccX * 9.806; //unit: ms^-2
  AccY = AccY * 9.806;
  AccZ = AccZ * 9.806;
  
  //Read Gyroscope data:
  previousTime = currentTime; //previous time is stored before the actual time 
  currentTime = millis(); 
  elapsedTime = (currentTime - previousTime) / 1000; //divide by 1000 to get elasped time in s
  Wire.beginTransmission(MPU); //start reading data
  Wire.write(0x43); //starts from register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true); //read 6 registers total, each axis value is stored in each register
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; //unit: deg/s
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0; //divide by 131 LSB/deg/s
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  //Correct the outputs with the calculated error values:
  GyroX = GyroX - 0.52; //GyroErrorX (~0.56)
  GyroY = GyroY + 0.09; //GyroErrorY(~2)
  GyroZ = GyroZ - 0.02; //GyroErrorZ(~0.8)
  //Currently, the raw values are in deg/s, so need to multiply the time elasped to obtain the angle:
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; //deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw = yaw + GyroZ* elapsedTime;

  //Complimentary filter to account for accumulated error (drift) of gyroscope values over time - combine accelerometer and gyro angle values
  roll = 0.96  * gyroAngleX + 0.04 * accAngleX; //since data from gyroscope suffers from drift over time, 4% of accelerometer value to eliminate the gyroscope drift error
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  //print values on serial monitor:
  Serial.print(AccX);
  Serial.print(",");
  Serial.print(AccY);
  Serial.print(",");
  Serial.println(AccZ);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);
  
}

#pragma once
#include <SPI.h>
#include <arduino.h>
#include "LSM6DSL_Consts.h"

class SPIComm {
public:
  SPIComm(uint32_t bandwidth, uint8_t slavePin1, uint8_t slavePin2)
  : settings(bandwidth, MSBFIRST, SPI_MODE0), slavePins{slavePin1,slavePin2} {}
  void init() {
    pinMode(slavePins[0], OUTPUT);
    pinMode(slavePins[1], OUTPUT);
    SPI.begin();
  }
  uint8_t readReg(uint8_t id, uint8_t address) {
    SPI.beginTransaction(settings);
    digitalWrite(slavePins[id], LOW);
    SPI.transfer(address | READ_FLAG);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(slavePins[id], HIGH);
    SPI.endTransaction();
    return value;
  }
  void writeReg(uint8_t id, uint8_t address, uint8_t data) {
    SPI.beginTransaction(settings);
    digitalWrite(slavePins[id], LOW);
    SPI.transfer(address);
    SPI.transfer(data);
    digitalWrite(slavePins[id], HIGH);
    SPI.endTransaction();  
  }
private:
  const int READ_FLAG = 0x80;
  SPISettings settings;
  uint8_t slavePins[2];
};


//----------------------------------------------------------------------------------------------------------------
// Class LSM6DLS
//----------------------------------------------------------------------------------------------------------------
class LSM6DLS
{
public:
  LSM6DLS(uint32_t bandwidth, uint8_t slavePinIMU, uint8_t slavePinMAG, float compFilterConst, float GGain, float dt)
  : spi(bandwidth, slavePinIMU, slavePinMAG), compFilterConst(compFilterConst), GGain(GGain), dt(dt) {}
  void spiInit() { spi.init(); }
  bool detectIMU() {
    //Serial.printf("%d %d\n", spi.readReg(1, LIS3MDL_WHO_AM_I), LIS3MDL_ADDRESS);
    return spi.readReg(0, LSM6DSL_WHO_AM_I) == LSM6DSL_ADDRESS;
  }
  void initIMU() {
    // Initialise the accelerometer
    spi.writeReg(0, LSM6DSL_CTRL1_XL, 0x9F);        // ODR 3.33 kHz, +/- 8g , BW = 400hz
    spi.writeReg(0, LSM6DSL_CTRL8_XL, 0xC8);        // Low pass filter enabled, BW9, composite filter
    spi.writeReg(0, LSM6DSL_CTRL3_C, 0x44);         // Enable Block Data update, increment during multi byte read

    // Initialise the gyroscope
    spi.writeReg(0, LSM6DSL_CTRL2_G, 0x9C);         // ODR 3.3 kHz, 2000 dps

    // Initialize magnetometer 
    spi.writeReg(1, LIS3MDL_CTRL_REG1, 0xDC);      // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
    spi.writeReg(1, LIS3MDL_CTRL_REG2, 0x20);      // +/- 8 gauss
    spi.writeReg(1, LIS3MDL_CTRL_REG3, 0x00);      // Continuous-conversion mode
    
    Serial.printf("InitIMU done\n");
  }
  void readValues() {
    readACC();
    readGYR();
    readMAG();
  }
  void readACC(){
    acc[0] = read(0, LSM6DSL_OUTX_L_XL, LSM6DSL_OUTX_H_XL);
    acc[1] = read(0, LSM6DSL_OUTY_L_XL, LSM6DSL_OUTY_H_XL);
    acc[2] = read(0, LSM6DSL_OUTZ_L_XL, LSM6DSL_OUTZ_H_XL);
  }
  void readGYR(){
    gyr[0] = read(0, LSM6DSL_OUTX_L_G, LSM6DSL_OUTX_H_G);
    gyr[1] = read(0, LSM6DSL_OUTY_L_G, LSM6DSL_OUTY_H_G);
    gyr[2] = read(0, LSM6DSL_OUTZ_L_G, LSM6DSL_OUTZ_H_G);
  }
  void readMAG(){
    mag[0] = read(1, LIS3MDL_OUT_X_L, LIS3MDL_OUT_X_H);
    mag[1] = read(1, LIS3MDL_OUT_Y_L, LIS3MDL_OUT_Y_H);
    mag[2] = read(1, LIS3MDL_OUT_Z_L, LIS3MDL_OUT_Z_H);
  }
  void getAccAngles() {
    readValues();
    //Convert Gyro raw to degrees per second
    float rate_gyr_x = (float) gyr[0] * GGain;
    float rate_gyr_y = (float) gyr[1] * GGain;
    float rate_gyr_z = (float) gyr[2] * GGain;
  
    //Calculate the angles from the gyro
    gyroXangle += rate_gyr_x * dt;
    gyroYangle += rate_gyr_y * dt;
    gyroZangle += rate_gyr_z * dt;
  
    //Convert Accelerometer values to degrees
    float AccXangle = (atan2(acc[1], acc[2]) + M_PI) * RAD_TO_DEG;
    float AccYangle = (atan2(acc[2], acc[0]) + M_PI) * RAD_TO_DEG;
  
    //If IMU is up the correct way, use these lines
    AccXangle -= (float)180.0;
    if (AccYangle > 90)
      AccYangle -= (float)270;
    else
      AccYangle += (float)90;
  
    //Complementary filter used to combine the accelerometer and gyro values.
    CFangleX = compFilterConst * (CFangleX + rate_gyr_x * dt) + (1 - compFilterConst) * AccXangle;
    CFangleY = compFilterConst * (CFangleY + rate_gyr_y * dt) + (1 - compFilterConst) * AccYangle;
    
    heading = atan2(mag[1], mag[0]) * RAD_TO_DEG;

      Serial.print("#AccX\t");
  Serial.print(acc[0]);
  Serial.print("\t###  AccY  ");
  Serial.print(acc[1]);
    Serial.print("\t###  AccY  ");
  Serial.print(acc[2]);
  
  Serial.print("  ###  GyrX\t");
  Serial.print(gyroXangle);
  Serial.print("  ###  GyrY  \t");
  Serial.print(gyroYangle);
  Serial.print("   ###  GyrZ\t");
  Serial.print(gyroZangle);
  Serial.print("     ######    CFangleX\t");
  
  }
  
  void resetZero(uint32_t warmup, uint32_t n) {
    Serial.printf("Reset zero angles...\n");
    float X0 = 0;
    float Y0 = 0;
    for (uint32_t i = 0; i < warmup; ++i) {
      getAccAngles();
      delay(dt * 1000);
    }
    for (uint32_t i = 0; i < n; ++i) {
      getAccAngles();
      X0 += CFangleX;
      Y0 += CFangleY;
      delay(dt * 1000);
    }
    CFangleX0 = X0 / n;
    CFangleY0 = Y0 / n;
    Serial.printf("X0 = %f Y0 = %f\n", CFangleX0, CFangleY0);
  }
  
  void print() {
    Serial.printf("CFangle = [%f, %f] Heading = %f\n", CFangleX - CFangleX0, CFangleY - CFangleY0, heading);
  }
protected:
  int16_t read(uint8_t id, int addr_lo, int addr_hi){
    uint8_t lo = spi.readReg(id, addr_lo);
    uint8_t hi = spi.readReg(id, addr_hi);
    return  int16_t(lo | hi << 8);
  }
private:
  SPIComm spi;
  int16_t acc[3] = { 0 };
  int16_t gyr[4] = { 0 };
  int16_t mag[3] = { 0 };

  float gyroXangle = 0;
  float gyroYangle = 0;
  float gyroZangle = 0; 

  float compFilterConst = 0;
  float GGain = 0;
  float dt = 0;
 
  float CFangleX = 0;
  float CFangleY = 0;
  float CFangleX0 = 0;
  float CFangleY0 = 0;
  float heading = 0;
};

#include "LSM6DSL.h"

const int ledPin = LED_BUILTIN;
LSM6DLS imu(10000000, 10, 15, 0.97, 0.070, 0.02);

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.printf("SPI sample starting...\n");
  imu.spiInit();
  if (imu.detectIMU())
    Serial.printf("BerryIMUv3 Found\n");
  else
    Serial.printf("BerryIMUv3 Not Found\n");
  Serial.printf("Initializing IMU...\n");
  imu.initIMU();
  delay(1000);
  imu.resetZero(100, 100);
}

int i = 0;

void loop() {  
  imu.getAccAngles();
  if (i % 50 == 0) {
    imu.print();
    i = 0;
  }
  i++;
  delay(0.02 * 1000);
}

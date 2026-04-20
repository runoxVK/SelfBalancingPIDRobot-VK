#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU-6050 connected.");
  } else {
    Serial.println("Connection failed — check wiring.");
    while (1);
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float angleY = atan2(ax, az) * 180.0 / PI;

  Serial.print("AngleY: ");
  Serial.println(angleY);

  delay(50);
}
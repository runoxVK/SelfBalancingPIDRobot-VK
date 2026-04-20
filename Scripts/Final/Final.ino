#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define AIN1 25
#define AIN2 26
#define BIN1 27
#define BIN2 14

#define PWM_FREQ 20000
#define PWM_RES  8

float Kp = 15.0;
float Ki = 0.0;
float Kd = 1.0;
float targetAngle = 0.0;
float angle = 0.0;
float lastError = 0.0;
float integral = 0.0;

unsigned long lastTime = 0;
float dt = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpu.initialize();

  ledcAttach(AIN1, PWM_FREQ, PWM_RES);
  ledcAttach(AIN2, PWM_FREQ, PWM_RES);
  ledcAttach(BIN1, PWM_FREQ, PWM_RES);
  ledcAttach(BIN2, PWM_FREQ, PWM_RES);

  ledcWrite(AIN1, 0); ledcWrite(AIN2, 0);
  ledcWrite(BIN1, 0); ledcWrite(BIN2, 0);

  Serial.println("Calibrating... hold upright.");
  float sum = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += atan2(ax, az) * 180.0 / PI;
    delay(5);
  }
  targetAngle = sum / 200.0;
  angle = targetAngle;
  lastTime = micros();

  Serial.print("Target: ");
  Serial.println(targetAngle);
  Serial.println("Ready. y=status  p/m=Kp  i/b=Kd  t/v=target");
}

void loop() {
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  if (dt < 0.001) return;
  lastTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelAngle = atan2(ax, az) * 180.0 / PI;
  float gyroRate   = gy / 131.0;
  angle = 0.95 * (angle + gyroRate * dt) + 0.05 * accelAngle;

  float error      = angle - targetAngle;
  integral        += error * dt;
  integral         = constrain(integral, -50, 50);
  float derivative = (error - lastError) / dt;
  lastError        = error;
  float output     = -(Kp * error + Ki * integral + Kd * derivative);
  output           = constrain(output, -255, 255);

  if (abs(output) < 2) {
    ledcWrite(AIN1, 0); ledcWrite(AIN2, 0);
    ledcWrite(BIN1, 0); ledcWrite(BIN2, 0);
  } else {
    int speed = map(abs((int)output), 2, 255, 60, 255);
    speed = constrain(speed, 60, 255);
    if (output > 0) {
      ledcWrite(AIN1, speed); ledcWrite(AIN2, 0);
      ledcWrite(BIN1, speed); ledcWrite(BIN2, 0);
    } else {
      ledcWrite(AIN1, 0); ledcWrite(AIN2, speed);
      ledcWrite(BIN1, 0); ledcWrite(BIN2, speed);
    }
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') { Kp += 1.0;  Serial.print("Kp:"); Serial.println(Kp); }
    if (c == 'm') { Kp -= 1.0;  Serial.print("Kp:"); Serial.println(Kp); }
    if (c == 'i') { Kd += 0.1;  Serial.print("Kd:"); Serial.println(Kd); }
    if (c == 'b') { Kd -= 0.1;  Serial.print("Kd:"); Serial.println(Kd); }
    if (c == 't') { targetAngle += 0.5; Serial.print("SP:"); Serial.println(targetAngle); }
    if (c == 'v') { targetAngle -= 0.5; Serial.print("SP:"); Serial.println(targetAngle); }
    if (c == 'y') {
      Serial.print("Angle:"); Serial.print(angle);
      Serial.print(" Target:"); Serial.print(targetAngle);
      Serial.print(" Kp:"); Serial.print(Kp);
      Serial.print(" Kd:"); Serial.println(Kd);
    }
  }
}
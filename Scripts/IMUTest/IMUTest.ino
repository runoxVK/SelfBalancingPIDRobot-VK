#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define AIN1 25
#define AIN2 26
#define BIN1 27
#define BIN2 14

#define PWM_FREQ 20000
#define PWM_RES  8

#define Kp         40.0
#define Kd         0.05
#define Ki         40.0
#define sampleTime 0.005

// use your calibrated offsets
#define Y_ACCEL_OFFSET  -475
#define Z_ACCEL_OFFSET   3256
#define X_GYRO_OFFSET    58

float targetAngle = -2.5;

int16_t accY, accZ, gyroX;
volatile int motorPower = 0;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0;
volatile float error, errorSum = 0;

hw_timer_t *timer = NULL;
volatile bool doISR = false;

void IRAM_ATTR onTimer() {
  doISR = true;
}

void setMotors(int power) {
  power = constrain(power, -255, 255);

  if (power > 0) {
    ledcWrite(AIN1, power); ledcWrite(AIN2, 0);
    ledcWrite(BIN1, power); ledcWrite(BIN2, 0);
  } else if (power < 0) {
    ledcWrite(AIN1, 0); ledcWrite(AIN2, -power);
    ledcWrite(BIN1, 0); ledcWrite(BIN2, -power);
  } else {
    ledcWrite(AIN1, 0); ledcWrite(AIN2, 0);
    ledcWrite(BIN1, 0); ledcWrite(BIN2, 0);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  mpu.initialize();
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);
  mpu.setXGyroOffset(X_GYRO_OFFSET);

  ledcAttach(AIN1, PWM_FREQ, PWM_RES);
  ledcAttach(AIN2, PWM_FREQ, PWM_RES);
  ledcAttach(BIN1, PWM_FREQ, PWM_RES);
  ledcAttach(BIN2, PWM_FREQ, PWM_RES);

  ledcWrite(AIN1, 0); ledcWrite(AIN2, 0);
  ledcWrite(BIN1, 0); ledcWrite(BIN2, 0);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 5000, true, 0);

  Serial.println("Ready. y=status  t/v=target");
}

void loop() {
  if (doISR) {
    doISR = false;

    accY  = mpu.getAccelerationY();
    accZ  = mpu.getAccelerationZ();
    gyroX = mpu.getRotationX();

    accAngle  = atan2(accY, accZ) * RAD_TO_DEG;
    int gyroRate = map(gyroX, -32768, 32767, -250, 250);
    gyroAngle = (float)gyroRate * sampleTime;
    currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * accAngle;

    error    = currentAngle - targetAngle;
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -300, 300);

    motorPower = Kp * error + Ki * errorSum * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
    motorPower = constrain(motorPower, -255, 255);
    prevAngle = currentAngle;

    setMotors(motorPower);
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') { targetAngle += 0.5; Serial.print("Target:"); Serial.println(targetAngle); }
    if (c == 'v') { targetAngle -= 0.5; Serial.print("Target:"); Serial.println(targetAngle); }
    if (c == 'y') {
      Serial.print("Angle:"); Serial.print(currentAngle);
      Serial.print(" Target:"); Serial.print(targetAngle);
      Serial.print(" Power:"); Serial.println(motorPower);
    }
  }
}
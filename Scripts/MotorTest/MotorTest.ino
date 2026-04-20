#define AIN1 25
#define AIN2 26
#define BIN1 27
#define BIN2 14

#define PWM_FREQ 1000
#define PWM_RES  8

void setup() {
  Serial.begin(115200);

  ledcAttach(AIN1, PWM_FREQ, PWM_RES);
  ledcAttach(AIN2, PWM_FREQ, PWM_RES);
  ledcAttach(BIN1, PWM_FREQ, PWM_RES);
  ledcAttach(BIN2, PWM_FREQ, PWM_RES);

  Serial.println("Motor test starting...");
}

void loop() {
  Serial.println("Motor A — forward");
  ledcWrite(AIN1, 150); ledcWrite(AIN2, 0);
  delay(1500);

  Serial.println("Motor A — stop");
  ledcWrite(AIN1, 0); ledcWrite(AIN2, 0);
  delay(500);

  Serial.println("Motor A — backward");
  ledcWrite(AIN1, 0); ledcWrite(AIN2, 150);
  delay(1500);

  Serial.println("Motor A — stop");
  ledcWrite(AIN1, 0); ledcWrite(AIN2, 0);
  delay(500);

  Serial.println("Motor B — forward");
  ledcWrite(BIN1, 150); ledcWrite(BIN2, 0);
  delay(1500);

  Serial.println("Motor B — stop");
  ledcWrite(BIN1, 0); ledcWrite(BIN2, 0);
  delay(500);

  Serial.println("Motor B — backward");
  ledcWrite(BIN1, 0); ledcWrite(BIN2, 150);
  delay(1500);

  Serial.println("Motor B — stop"); 
  ledcWrite(BIN1, 0); ledcWrite(BIN2, 0);
  delay(500);
}
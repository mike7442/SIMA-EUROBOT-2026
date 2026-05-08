// Пины для управления правым мотором
#define MOTOR_RIGHT_IN1 7
#define MOTOR_RIGHT_PWM 6  // ШИМ
// Пины для управления левым мотором
#define MOTOR_LEFT_IN1 8
#define MOTOR_LEFT_PWM 11  // ШИМ
// Пины для энкодеров
#define ENCODER_LEFT_A    2 // INT0
#define ENCODER_LEFT_B    4
#define ENCODER_RIGHT_A   3 // INT1
#define ENCODER_RIGHT_B   5
// Переменные для хранения позиции энкодеров
volatile long encoderLeftPos = 0;
volatile long encoderRightPos = 0;
// Параметры для одометрии (КАЛИБРИРОВАТЬ!)
#define TICKS_PER_REV 1450      // Количество импульсов на оборот колеса
#define WHEEL_DIAMETER 0.0445    // Диаметр колеса в метрах (44.5 мм)
#define WHEEL_BASE 0.105        // Расстояние между колесами в метрах (105 мм)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

// --- НОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ ИЗМЕРЕНИЯ СКОРОСТИ КОЛЕС ---
const unsigned long SPEED_CALC_INTERVAL_MS = 50; // 50 мс -> ~20 Гц
static unsigned long lastSpeedCalcTime = 0;

static long prev_encoder_left_calc = 0;
static long prev_encoder_right_calc = 0;
static unsigned long prev_time_calc = 0;

float currentLeftWheelSpeed = 0.0;
float currentRightWheelSpeed = 0.0;
// ----------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateEncoderRight, RISING);
  stopMotors();

  prev_encoder_left_calc = encoderLeftPos;
  prev_encoder_right_calc = encoderRightPos;
  prev_time_calc = millis();

  Serial.println("Dual motor control & Wheel Speed Monitor started...");
  Serial.println("Control Format: [motor_id],[speed]");
  Serial.println("motor_id: 0=left, 1=right, 2=both");
  Serial.println("speed: -255 to 255 (negative = backward)");
  Serial.println("--- Wheel Speed Data (Left_m/s, Right_m/s) ---");
  Serial.println("Left_m/s,Right_m/s");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("Invalid format. Use: motor_id,speed");
      return;
    }
    int motorId = input.substring(0, commaIndex).toInt();
    int speed = input.substring(commaIndex + 1).toInt();

    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    switch (motorId) {
      case 0:
        setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, speed);
        break;
      case 1:
        setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, speed);
        break;
      case 2:
        setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, speed);
        setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, speed);
        break;
      default:
        Serial.println("Invalid motor ID. Use 0, 1, or 2.");
        return;
    }
  }

  if (millis() - lastSpeedCalcTime >= SPEED_CALC_INTERVAL_MS) {
    calculateWheelSpeeds();
    printWheelSpeeds();
    lastSpeedCalcTime = millis();
  }
}

void setMotorSpeed(int inPin, int pwmPin, int speed) {
  if (speed > 0) {
    digitalWrite(inPin, HIGH);
    analogWrite(pwmPin, 255 - speed);
  } else if (speed < 0) {
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, abs(speed));
  } else {
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopMotors() {
  setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, 0);
  setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, 0);
}

void updateEncoderLeft() {
  if (digitalRead(ENCODER_LEFT_B) == HIGH) {
    encoderLeftPos++;
  } else {
    encoderLeftPos--;
  }
}

void updateEncoderRight() {
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) {
    encoderRightPos++;
  } else {
    encoderRightPos--;
  }
}

void calculateWheelSpeeds() {
  unsigned long currentTime = millis();
  float deltaTimeSec = (currentTime - prev_time_calc) / 1000.0f;

  long currentLeftPos = encoderLeftPos;
  long currentRightPos = encoderRightPos;

  long deltaTicksLeft = currentLeftPos - prev_encoder_left_calc;
  long deltaTicksRight = currentRightPos - prev_encoder_right_calc;

  currentLeftWheelSpeed = (deltaTicksLeft * WHEEL_CIRCUMFERENCE) / (TICKS_PER_REV * deltaTimeSec);
  currentRightWheelSpeed = (deltaTicksRight * WHEEL_CIRCUMFERENCE) / (TICKS_PER_REV * deltaTimeSec);

  prev_encoder_left_calc = currentLeftPos;
  prev_encoder_right_calc = currentRightPos;
  prev_time_calc = currentTime;
}

void printWheelSpeeds() {
  Serial.print(currentLeftWheelSpeed, 3);
  Serial.print(",");
  Serial.println(currentRightWheelSpeed, 3);
}

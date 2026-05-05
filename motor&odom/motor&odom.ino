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

// Переменные для одометрии
float x = 0.0;           // Позиция по X в метрах
float y = 0.0;           // Позиция по Y в метрах
float theta = 0.0;       // Ориентация в радианах (0 = вдоль оси X)
long prevLeftPos = 0;    // Предыдущее значение левого энкодера
long prevRightPos = 0;   // Предыдущее значение правого энкодера

void setup() {
  Serial.begin(115200);

  // Настройка пинов моторов как выходы
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);

  // Настройка пинов энкодеров как входы с подтяжкой
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // Подключение прерываний по возрастанию (RISING) для энкодеров
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateEncoderRight, RISING);

  // Сброс моторов и одометрии
  stopMotors();
  resetOdometry();

  Serial.println("Dual motor control & Odometry started...");
  Serial.println("Control Format: [motor_id],[speed]");
  Serial.println("motor_id: 0=left, 1=right, 2=both");
  Serial.println("speed: -255 to 255 (negative = backward)");
  Serial.println("--- Odometry Data (X_cm, Y_cm, Theta_deg) ---");
  Serial.println("X_cm,Y_cm,Theta_deg"); // Заголовок для CSV-вывода одометрии
}

void loop() {
  // Обработка Serial для управления моторами
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // убираем лишние пробелы
    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("Invalid format. Use: motor_id,speed");
      return;
    }

    int motorId = input.substring(0, commaIndex).toInt();
    int speed = input.substring(commaIndex + 1).toInt();

    // Ограничим скорость
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    // Выбираем, какой мотор управлять
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

    Serial.print("Motor: ");
    Serial.print(motorId);
    Serial.print(", Speed: ");
    Serial.println(speed);
  }

  // Обновляем одометрию каждые 50 мс
  static unsigned long lastOdometryUpdate = 0;
  if (millis() - lastOdometryUpdate > 50) {
    updateOdometry();
    lastOdometryUpdate = millis();
  }

  // Печатаем ТОЛЬКО координаты и угол каждые 500 мс
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) {
    printOdometry();
    lastPrintTime = millis();
  }
}

// =========================
// ФУНКЦИИ УПРАВЛЕНИЯ МОТОРАМИ
// =========================
void setMotorSpeed(int inPin, int pwmPin, int speed) {
  if (speed > 0) {
    // Вращение вперёд
    digitalWrite(inPin, HIGH);
    analogWrite(pwmPin, 255 - speed);  // ⚠️ Обратная логика (если нужно)
  } else if (speed < 0) {
    // Вращение назад
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, abs(speed)); // подаём модуль скорости
  } else { // speed == 0
    // Остановка
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopMotors() {
  setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, 0);
  setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, 0);
}

// =========================
// ФУНКЦИИ ОДОМЕТРИИ
// =========================
void updateOdometry() {
  // Получаем текущие показания энкодеров
  long currentLeftPos = encoderLeftPos;
  long currentRightPos = encoderRightPos;

  // Вычисляем изменения показаний энкодеров
  long deltaLeft = currentLeftPos - prevLeftPos;
  long deltaRight = currentRightPos - prevRightPos;

  // Сохраняем текущие позиции для следующего обновления
  prevLeftPos = currentLeftPos;
  prevRightPos = currentRightPos;

  // Конвертируем показания энкодеров в пройденное расстояние (метры)
  float distanceLeft = (deltaLeft * WHEEL_CIRCUMFERENCE) / TICKS_PER_REV;
  float distanceRight = (deltaRight * WHEEL_CIRCUMFERENCE) / TICKS_PER_REV;

  // Вычисляем среднее пройденное расстояние
  float deltaDistance = (distanceLeft + distanceRight) / 2.0;

  // Вычисляем изменение угла
  float deltaTheta = (distanceRight - distanceLeft) / WHEEL_BASE;

  // Обновляем ориентацию
  theta += deltaTheta;

  // Нормализуем угол в диапазон [-PI, PI]
  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;

  // Обновляем позицию
  if (abs(deltaTheta) < 0.001) { // Прямолинейное движение (почти)
    x += deltaDistance * cos(theta);
    y += deltaDistance * sin(theta);
  } else { // Движение по дуге
    float radius = deltaDistance / deltaTheta;
    x += radius * (sin(theta + deltaTheta) - sin(theta));
    y += radius * (-cos(theta + deltaTheta) + cos(theta));
  }
}

void resetOdometry() {
  x = 0.0;
  y = 0.0;
  theta = 0.0;
  encoderLeftPos = 0;
  encoderRightPos = 0;
  prevLeftPos = 0;
  prevRightPos = 0;
}

// ЭТО ЕДИНСТВЕННАЯ ФУНКЦИЯ, КОТОРАЯ ЛОГИРУЕТ ДАННЫЕ ОДОМЕТРИИ
void printOdometry() {
  // Выводим ТОЛЬКО координаты и угол в компактном CSV формате
  Serial.print(x * 100, 1);  // X в сантиметрах
  Serial.print(",");
  Serial.print(y * 100, 1);  // Y в сантиметрах
  Serial.print(",");
  Serial.println(theta * 180.0 / PI, 1);  // Угол в градусах
}

// =========================
// ФУНКЦИИ ДЛЯ ЭНКОДЕРОВ
// =========================
void updateEncoderLeft() {
  if (digitalRead(ENCODER_LEFT_B) == HIGH) {
    encoderLeftPos++; // вращение вперёд
  } else {
    encoderLeftPos--; // вращение назад
  }
}

void updateEncoderRight() {
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) {
    encoderRightPos++; // вращение вперёд
  } else {
    encoderRightPos--; // вращение назад
  }
}

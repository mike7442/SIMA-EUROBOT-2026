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
#define WHEEL_DIAMETER 0.0445    // Диаметр колеса в метрах (65 мм)
#define WHEEL_BASE 0.105        // Расстояние между колесами в метрах (180 мм)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

// Переменные для одометрии
float x = 0.0;           // Позиция по X в метрах
float y = 0.0;           // Позиция по Y в метрах
float theta = 0.0;       // Ориентация в радианах (0 = вдоль оси X)
long prevLeftPos = 0;    // Предыдущее значение левого энкодера
long prevRightPos = 0;   // Предыдущее значение правого энкодера

void setup() {
  Serial.begin(115200);
  
  // Настройка пинов как входы с подтяжкой
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  
  // Подключение прерываний по возрастанию (RISING)
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateEncoderRight, RISING);
  
  // Сброс одометрии
  resetOdometry();
  
  // Заголовок для CSV-формата
  Serial.println("X_cm,Y_cm,Theta_deg");
}

void loop() {
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

// ЭТО ЕДИНСТВЕННАЯ ФУНКЦИЯ, КОТОРАЯ ЛОГИРУЕТ ДАННЫЕ
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

#define LOGG 
// ПИНЫ ДЛЯ МОТОРОВ
// =========================
#define MOTOR_LEFT_IN1 8
#define MOTOR_LEFT_PWM 11  // ШИМ
#define MOTOR_RIGHT_IN1 7
#define MOTOR_RIGHT_PWM 6  // ШИМ

// =========================
// ПИНЫ ДЛЯ ЭНКОДЕРОВ
// =========================
#define ENCODER_LEFT_A 2  // INT0
#define ENCODER_LEFT_B 4
#define ENCODER_RIGHT_A 3  // INT1
#define ENCODER_RIGHT_B 5

// =========================
// ПАРАМЕТРЫ ОДОМЕТРИИ (КАЛИБРИРОВАТЬ!)
// =========================
#define TICKS_PER_REV 1450     // Количество импульсов на оборот колеса
#define WHEEL_DIAMETER 0.0445  // Диаметр колеса в метрах (65 мм)
#define WHEEL_BASE 0.095       // Расстояние между колесами в метрах (180 мм)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

// =========================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// =========================
// Одометрия#define LOGG 
// ПИНЫ ДЛЯ МОТОРОВ
// =========================
#define MOTOR_LEFT_IN1 8
#define MOTOR_LEFT_PWM 11  // ШИМ
#define MOTOR_RIGHT_IN1 7
#define MOTOR_RIGHT_PWM 6  // ШИМ

// =========================
// ПИНЫ ДЛЯ ЭНКОДЕРОВ
// =========================
#define ENCODER_LEFT_A 2  // INT0
#define ENCODER_LEFT_B 4
#define ENCODER_RIGHT_A 3  // INT1
#define ENCODER_RIGHT_B 5

// =========================
// ПАРАМЕТРЫ ОДОМЕТРИИ (КАЛИБРИРОВАТЬ!)
// =========================
#define LEFT_TICKS_PER_REV 1360     // Количество импульсов на оборот колеса
#define RIGHT_TICKS_PER_REV 1472    
#define WHEEL_DIAMETER 0.0445  // Диаметр колеса в метрах (65 мм)
#define WHEEL_BASE 0.095       // Расстояние между колесами в метрах (180 мм)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

// =========================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// =========================
// Одометрия
float x = 0.0;          // Позиция по X в метрах
float y = 0.0;          // Позиция по Y в метрах
float theta = 0.0;      // Ориентация в радианах (0 = вдоль оси X)
long prevLeftPos = 0;   // Предыдущее значение левого энкодера
long prevRightPos = 0;  // Предыдущее значение правого энкодера

// Энкодеры
volatile long encoderLeftPos = 0;
volatile long encoderRightPos = 0;

// П-регулятор
float KP_LINEAR = 120.0;           // коэффициент для линейной скорости
float KP_ANGULAR = 60.0;           // коэффициент для угловой скорости
float DISTANCE_THRESHOLD = 0.005;  // 5 см
float ANGLE_THRESHOLD_RAD = 0.05;  // ~3 градуса
bool movingToTarget = true;
uint8_t kff = 30;

// Целевые координаты (задаются в setup)
float targetX = 1.0;     // 1 метр по X
float targetY = 0.0;      // 0.5 метра по Y
float targetTheta = PI;  // 90 градусов в радианах

// =========================
// ФУНКЦИИ УПРАВЛЕНИЯ МОТОРАМИ
// =========================

unsigned long timer_cycle = 0;

const int MIN_SPEED_THRESHOLD = 50;  // Минимальное значение ШИМ, при котором мотор крутится

int applyDeadZoneCompensation(int speed) {
  if (speed == 0) return 0;
  else {
    if (speed > 0) {
      speed += kff;
      return speed;
    } else if (speed < 0) {
      speed -= kff;
      return speed;
    }
  }
}

void setMotorSpeed(int inPin, int pwmPin, int speed) {
  speed = constrain(applyDeadZoneCompensation(speed),-255,255);  // Применяем коррекцию
  if (speed > 0) {
    // Вращение вперёд
    digitalWrite(inPin, HIGH);
    analogWrite(pwmPin, 255 - speed);  // ⚠️ Обратная логика
  } else if (speed < 0) {
    // Вращение назад
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, -speed);  // подаём модуль скорости
  } else {                        // speed == 0
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
  float distanceLeft = (deltaLeft * WHEEL_CIRCUMFERENCE) / LEFT_TICKS_PER_REV;
  float distanceRight = (deltaRight * WHEEL_CIRCUMFERENCE) / RIGHT_TICKS_PER_REV;


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
  if (abs(deltaTheta) < 0.001) {  // Прямолинейное движение (почти)
    x += deltaDistance * cos(theta);
    y += deltaDistance * sin(theta);
  } else {  // Движение по дуге
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
#ifdef LOGG 
void printOdometry() {
  // Выводим ТОЛЬКО координаты и угол в компактном CSV формате
  Serial.print(x * 100, 1);  // X в сантиметрах
  Serial.print(",");
  Serial.print(y * 100, 1);  // Y в сантиметрах
  Serial.print(",");
  Serial.println(theta * 180.0 / PI, 1);  // Угол в градусах
}
#endif
// =========================
// ФУНКЦИИ ДЛЯ ЭНКОДЕРОВ
// =========================
void updateEncoderLeft() {
  if (digitalRead(ENCODER_LEFT_B) == HIGH) {
    encoderLeftPos++;  // вращение вперёд
  } else {
    encoderLeftPos--;  // вращение назад
  }
}

void updateEncoderRight() {
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) {
    encoderRightPos++;  // вращение вперёд
  } else {
    encoderRightPos--;  // вращение назад
  }
}

// =========================
// ФУНКЦИИ П-РЕГУЛЯТОРА
// =========================
void normalizeAngle(float &angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
}

void goToPose() {
  float dx = targetX - x;
  float dy = targetY - y;
  float distance = sqrt(dx * dx + dy * dy);

  // Serial.print(dx);
  // Serial.print("   ");
  // Serial.print(dy);
  // Serial.print("   ");
  // Serial.println(distance);

  if (movingToTarget) {
    // --- Движение к точке ---
    if (distance <= DISTANCE_THRESHOLD) {
      movingToTarget = false;  // Переходим к повороту
      return;
    }

    float targetAngle = atan2(dy, dx);
    float angleError = targetAngle - theta;
    normalizeAngle(angleError);

    // Ограничиваем ошибку угла, чтобы не разворачиваться на 180 градусов при малом отклонении
    if (abs(angleError) > PI / 2) {
      // Если угол > 90 градусов, разворачиваемся на месте
      float angularSpeed = KP_ANGULAR * angleError;
      setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, -constrain(angularSpeed, -255, 255));
      setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, constrain(angularSpeed, -255, 255));
      return;
    }

    // Рассчитываем управляющие воздействия
    float linearSpeed = KP_LINEAR * distance;
    float angularCorrection = KP_ANGULAR * angleError;

    // Ограничиваем максимальную скорость
    linearSpeed = constrain(linearSpeed, -255+2*kff, 255-2*kff);
    angularCorrection = constrain(angularCorrection, -100, 100);  // Ограничиваем коррекцию

    // Устанавливаем скорости моторов (дифференциальное управление)
    float leftSpeed = linearSpeed - angularCorrection;
    float rightSpeed = linearSpeed + angularCorrection;
    // Serial.print(rightSpeed);
    // Serial.print("   ");
    // Serial.println(leftSpeed);
    setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, constrain(leftSpeed, -255, 255));
    setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, constrain(rightSpeed, -255, 255));

  } else {
    // --- Поворот на угол ---
    float angleError = targetTheta - theta;
    normalizeAngle(angleError);

    if (abs(angleError) <= ANGLE_THRESHOLD_RAD) {
      stopMotors();  // Доехали и повернули — останавливаемся
      Serial.println("Goal reached!");
      return;
    }

    float angularSpeed = KP_ANGULAR * angleError;
    angularSpeed = applyDeadZoneCompensation(constrain(angularSpeed, -200, 200));  // Ограничиваем скорость поворота
    
    // Устанавливаем скорости моторов для поворота
    setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, -angularSpeed);
    setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, angularSpeed);
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  // Настройка пинов моторов
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  // Настройка пинов энкодеров
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // Подключение прерываний
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateEncoderRight, RISING);

  // Сброс одометрии
  resetOdometry();

  Serial.println("Robot ready. Starting navigation...");
  Serial.print("Target: X=");
  Serial.print(targetX);
  Serial.print(", Y=");
  Serial.print(targetY);
  Serial.print(", Theta=");
  Serial.println(targetTheta);

  // --- ЗАПУСК П-РЕГУЛЯТОРА ---
  // Этот цикл будет работать до тех пор, пока не достигнем цели
  while (true) {
    if (millis() - timer_cycle >= 10) {
      updateOdometry();
      #ifdef LOGG
      printOdometry();
      #endif
      // Вызываем функцию П-регулятора
      goToPose();

      // Проверяем, достигли ли мы цели
      float dx = targetX - x;
      float dy = targetY - y;
      float distance = sqrt(dx * dx + dy * dy);
      float angleError = targetTheta - theta;
      normalizeAngle(angleError);

      if (distance <= DISTANCE_THRESHOLD && !movingToTarget && abs(angleError) <= ANGLE_THRESHOLD_RAD) {
        stopMotors();
        break;  // Выходим из цикла, когда цель достигнута
      }
      timer_cycle = millis();
    }
  }
}

// =========================
// LOOP
// =========================
void loop() {
  // Пустой loop, так как всё происходит в setup()
}
float x = 0.0;          // Позиция по X в метрах
float y = 0.0;          // Позиция по Y в метрах
float theta = 0.0;      // Ориентация в радианах (0 = вдоль оси X)
long prevLeftPos = 0;   // Предыдущее значение левого энкодера
long prevRightPos = 0;  // Предыдущее значение правого энкодера

// Энкодеры
volatile long encoderLeftPos = 0;
volatile long encoderRightPos = 0;

// П-регулятор
float KP_LINEAR = 200.0;           // коэффициент для линейной скорости
float KP_ANGULAR = 20.0;           // коэффициент для угловой скорости
float DISTANCE_THRESHOLD = 0.005;  // 5 см
float ANGLE_THRESHOLD_RAD = 0.05;  // ~3 градуса
bool movingToTarget = true;
uint8_t kff = 30;

// Целевые координаты (задаются в setup)
float targetX = 0.0;     // 1 метр по X
float targetY = 0.0;      // 0.5 метра по Y
float targetTheta = PI;  // 90 градусов в радианах

// =========================
// ФУНКЦИИ УПРАВЛЕНИЯ МОТОРАМИ
// =========================

unsigned long timer_cycle = 0;

const int MIN_SPEED_THRESHOLD = 50;  // Минимальное значение ШИМ, при котором мотор крутится

int applyDeadZoneCompensation(int speed) {
  if (speed == 0) return 0;
  else {
    if (speed > 0) {
      speed += kff;
      return speed;
    } else if (speed < 0) {
      speed -= kff;
      return speed;
    }
  }
}

void setMotorSpeed(int inPin, int pwmPin, int speed) {
  speed = constrain(applyDeadZoneCompensation(speed),-255,255);  // Применяем коррекцию
  if (speed > 0) {
    // Вращение вперёд
    digitalWrite(inPin, HIGH);
    analogWrite(pwmPin, 255 - speed);  // ⚠️ Обратная логика
  } else if (speed < 0) {
    // Вращение назад
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, -speed);  // подаём модуль скорости
  } else {                        // speed == 0
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
  if (abs(deltaTheta) < 0.001) {  // Прямолинейное движение (почти)
    x += deltaDistance * cos(theta);
    y += deltaDistance * sin(theta);
  } else {  // Движение по дуге
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
#ifdef LOGG 
void printOdometry() {
  // Выводим ТОЛЬКО координаты и угол в компактном CSV формате
  Serial.print(x * 100, 1);  // X в сантиметрах
  Serial.print(",");
  Serial.print(y * 100, 1);  // Y в сантиметрах
  Serial.print(",");
  Serial.println(theta * 180.0 / PI, 1);  // Угол в градусах
}
#endif
// =========================
// ФУНКЦИИ ДЛЯ ЭНКОДЕРОВ
// =========================
void updateEncoderLeft() {
  if (digitalRead(ENCODER_LEFT_B) == HIGH) {
    encoderLeftPos++;  // вращение вперёд
  } else {
    encoderLeftPos--;  // вращение назад
  }
}

void updateEncoderRight() {
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) {
    encoderRightPos++;  // вращение вперёд
  } else {
    encoderRightPos--;  // вращение назад
  }
}

// =========================
// ФУНКЦИИ П-РЕГУЛЯТОРА
// =========================
void normalizeAngle(float &angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
}

void goToPose() {
  float dx = targetX - x;
  float dy = targetY - y;
  float distance = sqrt(dx * dx + dy * dy);

  // Serial.print(dx);
  // Serial.print("   ");
  // Serial.print(dy);
  // Serial.print("   ");
  // Serial.println(distance);

  if (movingToTarget) {
    // --- Движение к точке ---
    if (distance <= DISTANCE_THRESHOLD) {
      movingToTarget = false;  // Переходим к повороту
      return;
    }

    float targetAngle = atan2(dy, dx);
    float angleError = targetAngle - theta;
    normalizeAngle(angleError);

    // Ограничиваем ошибку угла, чтобы не разворачиваться на 180 градусов при малом отклонении
    if (abs(angleError) > PI / 2) {
      // Если угол > 90 градусов, разворачиваемся на месте
      float angularSpeed = KP_ANGULAR * angleError;
      setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, -constrain(angularSpeed, -255, 255));
      setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, constrain(angularSpeed, -255, 255));
      return;
    }

    // Рассчитываем управляющие воздействия
    float linearSpeed = KP_LINEAR * distance;
    float angularCorrection = KP_ANGULAR * angleError;

    // Ограничиваем максимальную скорость
    linearSpeed = constrain(linearSpeed, -255+2*kff, 255-2*kff);
    angularCorrection = constrain(angularCorrection, -100, 100);  // Ограничиваем коррекцию

    // Устанавливаем скорости моторов (дифференциальное управление)
    float leftSpeed = linearSpeed - angularCorrection;
    float rightSpeed = linearSpeed + angularCorrection;
    // Serial.print(rightSpeed);
    // Serial.print("   ");
    // Serial.println(leftSpeed);
    setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, constrain(leftSpeed, -255, 255));
    setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, constrain(rightSpeed, -255, 255));

  } else {
    // --- Поворот на угол ---
    float angleError = targetTheta - theta;
    normalizeAngle(angleError);

    if (abs(angleError) <= ANGLE_THRESHOLD_RAD) {
      stopMotors();  // Доехали и повернули — останавливаемся
      Serial.println("Goal reached!");
      return;
    }

    float angularSpeed = KP_ANGULAR * angleError;
    angularSpeed = applyDeadZoneCompensation(constrain(angularSpeed, -200, 200));  // Ограничиваем скорость поворота
    
    // Устанавливаем скорости моторов для поворота
    setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, -angularSpeed);
    setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, angularSpeed);
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  // Настройка пинов моторов
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  // Настройка пинов энкодеров
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // Подключение прерываний
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateEncoderRight, RISING);

  // Сброс одометрии
  resetOdometry();

  Serial.println("Robot ready. Starting navigation...");
  Serial.print("Target: X=");
  Serial.print(targetX);
  Serial.print(", Y=");
  Serial.print(targetY);
  Serial.print(", Theta=");
  Serial.println(targetTheta);

  // --- ЗАПУСК П-РЕГУЛЯТОРА ---
  // Этот цикл будет работать до тех пор, пока не достигнем цели
  while (true) {
    if (millis() - timer_cycle >= 10) {
      updateOdometry();
      #ifdef LOGG
      printOdometry();
      #endif
      // Вызываем функцию П-регулятора
      goToPose();

      // Проверяем, достигли ли мы цели
      float dx = targetX - x;
      float dy = targetY - y;
      float distance = sqrt(dx * dx + dy * dy);
      float angleError = targetTheta - theta;
      normalizeAngle(angleError);

      if (distance <= DISTANCE_THRESHOLD && !movingToTarget && abs(angleError) <= ANGLE_THRESHOLD_RAD) {
        stopMotors();
        break;  // Выходим из цикла, когда цель достигнута
      }
      timer_cycle = millis();
    }
  }
}

// =========================
// LOOP
// =========================
void loop() {
  // Пустой loop, так как всё происходит в setup()
}

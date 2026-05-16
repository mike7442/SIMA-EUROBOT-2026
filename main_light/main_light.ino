#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <avr/pgmspace.h>

// =========================
// ПИНЫ
// =========================
#define SDA_PIN A4
#define SCL_PIN A5
#define XSHUT_LEFT  A2
#define XSHUT_RIGHT A3
#define SERVO_PIN 9
#define BUTTON_PIN_1 12
#define BUTTON_PIN_2 10

// =========================
// АДРЕСА ДАТЧИКОВ
// =========================
#define VL53_ADDR_LEFT  0x30
#define VL53_ADDR_RIGHT 0x29

// =========================
// МОТОРЫ
// =========================
#define MOTOR_LEFT_IN1 8
#define MOTOR_LEFT_PWM 11
#define MOTOR_RIGHT_IN1 7
#define MOTOR_RIGHT_PWM 6

#define MOTOR_SPEED 200
#define MOTOR_TURN_SPEED 150

// =========================
// ТАЙМЕРЫ
// =========================
#define START_DELAY_MS 5000      // 85 секунд ожидания перед маршрутом
#define RUN_TIME_MS 20000        // 100 секунд на выполнение маршрута
#define LOG_INTERVAL_MS 100       // Интервал логирования
#define BUTTON_DEBOUNCE_MS 100    // Дебаунс кнопок
#define OBSTACLE_DIST_MM 200      // Минимальное расстояние до препятствия

// =========================
// МАРШРУТ
// =========================
#define MAX_STEPS 6
#define SIDE_BLUE 0
#define SIDE_YELLOW 1

// =========================
// ОБЪЕКТЫ
// =========================
VL53L0X sensorLeft, sensorRight;
Servo actionServo;

// =========================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// =========================
uint16_t distLeft = 0, distRight = 0;
bool vl53LeftOk = false, vl53RightOk = false;
bool button1Toggle = false;

// =========================
// МАРШРУТ В PROGMEM
// [сторона][мотор][шаг] -> мотор: 0=левый, 1=правый
// =========================
const unsigned long routeTiming[MAX_STEPS] PROGMEM = {5000, 500, 3000, 1000, 0, 0};
const int routeMotors[2][2][MAX_STEPS] PROGMEM = {
  { // SIDE_BLUE
    { MOTOR_SPEED, -MOTOR_TURN_SPEED, MOTOR_SPEED, 0, 0, 0 },
    { MOTOR_SPEED,  MOTOR_TURN_SPEED, MOTOR_SPEED, 0, 0, 0 }
  },
  { // SIDE_YELLOW (инвертированные повороты)
    { MOTOR_SPEED,  MOTOR_TURN_SPEED, MOTOR_SPEED, 0, 0, 0 },
    { MOTOR_SPEED, -MOTOR_TURN_SPEED, MOTOR_SPEED, 0, 0, 0 }
  }
};
const int routeSteps[2] = {4, 4};

// =========================
// ФУНКЦИИ
// =========================
void setMotor(int inPin, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(inPin, HIGH);
    analogWrite(pwmPin, 255 - speed);
  } else if (speed < 0) {
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopMotors() {
  setMotor(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, 0);
  setMotor(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, 0);
}

void initVL53() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);
  Wire.begin();
  delay(10);

  digitalWrite(XSHUT_LEFT, HIGH); delay(50);
  sensorLeft.setTimeout(100);
  if (sensorLeft.init()) {
    sensorLeft.setAddress(VL53_ADDR_LEFT);
    sensorLeft.setMeasurementTimingBudget(30000);
    sensorLeft.startContinuous();
    vl53LeftOk = true;
    Serial.println(F("VL53L OK"));
  }

  digitalWrite(XSHUT_RIGHT, HIGH); delay(50);
  sensorRight.setTimeout(100);
  if (sensorRight.init()) {
    sensorRight.setMeasurementTimingBudget(30000);
    sensorRight.startContinuous();
    vl53RightOk = true;
    Serial.println(F("VL53R OK"));
  }
}

// ИСПРАВЛЕННАЯ ФУНКЦИЯ ПРОВЕРКИ ПРЕПЯТСТВИЙ
bool canDrive() {
  if (!vl53LeftOk || !vl53RightOk) return true; // Если датчики не инициализированы, разрешаем движение
  
  uint16_t l = sensorLeft.readRangeContinuousMillimeters();
  uint16_t r = sensorRight.readRangeContinuousMillimeters();
  
  if (!sensorLeft.timeoutOccurred()) distLeft = l;
  if (!sensorRight.timeoutOccurred()) distRight = r;
  
  return (distLeft >= OBSTACLE_DIST_MM && distRight >= OBSTACLE_DIST_MM);
}

// =========================
// SETUP — ВСЯ ЛОГИКА ЗДЕСЬ
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  
  actionServo.attach(SERVO_PIN);
  actionServo.write(0);
  initVL53();
  
  Serial.println(F("=== READY ==="));
  
  // =========================
  // ЦИКЛ 1: ОЖИДАНИЕ ЗАПУСКА
  // =========================
  unsigned long lastLog = 0;
  bool lastB1State = false;
  
  while (true) {
    bool b1 = !digitalRead(BUTTON_PIN_1);
    bool b2 = !digitalRead(BUTTON_PIN_2);
    
    // Лог только при изменении toggle
    if (b1 != lastB1State) {
      button1Toggle = !button1Toggle;
      lastB1State = b1;
      Serial.print(F("SIDE: "));
      Serial.println(button1Toggle ? F("YELLOW") : F("BLUE"));
    }
    
    // Статус раз в 100мс
    if (millis() - lastLog >= LOG_INTERVAL_MS) {
      Serial.print(F("Wait B2 | Side: "));
      Serial.println(button1Toggle ? F("Y") : F("B"));
      lastLog = millis();
    }
    
    if (b2) {
      Serial.println(F(">>> START <<<"));
      break;
    }
    delay(10);
  }
  
  // =========================
  // ПОДГОТОВКА К ЗАПУСКУ
  // =========================
  stopMotors();
  actionServo.write(0);
  unsigned long runStart = millis();
  int step = 0;
  int side = button1Toggle ? SIDE_YELLOW : SIDE_BLUE;
  int totalSteps = routeSteps[side];
  unsigned long stepStart = millis();
  
  Serial.print(F("Running side: "));
  Serial.println(side == SIDE_YELLOW ? F("YELLOW") : F("BLUE"));
  
  // Ожидание 85 секунд перед стартом
  Serial.println(F("Waiting 85s..."));
  while (millis() - runStart < START_DELAY_MS) {
    delay(50);
  }
  
  stepStart = millis(); // Сбрасываем таймер шага после задержки
  
  // =========================
  // ЦИКЛ 2: ВЫПОЛНЕНИЕ МАРШРУТА
  // =========================
  while (millis() - runStart < RUN_TIME_MS) {
    unsigned long now = millis();
    
    // Проверка препятствий
    if (!canDrive()) {
      stopMotors();
      delay(10);
      continue;
    }
    
    // Проверка завершения текущего шага
    if (now - stepStart >= pgm_read_dword_near(&routeTiming[step])) {
      step++;
      if (step >= totalSteps) {
        Serial.println(F("Route complete"));
        stopMotors();
        break;
      }
      stepStart = now;
    }
    
    // Чтение скоростей из PROGMEM
    int speedL = pgm_read_word_near(&routeMotors[side][0][step]);
    int speedR = pgm_read_word_near(&routeMotors[side][1][step]);
    
    setMotor(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, speedL);
    setMotor(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, speedR);
    
    // Лог раз в 100мс
    if (now - lastLog >= LOG_INTERVAL_MS) {
      Serial.print(F("T:"));
      Serial.print((now - runStart) / 1000);
      Serial.print(F("s | Step:"));
      Serial.print(step);
      Serial.print(F(" | L:"));
      Serial.print(speedL);
      Serial.print(F(" R:"));
      Serial.print(speedR);
      Serial.print(F(" | "));
      Serial.println(canDrive() ? F("OK") : F("OBST"));
      lastLog = now;
    }
    
    delay(10);
  }
  
  // =========================
  // ФИНАЛ: БЕСКОНЕЧНОЕ ДЁРГАНИЕ СЕРВОЙ
  // =========================
  Serial.println(F("=== FLUTTER MODE ==="));
  stopMotors();
  
  while (true) {
    static unsigned long lastFlutter = 0;
    static bool pos = false;
    if (millis() - lastFlutter >= 200) {
      actionServo.write(pos ? 90 : 0);
      pos = !pos;
      lastFlutter = millis();
    }
  }
}

void loop() {
  // Вся логика в setup()
}

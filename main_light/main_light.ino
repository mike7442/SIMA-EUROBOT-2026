#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

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
#define MOTOR_DEADZONE 30

// =========================
// КОНСТАНТЫ
// =========================
#define MIN_DISTANCE_MM 200
#define VL53_INTERVAL_MS 100
#define BUTTON_INTERVAL_MS 100
#define SERIAL_BAUD_RATE 115200

// =========================
// ОБЪЕКТЫ
// =========================
VL53L0X sensorLeft;
VL53L0X sensorRight;
Servo actionServo;

// =========================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// =========================
uint16_t distLeft = 0;
uint16_t distRight = 0;
bool vl53LeftOk = false;
bool vl53RightOk = false;

int button1Value = 0;
int button2Value = 0;
int prevButton1Value = 0;
int button1StateChangeCount = 0;
bool button1Toggle = false; // False = синяя сторона, True = желтая сторона

int sensorArray[3];

unsigned long lastVL53Update = 0;
unsigned long lastButtonUpdate = 0;

// =========================
// МАРШРУТ И СОСТОЯНИЕ
// =========================
enum ActionState { IDLE, GO_FORWARD, TURN, DO_ACTION, WAIT_RELEASE };
ActionState actionState = IDLE;
unsigned long actionStartTime = 0;

// --- Определение маршрутов ---
const int MAX_ROUTE_STEPS = 6; // Увеличьте, если у одного из маршрутов больше шагов

// Маршрут для синей стороны (button1Toggle == false)
const int BLUE_ROUTE_STEPS = 4;
const unsigned long blueRouteTiming[BLUE_ROUTE_STEPS] = {5000, 500, 3000, 1000};
const int blueRouteMotorL[BLUE_ROUTE_STEPS] = {MOTOR_SPEED, -MOTOR_TURN_SPEED, MOTOR_SPEED, 0};
const int blueRouteMotorR[BLUE_ROUTE_STEPS] = {MOTOR_SPEED, MOTOR_TURN_SPEED, MOTOR_SPEED, 0};
const int blueRouteServo[BLUE_ROUTE_STEPS] = {-1, -1, -1, 90};

// Маршрут для желтой стороны (button1Toggle == true)
const int YELLOW_ROUTE_STEPS = 5; // Пример с другим количеством шагов
const unsigned long yellowRouteTiming[YELLOW_ROUTE_STEPS] = {3000, 800, 2000, 600, 2000};
const int yellowRouteMotorL[YELLOW_ROUTE_STEPS] = {MOTOR_SPEED, -MOTOR_TURN_SPEED, MOTOR_SPEED, MOTOR_TURN_SPEED, 0};
const int yellowRouteMotorR[YELLOW_ROUTE_STEPS] = {MOTOR_SPEED, MOTOR_TURN_SPEED, MOTOR_SPEED, -MOTOR_TURN_SPEED, 0};
const int yellowRouteServo[YELLOW_ROUTE_STEPS] = {-1, -1, 90, -1, 0};

int currentRouteStep = 0;
int totalStepsInCurrentRoute = 0; // Будет установлено при старте маршрута

// Указатели на используемые массивы маршрута
const unsigned long* activeRouteTiming;
const int* activeRouteMotorL;
const int* activeRouteMotorR;
const int* activeRouteServo;

// =========================
// ФУНКЦИИ МОТОРОВ
// =========================
void setMotorSpeed(int inPin, int pwmPin, int speed) {
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
  setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, 0);
  setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, 0);
}

void goForward(int speed) {
  setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, speed);
  setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, speed);
}

void turnInPlace(int speedLeft, int speedRight) {
  setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, speedLeft);
  setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, speedRight);
}

// =========================
// ДАТЧИКИ VL53
// =========================
void initVL53() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  Wire.begin();
  delay(10);

  // ЛЕВЫЙ ДАТЧИК
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  sensorLeft.setTimeout(100);
  if (sensorLeft.init()) {
    sensorLeft.setAddress(VL53_ADDR_LEFT);
    sensorLeft.setMeasurementTimingBudget(30000);
    sensorLeft.startContinuous();
    vl53LeftOk = true;
    Serial.println("VL53 LEFT OK");
  } else {
    vl53LeftOk = false;
    Serial.println("VL53 LEFT FAIL");
  }

  // ПРАВЫЙ ДАТЧИК
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  sensorRight.setTimeout(100);
  if (sensorRight.init()) {
    sensorRight.setMeasurementTimingBudget(30000);
    sensorRight.startContinuous();
    vl53RightOk = true;
    Serial.println("VL53 RIGHT OK");
  } else {
    vl53RightOk = false;
    Serial.println("VL53 RIGHT FAIL");
  }
}

void updateVL53() {
  unsigned long now = millis();
  if (now - lastVL53Update < VL53_INTERVAL_MS)
    return;

  lastVL53Update = now;

  if (vl53LeftOk) {
    uint16_t value = sensorLeft.readRangeContinuousMillimeters();
    if (!sensorLeft.timeoutOccurred())
      distLeft = value;
  }

  if (vl53RightOk) {
    uint16_t value = sensorRight.readRangeContinuousMillimeters();
    if (!sensorRight.timeoutOccurred())
      distRight = value;
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  Serial.println("Main Light - Dual Route System");

  // Инициализация кнопок
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);

  // Инициализация моторов
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  // Инициализация сервы
  actionServo.attach(SERVO_PIN);
  actionServo.write(0);

  // Инициализация датчиков
  initVL53();
}

// =========================
// LOOP
// =========================
void loop() {
  unsigned long currentTime = millis();

  // Читаем кнопки с debounce
  if (currentTime - lastButtonUpdate >= BUTTON_INTERVAL_MS) {
    lastButtonUpdate = currentTime;

    button1Value = !digitalRead(BUTTON_PIN_1);
    button2Value = !digitalRead(BUTTON_PIN_2);

    if (button1Value != prevButton1Value) {
      button1StateChangeCount++;
      prevButton1Value = button1Value;
    }

    if (button1StateChangeCount >= 2) {
      button1Toggle = !button1Toggle;
      button1StateChangeCount = 0;
    }
  }

  // Обновляем датчики расстояния
  updateVL53();

  // Проверяем препятствия
  if (vl53LeftOk && vl53RightOk) {
    sensorArray[0] = (distLeft < MIN_DISTANCE_MM || distRight < MIN_DISTANCE_MM) ? 0 : 1;
  } else {
    sensorArray[0] = 1;
  }

  sensorArray[1] = button1Toggle ? 1 : 0;
  sensorArray[2] = button2Value;

  // =========================
  // МАШИНА СОСТОЯНИЙ
  // =========================
  switch (actionState) {
    case IDLE:
      stopMotors();
      actionServo.write(0);  // Серво спрятана
      
      if (button2Value == 1) {
        // Выбор маршрута при старте
        if (button1Toggle) {
          // Желтая сторона
          activeRouteTiming = yellowRouteTiming;
          activeRouteMotorL = yellowRouteMotorL;
          activeRouteMotorR = yellowRouteMotorR;
          activeRouteServo = yellowRouteServo;
          totalStepsInCurrentRoute = YELLOW_ROUTE_STEPS;
          Serial.println("START - Yellow Side Selected");
        } else {
          // Синяя сторона
          activeRouteTiming = blueRouteTiming;
          activeRouteMotorL = blueRouteMotorL;
          activeRouteMotorR = blueRouteMotorR;
          activeRouteServo = blueRouteServo;
          totalStepsInCurrentRoute = BLUE_ROUTE_STEPS;
          Serial.println("START - Blue Side Selected");
        }
        
        actionState = GO_FORWARD;
        actionStartTime = currentTime;
        currentRouteStep = 0;
      }
      break;

    case GO_FORWARD:
    case TURN:
    case DO_ACTION:
      // Проверяем, не истекло ли время текущего этапа
      if (currentTime - actionStartTime >= activeRouteTiming[currentRouteStep]) {
        currentRouteStep++;
        if (currentRouteStep >= totalStepsInCurrentRoute) {
          // Маршрут завершен, переходим в ожидание отпускания кнопки
          actionState = WAIT_RELEASE;
        } else {
          // Переходим на следующий этап маршрута
          actionStartTime = currentTime;
        }
      }

      // Проверяем препятствия и выполняем маршрут
      if (button2Value == 1) {
        if (sensorArray[0] == 1) {
          // Нет препятствий, выполняем маршрут
          int speedL = activeRouteMotorL[currentRouteStep];
          int speedR = activeRouteMotorR[currentRouteStep];
          
          setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, speedL);
          setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, speedR);
          
          // Управляем сервой если нужно
          if (activeRouteServo[currentRouteStep] >= 0) {
            actionServo.write(activeRouteServo[currentRouteStep]);
          }
        } else {
          // Препятствие! Стоим на месте
          stopMotors();
        }
      } else {
        // button2 отпущена во время выполнения маршрута
        stopMotors();
        actionServo.write(0);
        actionState = IDLE;
      }
      break;

    case WAIT_RELEASE:
      stopMotors();
      actionServo.write(90);  // Серво поднята после маршрута
      
      if (button2Value == 0) {
        actionServo.write(0);  // Спрячем серво
        actionState = IDLE;
      }
      break;
  }

  // Логирование
  Serial.print("State: ");
  switch (actionState) {
    case IDLE: Serial.print("IDLE"); break;
    case GO_FORWARD: Serial.print("GO"); break;
    case TURN: Serial.print("TURN"); break;
    case DO_ACTION: Serial.print("ACTION"); break;
    case WAIT_RELEASE: Serial.print("WAIT"); break;
  }
  Serial.print(" | Can drive: ");
  Serial.print(sensorArray[0]);
  Serial.print(" | B1: ");
  Serial.print(sensorArray[1]);
  Serial.print(" | B2: ");
  Serial.println(sensorArray[2]);

  delay(50);
}

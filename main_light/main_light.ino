#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
// Для PROGMEM
#include <avr/pgmspace.h>

// =========================
// ПИНЫ
// =========================
#define SDA_PIN A4
#define SCL_PIN A5
#define XSHUT_LEFT A2
#define XSHUT_RIGHT A3
#define SERVO_PIN 9
#define BUTTON_PIN_1 12
#define BUTTON_PIN_2 10

// =========================
// АДРЕСА ДАТЧИКОВ
// =========================
#define VL53_ADDR_LEFT 0x30
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
#define START_DELAY_MS 85000      // 85 секунд ожидания перед началом движения
#define TOTAL_RUN_TIME_MS 100000  // 100 секунд на весь боевой режим (включая ожидание)
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
bool button1Toggle = false;  // False = синяя сторона, True = желтая сторона

int sensorArray[3];

unsigned long lastVL53Update = 0;
unsigned long lastButtonUpdate = 0;

// =========================
// МАРШРУТ И СОСТОЯНИЕ
// =========================
enum ActionState {
  IDLE,
  WAIT_START_DELAY,
  GO_FORWARD,
  TURN,
  DO_ACTION,
  WAIT_RELEASE,
  TIMEOUT
};
ActionState actionState = IDLE;
unsigned long actionStartTime = 0;
unsigned long battleModeStartTime = 0;

// --- Определение маршрутов в PROGMEM ---
const int BLUE_ROUTE_STEPS = 4;
const unsigned long blueRouteTiming[BLUE_ROUTE_STEPS] PROGMEM = { 5000, 500, 3000, 1000 };
const int blueRouteMotorL[BLUE_ROUTE_STEPS] PROGMEM = { MOTOR_SPEED, -MOTOR_TURN_SPEED, MOTOR_SPEED, 0 };
const int blueRouteMotorR[BLUE_ROUTE_STEPS] PROGMEM = { MOTOR_SPEED, MOTOR_TURN_SPEED, MOTOR_SPEED, 0 };
const int blueRouteServo[BLUE_ROUTE_STEPS] PROGMEM = { -1, -1, -1, 90 };

const int YELLOW_ROUTE_STEPS = 5;
const unsigned long yellowRouteTiming[YELLOW_ROUTE_STEPS] PROGMEM = { 3000, 800, 2000, 600, 2000 };
const int yellowRouteMotorL[YELLOW_ROUTE_STEPS] PROGMEM = { MOTOR_SPEED, -MOTOR_TURN_SPEED, MOTOR_SPEED, MOTOR_TURN_SPEED, 0 };
const int yellowRouteMotorR[YELLOW_ROUTE_STEPS] PROGMEM = { MOTOR_SPEED, MOTOR_TURN_SPEED, MOTOR_SPEED, -MOTOR_TURN_SPEED, 0 };
const int yellowRouteServo[YELLOW_ROUTE_STEPS] PROGMEM = { -1, -1, 90, -1, 0 };

int currentRouteStep = 0;
int totalStepsInCurrentRoute = 0;  // Будет установлено при старте маршрута

// Указатели на используемые массивы маршрута (в PROGMEM)
PGM_P activeRouteTiming;
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
    Serial.println(F("VL53 LEFT OK"));
  } else {
    vl53LeftOk = false;
    Serial.println(F("VL53 LEFT FAIL"));
  }

  // ПРАВЫЙ ДАТЧИК
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  sensorRight.setTimeout(100);
  if (sensorRight.init()) {
    sensorRight.setMeasurementTimingBudget(30000);
    sensorRight.startContinuous();
    vl53RightOk = true;
    Serial.println(F("VL53 RIGHT OK"));
  } else {
    vl53RightOk = false;
    Serial.println(F("VL53 RIGHT FAIL"));
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
// ФУНКЦИИ ДЛЯ ЧТЕНИЯ ДАННЫХ ИЗ PROGMEM
// =========================
inline unsigned long pgm_read_route_timing(PGM_P addr, int index) {
  return pgm_read_dword_near(addr + index * sizeof(unsigned long));
}

inline int pgm_read_route_int(const int* addr, int index) {
  return pgm_read_word_near(addr + index * sizeof(int));
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  Serial.println(F("Main Light - Optimized"));

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
        battleModeStartTime = currentTime;
        Serial.print(F("BATTLE MODE STARTED - Side: "));
        Serial.println(button1Toggle ? F("Yellow") : F("Blue"));
        actionState = WAIT_START_DELAY;
        actionStartTime = currentTime;
      }
      break;

    case WAIT_START_DELAY:
      stopMotors();
      actionServo.write(0);

      if (currentTime - actionStartTime >= START_DELAY_MS) {
        Serial.println(F("Start delay over, beginning route execution."));
        if (button1Toggle) {
          // Желтая сторона - указатели на PROGMEM
          activeRouteTiming = (PGM_P)yellowRouteTiming;
          activeRouteMotorL = yellowRouteMotorL;
          activeRouteMotorR = yellowRouteMotorR;
          activeRouteServo = yellowRouteServo;
          totalStepsInCurrentRoute = YELLOW_ROUTE_STEPS;
        } else {
          // Синяя сторона - указатели на PROGMEM
          activeRouteTiming = (PGM_P)blueRouteTiming;
          activeRouteMotorL = blueRouteMotorL;
          activeRouteMotorR = blueRouteMotorR;
          activeRouteServo = blueRouteServo;
          totalStepsInCurrentRoute = BLUE_ROUTE_STEPS;
        }

        actionState = GO_FORWARD;
        actionStartTime = currentTime;
        currentRouteStep = 0;
      }

      if (currentTime - battleModeStartTime >= TOTAL_RUN_TIME_MS) {
        Serial.println(F("Total run time expired while waiting for start delay."));
        actionState = TIMEOUT;
        stopMotors();
        actionServo.write(0);
      }
      break;

    case GO_FORWARD:  // Это состояние теперь обрабатывает все этапы маршрута
    case TURN:
    case DO_ACTION:

      // Проверяем таймаут боевого режима (включая выполнение маршрута)
      if (currentTime - battleModeStartTime >= TOTAL_RUN_TIME_MS) {
        Serial.println(F("Total run time expired during route execution."));
        actionState = TIMEOUT;  // Переход в состояние таймаута
        stopMotors();
        actionServo.write(0);
        while (1) {
          delay(1000);
          actionServo.write(0);
          delay(1000);
          actionServo.write(90);
        }
        break;  // Важно выйти из switch, чтобы остальная логика не выполнялась
      }

      // Проверяем препятствия
      bool canDrive = (sensorArray[0] == 1);

      // --- Новая логика ---
      // Проверяем, нужно ли завершить текущий шаг маршрута
      // Условие: можно двигаться ИЛИ не было препятствия на предыдущей итерации
      // Это позволяет продолжить отсчёт времени шага, только если робот не был остановлен препятствием.
      // Однако, если робот стоит из-за препятствия, время шага "заморожено".
      // Более точная проверка: если можно двигаться, проверяем время.
      // Если нельзя, просто останавливаем моторы и НЕ обновляем время начала шага.
      if (canDrive) {
        // Робот может двигаться, проверяем, не истекло ли время текущего этапа
        if (currentTime - actionStartTime >= pgm_read_route_timing(activeRouteTiming, currentRouteStep)) {
          currentRouteStep++;
          if (currentRouteStep >= totalStepsInCurrentRoute) {
            // Маршрут завершен, переходим в ожидание отпускания кнопки
            actionState = WAIT_RELEASE;
            Serial.println(F("Route completed, waiting for button release."));
          } else {
            // Переходим на следующий этап маршрута
            actionStartTime = currentTime;  // Время обновляется только при переходе к новому шагу
          }
        }

        // Выполняем действия для текущего шага (движение, серва), если можно двигаться
        int speedL = pgm_read_route_int(activeRouteMotorL, currentRouteStep);
        int speedR = pgm_read_route_int(activeRouteMotorR, currentRouteStep);

        setMotorSpeed(MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, speedL);
        setMotorSpeed(MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, speedR);

        int servo_pos = pgm_read_route_int(activeRouteServo, currentRouteStep);
        if (servo_pos >= 0) {
          actionServo.write(servo_pos);
        }
      } else {
        // Препятствие! Стоим на месте.
        // ВАЖНО: НЕ обновляем actionStartTime, чтобы таймер шага "заморозился".
        // Следующий шаг начнётся сразу после того, как исчезнет препятствие.
        stopMotors();
        // Дополнительно: можно оставить серву в текущем положении или изменить (по желанию).
        // actionServo.write(...); // <-- При необходимости добавьте сюда команду для сервы при остановке.
        Serial.println(F("Obstacle detected, pausing route timer."));  // Для отладки
      }
      // --- Конец новой логики ---
      break;  // Конец case GO_FORWARD / TURN / DO_ACTION

    case WAIT_RELEASE:
      // Проверяем, нужно ли выполнить действия при входе в состояние (один раз)
      static bool enteredWaitRelease = false;  // Статическая переменная для отслеживания входа
      if (!enteredWaitRelease) {
        Serial.println(F("Entered WAIT_RELEASE: Motors Stopped, Waiting for Button 2 Release."));  // Отладочное сообщение
        stopMotors();                                                                              // Остановка моторов при входе
        actionServo.write(90);                                                                     // Серво поднята после маршрута
        enteredWaitRelease = true;                                                                 // Помечаем, что вошли
      }

      // Проверяем, отпущена ли кнопка 2
      if (button2Value == 0) {
        Serial.println(F("Button 2 released, returning to IDLE."));  // Сообщение при выходе
        actionServo.write(0);                                        // Спрячем серво
        actionState = IDLE;
        enteredWaitRelease = false;  // Сбрасываем флаг для следующего цикла
      }
      // НЕ ставим break; здесь, чтобы после отпускания кнопки (или если она сразу была отпущена)
      // управление перешло к следующему case - TIMEOUT, и запустился бы while(1) flutter.
      // Если кнопка НЕ отпущена, break; не сработает, и loop() начнётся заново.
      // Если кнопка ОТПУЩЕНА, break; не сработает, и управление пойдёт к следующему case.

    case TIMEOUT:
      // Остановка моторов и сервы при входе в финальное состояние
      stopMotors();
      actionServo.write(0);
      Serial.println(F("Entering final flutter mode."));

      // Бесконечный цикл дёргания сервы
      static unsigned long lastFlutterTime = millis();  // Статическая переменная для отслеживания времени внутри while
      static bool flutterPos = false;                   // Статическая переменная для переключения позиции внутри while
      const unsigned long flutterInterval = 200;        // Интервал дёргания в миллисекундах

      while (1) {                      // Бесконечный цикл
        unsigned long now = millis();  // Локальная переменная для текущего времени в while
        if (now - lastFlutterTime >= flutterInterval) {
          actionServo.write(flutterPos ? 90 : 0);  // Переключаем позицию
          flutterPos = !flutterPos;                // Переключаем флаг
          lastFlutterTime = now;                   // Обновляем время последнего движения
        }
        // delay(10); // Небольшая задержка внутри while для стабильности, при желании
      }
      // Этот break; unreachable, но компилятор может ругаться, если его не поставить.
      // break;
  }

  // Логирование (это выполняется только если actionState != TIMEOUT, т.к. в TIMEOUT while(1))
  // Если хочется логировать и в финальном состоянии, нужно вынести логику логирования
  // в каждый case отдельно или завернуть в условие if (actionState != TIMEOUT) { ... }
  // Для простоты, логирование оставлено как есть - оно не будет выполняться после входа в while(1).
  if (actionState != TIMEOUT) {
    Serial.print(F("State: "));
    switch (actionState) {
      case IDLE: Serial.print(F("IDLE")); break;
      case WAIT_START_DELAY: Serial.print(F("WAIT_START")); break;
      case GO_FORWARD: Serial.print(F("GO")); break;
      case TURN: Serial.print(F("TURN")); break;
      case DO_ACTION: Serial.print(F("ACTION")); break;
      case WAIT_RELEASE:
        Serial.print(F("WAIT_REL"));
        break;
        // TIMEOUT не будет выведено из-за условия if
    }
    Serial.print(F(" | Time: "));
    Serial.print((currentTime - battleModeStartTime) / 1000.0, 1);
    Serial.print(F("s/"));
    Serial.print(TOTAL_RUN_TIME_MS / 1000.0, 1);
    Serial.print(F("s | Can drive: "));
    Serial.print(sensorArray[0]);
    Serial.print(F(" | B1: "));
    Serial.print(sensorArray[1]);
    Serial.print(F(" | B2: "));
    Serial.println(sensorArray[2]);
  }

  delay(50);
}

#include <Wire.h>
#include <VL53L0X.h>

// =========================
// ПИНЫ
// =========================
#define SDA_PIN A4
#define SCL_PIN A5
#define XSHUT_LEFT  A2
#define XSHUT_RIGHT A3

const int BUTTON_PIN_1 = 10;
const int BUTTON_PIN_2 = 12;

// =========================
// АДРЕСА ДАТЧИКОВ
// =========================
#define VL53_ADDR_LEFT  0x30
#define VL53_ADDR_RIGHT 0x29

// =========================
// ОБЪЕКТЫ ДАТЧИКОВ
// =========================
VL53L0X sensorLeft;
VL53L0X sensorRight;

// =========================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// =========================
uint16_t distLeft = 0;
uint16_t distRight = 0;
bool vl53LeftOk = false;
bool vl53RightOk = false;
bool vl53NewData = false;

int button1Value = 0;
int button2Value = 0;

// Массив для хранения: [button1, button2, obstacle]
int sensorArray[3];

// Минимальное расстояние для определения препятствия
const uint16_t MIN_DISTANCE_MM = 200; // например, 200 мм

// =========================
// НАСТРОЙКИ ОПРОСА
// =========================
unsigned long lastVL53Update = 0;
const unsigned long VL53_INTERVAL_MS = 100;

// =========================
// ИНИЦИАЛИЗАЦИЯ ДАТЧИКОВ
// =========================
void initVL53()
{
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  Wire.begin();
  delay(10);

  // -------------------------
  // ЛЕВЫЙ ДАТЧИК
  // -------------------------
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  sensorLeft.setTimeout(100);
  if (sensorLeft.init())
  {
    sensorLeft.setAddress(VL53_ADDR_LEFT);
    sensorLeft.setMeasurementTimingBudget(30000);
    sensorLeft.startContinuous();
    vl53LeftOk = true;
    Serial.println("VL53 LEFT init OK, addr = 0x30");
  }
  else
  {
    vl53LeftOk = false;
    Serial.println("VL53 LEFT init FAIL");
  }

  // -------------------------
  // ПРАВЫЙ ДАТЧИК
  // -------------------------
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  sensorRight.setTimeout(100);
  if (sensorRight.init())
  {
    sensorRight.setMeasurementTimingBudget(30000);
    sensorRight.startContinuous();
    vl53RightOk = true;
    Serial.println("VL53 RIGHT init OK, addr = 0x29");
  }
  else
  {
    vl53RightOk = false;
    Serial.println("VL53 RIGHT init FAIL");
  }
}

// =========================
// ФОНОВОЕ ОБНОВЛЕНИЕ ДАТЧИКОВ
// =========================
void updateVL53()
{
  unsigned long now = millis();
  if (now - lastVL53Update < VL53_INTERVAL_MS)
    return;

  lastVL53Update = now;
  vl53NewData = false;

  if (vl53LeftOk)
  {
    uint16_t value = sensorLeft.readRangeContinuousMillimeters();
    if (!sensorLeft.timeoutOccurred())
    {
      distLeft = value;
    }
  }

  if (vl53RightOk)
  {
    uint16_t value = sensorRight.readRangeContinuousMillimeters();
    if (!sensorRight.timeoutOccurred())
    {
      distRight = value;
    }
  }

  vl53NewData = true;
}

// =========================
// SETUP
// =========================
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Start combined sensor system");

  // Инициализация кнопок
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);

  // Инициализация дальномеров
  initVL53();
}

// =========================
// LOOP
// =========================
void loop()
{
  // Читаем кнопки
  button1Value = !digitalRead(BUTTON_PIN_1); // Инвертируем, если подтянуты к VCC
  button2Value = !digitalRead(BUTTON_PIN_2);

  // Обновляем дальномеры
  updateVL53();

  // Заполняем массив
  sensorArray[0] = button1Value;
  sensorArray[1] = button2Value;

  // Проверяем, есть ли препятствие
  if (vl53LeftOk && vl53RightOk) {
    if (distLeft < MIN_DISTANCE_MM || distRight < MIN_DISTANCE_MM) {
      sensorArray[2] = 0; // Препятствие
    } else {
      sensorArray[2] = 1; // Нет препятствия
    }
  } else {
    sensorArray[2] = 1; // Если датчики не работают, считаем, что "нет препятствия"
  }

  // Печатаем лог
  Serial.print("Btn1: ");
  Serial.print(sensorArray[0]);
  Serial.print(", Btn2: ");
  Serial.print(sensorArray[1]);
  Serial.print(", Obstacle: ");
  Serial.println(sensorArray[2]);

  delay(100); // Небольшая задержка для стабильности
}

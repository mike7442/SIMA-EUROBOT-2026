// Пины для энкодеров
#define ENCODER_LEFT_A    2 // INT0
#define ENCODER_LEFT_B    4
#define ENCODER_RIGHT_A   3 // INT1
#define ENCODER_RIGHT_B   5

// Переменные для хранения позиции
volatile long encoderLeftPos = 0;
volatile long encoderRightPos = 0;

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

  Serial.println("Simple encoder monitoring started...");
}

void loop() {
  // Печатаем позиции каждые 500 мс
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) {
    Serial.print("Left Encoder: ");
    Serial.print(encoderLeftPos);
    Serial.print("\tRight Encoder: ");
    Serial.println(encoderRightPos);
    lastPrintTime = millis();
  }
}

void updateEncoderLeft() {
  // Проверяем состояние пина 4
  if (digitalRead(ENCODER_LEFT_B) == HIGH) {
    encoderLeftPos++; // вращение вперёд
  } else {
    encoderLeftPos--; // вращение назад
  }
}

void updateEncoderRight() {
  // Проверяем состояние пина 5
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) {
    encoderRightPos++; // вращение вперёд
  } else {
    encoderRightPos--; // вращение назад
  }
}

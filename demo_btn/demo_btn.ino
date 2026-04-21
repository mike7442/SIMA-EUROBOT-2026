// Пины для кнопок
const int BUTTON_PIN_1 = 10;
const int BUTTON_PIN_2 = 12;

void setup() {
  Serial.begin(115200);

  // Настройка пинов как входы (внешняя подтяжка к земле)
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);

  Serial.println("Simple button printing started...");
}

void loop() {
  // Читаем состояние кнопок
  int button1Value = digitalRead(BUTTON_PIN_1);
  int button2Value = digitalRead(BUTTON_PIN_2);

  // Печатаем значения
  Serial.print("Button 1: ");
  Serial.print(button1Value);
  Serial.print("\tButton 2: ");
  Serial.println(button2Value);
}

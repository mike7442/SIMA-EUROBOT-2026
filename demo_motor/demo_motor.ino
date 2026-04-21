// Пины для управления правым мотором
#define MOTOR_RIGHT_IN1 7
#define MOTOR_RIGHT_PWM 6  // ШИМ

// Пины для управления левым мотором
#define MOTOR_LEFT_IN1 8
#define MOTOR_LEFT_PWM 11  // ШИМ

void setup() {
  Serial.begin(9600);

  // Настраиваем пины как выходы
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);

  // Останавливаем моторы
  stopMotors();

  Serial.println("Dual motor control via Serial started...");
  Serial.println("Format: [motor_id],[speed]");
  Serial.println("motor_id: 0=left, 1=right, 2=both");
  Serial.println("speed: -255 to 255 (negative = backward)");
}

void loop() {
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
}

void setMotorSpeed(int inPin, int pwmPin, int speed) {
  if (speed > 0) {
    // Вращение вперёд
    digitalWrite(inPin, HIGH);
    analogWrite(pwmPin, 255 - speed);  // ⚠️ Обратная логика
  } else if (speed < 0) {
    // Вращение назад
    digitalWrite(inPin, LOW);
    analogWrite(pwmPin, -speed); // подаём модуль скорости
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

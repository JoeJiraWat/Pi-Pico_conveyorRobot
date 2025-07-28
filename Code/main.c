#include <Wire.h>
#include <Servo.h>

#define SDA_PIN 4
#define SCL_PIN 5

#define IR_SENSOR_PIN 14
#define SERVO1_PIN 16
#define SERVO2_PIN 17

#define TCS34725_ADDRESS 0x29
#define TCS34725_COMMAND_BIT 0x80

Servo servo1;
Servo servo2;

void tcs34725_write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCS34725_ADDRESS);
  Wire.write(TCS34725_COMMAND_BIT | reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint16_t tcs34725_read16(uint8_t reg) {
  Wire.beginTransmission(TCS34725_ADDRESS);
  Wire.write(TCS34725_COMMAND_BIT | reg);
  Wire.endTransmission(false);  // repeated start

  Wire.requestFrom(TCS34725_ADDRESS, 2);
  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  return (high << 8) | low;
}

void tcs34725_init() {
  tcs34725_write8(0x00, 0x01); // Power ON
  delay(10);
  tcs34725_write8(0x00, 0x03); // เปิด RGBC
  tcs34725_write8(0x01, 0xD5); // Integration time
}

void read_color(uint16_t *r, uint16_t *g, uint16_t *b) {
  *r = tcs34725_read16(0x16);
  *g = tcs34725_read16(0x18);
  *b = tcs34725_read16(0x1A);
}

void setup() {
  Serial.begin(115200);

  // setup I2C
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  // setup IR Sensor
  pinMode(IR_SENSOR_PIN, INPUT);

  // Servo setup
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(0);
  servo2.write(0);

  // TCS34725 setup
  tcs34725_init();
  delay(100);
}

void loop() {
  if (digitalRead(IR_SENSOR_PIN) == LOW) { // ตรวจจับวัตถุ
    uint16_t r, g, b;
    read_color(&r, &g, &b);

    Serial.print("R: "); Serial.print(r);
    Serial.print("G: "); Serial.print(g);
    Serial.print("B: "); Serial.println(b);

    if (r > g && r > b) {
      // เงื่อนไขของสีแดง
      servo1.write(0);
      servo2.write(0);
    } else if (b > r && b > g) {
      // เงื่อนไขของสีน้ำเงิน
      servo1.write(90);
      servo2.write(0);
    } else if (g > r && g > b) {
      // เงื่อนไขของสีเขียว
      servo1.write(0);
      servo2.write(90);
    }

    delay(1000);
  } else {
    // reset servo
    servo1.write(0);
    servo2.write(0);
  }

  delay(200);
}

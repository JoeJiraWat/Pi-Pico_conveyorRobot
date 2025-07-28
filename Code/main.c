#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#define I2C_PORT i2c1
#define SDA_PIN 4
#define SCL_PIN 5

#define IR_SENSOR_PIN 14
#define SERVO1_PIN 16
#define SERVO2_PIN 17

#define TCS34725_ADDRESS 0x29
#define TCS34725_COMMAND_BIT 0x80

#define SERVO_MIN 1000  // 0 degree
#define SERVO_MID 1500  // 90 degree

void i2c_write8(uint8_t reg, uint8_t value) {
    uint8_t buf[] = {TCS34725_COMMAND_BIT | reg, value};
    i2c_write_blocking(I2C_PORT, TCS34725_ADDRESS, buf, 2, false);
}

void i2c_read16(uint8_t reg, uint16_t* value) {
    uint8_t reg_addr = TCS34725_COMMAND_BIT | reg;
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, TCS34725_ADDRESS, &reg_addr, 1, true);
    i2c_read_blocking(I2C_PORT, TCS34725_ADDRESS, buf, 2, false);
    *value = (buf[1] << 8) | buf[0];
}

void tcs34725_init() {
    i2c_write8(0x00, 0x01);  // Power ON
    sleep_ms(10);
    i2c_write8(0x00, 0x03);  // Enable RGBC
    i2c_write8(0x01, 0xD5);  // Integration time
}

void read_color(uint16_t* r, uint16_t* g, uint16_t* b) {
    i2c_read16(0x16, r);  // Red
    i2c_read16(0x18, g);  // Green
    i2c_read16(0x1A, b);  // Blue
}

void setup_servo(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 20000); // 50Hz
    pwm_set_enabled(slice, true);
}

void set_servo_angle(uint pin, int angle_deg) {
    uint slice = pwm_gpio_to_slice_num(pin);
    int pulse_width = 1000 + (angle_deg * 1000) / 180; // 1000-2000us
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), pulse_width);
}

int main() {
    stdio_init_all();

    // Init I2C
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // IR Sensor
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);

    // Servo
    setup_servo(SERVO1_PIN);
    setup_servo(SERVO2_PIN);

    // Init color sensor
    tcs34725_init();
    sleep_ms(100);

    while (true) {
        if (gpio_get(IR_SENSOR_PIN) == 0) {  // IR detects object
            uint16_t r, g, b;
            read_color(&r, &g, &b);

            printf("R:%d G:%d B:%d\n", r, g, b);

            if (r > g && r > b) {
                // RED → do nothing
                set_servo_angle(SERVO1_PIN, 0);
                set_servo_angle(SERVO2_PIN, 0);
            } else if (b > r && b > g) {
                // BLUE → Servo1 → 90
                set_servo_angle(SERVO1_PIN, 90);
                set_servo_angle(SERVO2_PIN, 0);
            } else if (g > r && g > b) {
                // GREEN → Servo2 → 90
                set_servo_angle(SERVO1_PIN, 0);
                set_servo_angle(SERVO2_PIN, 90);
            }

            sleep_ms(1000); // Wait before next read
        } else {
            // No object detected
            set_servo_angle(SERVO1_PIN, 0);
            set_servo_angle(SERVO2_PIN, 0);
        }

        sleep_ms(200);
    }

    return 0;
}

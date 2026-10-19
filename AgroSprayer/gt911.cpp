#include "gt911.h"
#include "board_config.h"
#include "hw_i2c.h"
#include "io_extension.h"
#include "driver/gpio.h"

static i2c_master_dev_handle_t s_gt = nullptr;

static bool reg_read(uint16_t reg, uint8_t *data, size_t len) {
    uint8_t a[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    return hw_i2c_write_read(s_gt, a, sizeof(a), data, len);
}

static bool reg_write8(uint16_t reg, uint8_t value) {
    uint8_t d[3] = {(uint8_t)(reg >> 8), (uint8_t)reg, value};
    return hw_i2c_write(s_gt, d, sizeof(d));
}

bool gt911_begin() {
    if (!hw_i2c_begin() || !io_extension_begin()) return false;

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << GT911_INT;
    io.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io));

    io_extension_write(IOX_TOUCH_RST, false);
    delay(100);
    gpio_set_level(GT911_INT, 0);
    delay(100);
    io_extension_write(IOX_TOUCH_RST, true);
    delay(200);

    io = {};
    io.pin_bit_mask = 1ULL << GT911_INT;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io));

    if (!hw_i2c_add_device(GT911_ADDR, &s_gt)) return false;

    uint8_t id[3] = {};
    if (!reg_read(0x8140, id, 3)) return false;
    Serial.printf("GT911 ID: %c%c%c\n", id[0], id[1], id[2]);
    return true;
}

TouchPoint gt911_read() {
    TouchPoint p = {false, 0, 0};
    if (!s_gt) return p;

    uint8_t status = 0;
    if (!reg_read(0x814E, &status, 1)) return p;
    if ((status & 0x80) == 0) return p;

    uint8_t count = status & 0x0F;
    if (count == 0 || count > 5) {
        reg_write8(0x814E, 0);
        return p;
    }

    uint8_t d[8] = {};
    if (!reg_read(0x814F, d, sizeof(d))) {
        reg_write8(0x814E, 0);
        return p;
    }

    p.x = (uint16_t)d[1] | ((uint16_t)d[2] << 8);
    p.y = (uint16_t)d[3] | ((uint16_t)d[4] << 8);
    p.pressed = p.x < LCD_H_RES && p.y < LCD_V_RES;

    reg_write8(0x814E, 0);
    return p;
}

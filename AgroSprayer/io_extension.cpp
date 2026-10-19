#include "io_extension.h"
#include "board_config.h"
#include "hw_i2c.h"

static i2c_master_dev_handle_t s_io = nullptr;
static uint8_t s_out = 0xFF;

static bool wr(uint8_t reg, uint8_t value) {
    uint8_t d[2] = {reg, value};
    return hw_i2c_write(s_io, d, sizeof(d));
}

bool io_extension_begin() {
    if (s_io) return true;
    if (!hw_i2c_begin()) return false;
    if (!hw_i2c_add_device(IOX_ADDR, &s_io)) return false;
    if (!wr(IOX_REG_MODE, 0xFF)) return false;
    s_out = 0xFF;
    return wr(IOX_REG_OUTPUT, s_out);
}

bool io_extension_write(uint8_t pin, bool value) {
    if (!s_io || pin > 7) return false;
    if (value) s_out |= 1u << pin;
    else       s_out &= ~(1u << pin);
    return wr(IOX_REG_OUTPUT, s_out);
}

void backlight_on()  { io_extension_write(IOX_BACKLIGHT, true); }
void backlight_off() { io_extension_write(IOX_BACKLIGHT, false); }

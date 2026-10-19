#include "hw_i2c.h"
#include "board_config.h"

static i2c_master_bus_handle_t s_bus = nullptr;

bool hw_i2c_begin() {
    if (s_bus) return true;
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = I2C_NUM_0;
    cfg.sda_io_num = I2C_SDA;
    cfg.scl_io_num = I2C_SCL;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = true;
    return i2c_new_master_bus(&cfg, &s_bus) == ESP_OK;
}

bool hw_i2c_add_device(uint8_t address, i2c_master_dev_handle_t *out) {
    if (!s_bus || !out) return false;
    i2c_device_config_t cfg = {};
    cfg.device_address = address;
    cfg.scl_speed_hz = I2C_FREQ_HZ;
    return i2c_master_bus_add_device(s_bus, &cfg, out) == ESP_OK;
}

bool hw_i2c_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len) {
    return dev && data && len && i2c_master_transmit(dev, data, len, 100) == ESP_OK;
}

bool hw_i2c_write_read(i2c_master_dev_handle_t dev, const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len) {
    return dev && tx && tx_len && rx && rx_len &&
           i2c_master_transmit_receive(dev, tx, tx_len, rx, rx_len, 100) == ESP_OK;
}

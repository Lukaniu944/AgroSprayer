#pragma once
#include <Arduino.h>
#include "driver/i2c_master.h"
bool hw_i2c_begin();
bool hw_i2c_add_device(uint8_t address, i2c_master_dev_handle_t *out);
bool hw_i2c_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len);
bool hw_i2c_write_read(i2c_master_dev_handle_t dev, const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len);

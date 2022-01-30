/**
 * @file I2C_Drv.h
 * @author Ricard Bitriá Ribes (https://github.com/dracir9)
 * @brief Touch screen I2C driver
 * @version 0.1
 * @date 2021-08-10
 * 
 * @copyright Copyright (c) 2021 Ricard Bitriá Ribes
 * 
 * Based on original ESP-IDF I2C driver.
 * Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#ifndef I2C_DRV_H
#define I2C_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "esp_err.h"

/**
 * @brief I2C initialization parameters
 */
typedef struct{
    i2c_mode_t mode;       /*!< I2C mode */
    gpio_num_t sda_io_num;        /*!< GPIO number for I2C sda signal */
    gpio_pullup_t sda_pullup_en;  /*!< Internal GPIO pull mode for I2C sda signal*/
    gpio_num_t scl_io_num;        /*!< GPIO number for I2C scl signal */
    gpio_pullup_t scl_pullup_en;  /*!< Internal GPIO pull mode for I2C scl signal*/
    struct {
        uint32_t clk_speed;     /*!< I2C clock frequency for master mode, (no higher than 1MHz for now) */
    } master;
    struct {
        uint8_t addr_10bit_en;  /*!< I2C 10bit address mode enable for slave mode */
        uint16_t slave_addr;    /*!< I2C address for slave mode */
    } slave;
}i2c_full_config_t;

esp_err_t i2cTch_init_params(i2c_port_t i2c_num, const i2c_full_config_t *i2c_conf);
esp_err_t i2cTch_install_driver(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len);
esp_err_t i2cTch_delete_driver(i2c_port_t i2c_num);
int i2cTch_slave_read_data(i2c_port_t i2c_num, uint8_t *data, size_t max_size, TickType_t ticks_to_wait);
esp_err_t i2cTch_master_send_data(i2c_port_t i2c_num, uint8_t* data, uint16_t len, uint8_t address, TickType_t ticks_to_wait);
esp_err_t i2cTch_master_read_data(i2c_port_t i2c_num, uint8_t* data, uint16_t len, uint8_t address, TickType_t ticks_to_wait);

i2c_mode_t i2cTch_get_mode(i2c_port_t i2c_num);
esp_err_t i2cTch_set_mode(i2c_port_t i2c_num, i2c_mode_t mode);

bool i2cTch_getEvent(int* event);

#ifdef __cplusplus
}
#endif
#endif

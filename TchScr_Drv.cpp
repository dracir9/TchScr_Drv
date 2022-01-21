/**
 * @file TchScr_Drv.cpp
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

#include "TchScr_Drv.h"
#include "esp_log.h"

TchScr_Drv::TchScr_Drv(i2c_port_t i2c_num) :
    i2c_num (i2c_num)
{
}

TchScr_Drv::~TchScr_Drv()
{
    i2cTch_delete_driver(i2c_num);
}

esp_err_t TchScr_Drv::begin(i2c_mode_t mode, gpio_num_t sda, gpio_num_t scl, uint16_t addr, uint32_t freq)
{
    i2c_full_config_t conf;
    conf.mode = mode;
    conf.sda_io_num = sda;         // select GPIO specific to your project
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = scl;         // select GPIO specific to your project
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.slave.addr_10bit_en = false;
    conf.slave.slave_addr = addr;
    conf.master.clk_speed = freq;

    esp_err_t err = i2cTch_init_params(i2c_num, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    err = i2cTch_install_driver(i2c_num, mode, 128);
    if (err != ESP_OK) {
        return err;
    }

    hw_init = true;
    return ESP_OK;
}

esp_err_t TchScr_Drv::getLastPoint(Vec2h* point, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    esp_err_t err = i2cTch_set_mode(i2c_num, I2C_MODE_MASTER);
    if (err != ESP_OK) {
        return err;
    }
    return i2cTch_master_read_data(i2c_num, (uint8_t*)point, 4, 0x81, timeout);
}

esp_err_t TchScr_Drv::getLastButton(TchEvent* evnt, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    esp_err_t err = i2cTch_set_mode(i2c_num, I2C_MODE_MASTER);
    if (err != ESP_OK) {
        return err;
    }
    return i2cTch_master_read_data(i2c_num, (uint8_t*)evnt, 1, 0x91, timeout);
}

uint32_t TchScr_Drv::getEvent(TchEvent* evnt, Vec2h* point, TickType_t timeout)
{
    static uint8_t _buff[4];
    if (!hw_init) return ESP_FAIL;

    esp_err_t err = i2cTch_set_mode(i2c_num, I2C_MODE_SLAVE);
    if (err != ESP_OK) {
        return 0;
    }

    portTickType ticks_rem = timeout;
    portTickType ticks_end = xTaskGetTickCount() + timeout;
    while (ticks_rem <= timeout) {
        uint8_t len = 0;
        i2cTch_slave_read_data(i2c_num, &len, 1, ticks_rem);
        if (len == 1 && evnt != NULL)
        {
            i2cTch_slave_read_data(i2c_num, (uint8_t*)evnt, 1, ticks_rem);
            return len;
        }
        else if (len == 4 && point != NULL)
        {
            i2cTch_slave_read_data(i2c_num, (uint8_t*)point, 4, ticks_rem);
            return len;
        }
        else if (len == 1 || len == 4)
        {
            i2cTch_slave_read_data(i2c_num, _buff, len, ticks_rem);
        }
        else
        {
            ESP_LOGD("TchScr", "Read Error");
            return 0;   // Read error
        }

        if (timeout != portMAX_DELAY) {
            ticks_rem = ticks_end - xTaskGetTickCount();
        }
    }
    
    return 0;   // Timeout
}

esp_err_t TchScr_Drv::setCalibration(TchCalib* calib, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    esp_err_t err = i2cTch_set_mode(i2c_num, I2C_MODE_MASTER);
    if (err != ESP_OK) {
        return err;
    }

    return i2cTch_master_send_data(i2c_num, (uint8_t*)calib, sizeof(TchCalib), 0x80, timeout);
}

esp_err_t TchScr_Drv::setThresholds(int16_t minPres, int16_t maxPres, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    esp_err_t err = i2cTch_set_mode(i2c_num, I2C_MODE_MASTER);
    if (err != ESP_OK) {
        return err;
    }
    int16_t data[2] = {minPres, maxPres};

    return i2cTch_master_send_data(i2c_num, (uint8_t*)data, 4, 0x90, timeout);
}

esp_err_t TchScr_Drv::setButton(Button* btn, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    i2cTch_set_mode(i2c_num, I2C_MODE_MASTER);

    return i2cTch_master_send_data(i2c_num, (uint8_t*)btn, sizeof(Button), 0xA0, timeout);
}

esp_err_t TchScr_Drv::setNotifications(bool touch, bool button, bool flipXY, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    i2cTch_set_mode(i2c_num, I2C_MODE_MASTER);
    uint8_t data = touch | (button << 1) | (flipXY << 2);

    return i2cTch_master_send_data(i2c_num, &data, 2, 0xB0, timeout);
}

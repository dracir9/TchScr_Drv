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

esp_err_t TchScr_Drv::getLastEvent(TchEvent* evnt, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    return i2cTch_master_read_data(i2c_num, (uint8_t*)evnt, 5, 0x81, timeout);
}

esp_err_t TchScr_Drv::getEvent(TchEvent* evnt, TickType_t timeout)
{
    static uint8_t _buff[5];
    if (!hw_init) return ESP_FAIL;

    int len =  i2cTch_slave_read_data(i2c_num, _buff, 5, timeout);

    if (len != 5)
        return ESP_ERR_INVALID_RESPONSE;

    evnt->id = _buff[0] & 0x1F;
    evnt->trigger = (TrgSrc)(_buff[0] >> 5);
    evnt->pos.x = ((int16_t*)&_buff[1])[0];
    evnt->pos.y = ((int16_t*)&_buff[1])[1];
    return ESP_OK;
}

esp_err_t TchScr_Drv::setCalibration(const TchCalib* calib, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    return i2cTch_master_send_data(i2c_num, (uint8_t*)calib, sizeof(TchCalib), 0x80, timeout);
}

esp_err_t TchScr_Drv::setThresholds(const uint16_t minPres, const uint16_t maxPres, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    uint16_t data[2] = {minPres, maxPres};

    return i2cTch_master_send_data(i2c_num, (uint8_t*)data, 4, 0x90, timeout);
}

esp_err_t TchScr_Drv::setButton(const Button* btn, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    return i2cTch_master_send_data(i2c_num, (uint8_t*)btn, sizeof(Button), 0xA0, timeout);
}

esp_err_t TchScr_Drv::setNotifications(bool touch, bool button, bool flipXY, uint16_t freq, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    float reloadVal = roundf(65536.0f - 24.5e6f/(12.0f * freq));
    if (reloadVal < 0.0f)
        freq = 0;
    else
        freq = reloadVal;
    uint8_t data[3] = {(uint8_t)(touch | (button << 1) | (flipXY << 2)), ((uint8_t*)&freq)[0], ((uint8_t*)&freq)[1]};

    return i2cTch_master_send_data(i2c_num, data, 3, 0xB0, timeout);
}

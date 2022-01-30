/**
 * @file TchScr_Drv.h
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

#ifndef TCHSCR_DRV_H
#define TCHSCR_DRV_H

#include "I2C_Drv.h"
#include "Vector.h"

struct TchCalib
{
    int16_t dx;
    int16_t rx_min;
    int16_t rx_max;

    int16_t dy;
    int16_t ry_min;
    int16_t ry_max;
};

enum TrgSrc : uint8_t
{
    IDLE,
    PRESS,
    HOLD_STRT,
    HOLD_END,
    RELEASE
};

struct Button
{
    uint8_t id;
    bool    enPressEv:1;
    bool    enHoldEv:1;
    bool    enReleaseEv:1;
    uint8_t holdTime:5;
	int16_t xmin;
	int16_t xmax;
	int16_t ymin;
	int16_t ymax;
};

struct TchEvent
{
    uint8_t id:5;
    TrgSrc event:3;
};

class TchScr_Drv
{
private:
    bool hw_init;
    i2c_port_t i2c_num;
public:
    TchScr_Drv(i2c_port_t i2c_num);
    ~TchScr_Drv();

    esp_err_t begin(i2c_mode_t mode, gpio_num_t sda, gpio_num_t scl, uint16_t addr, uint32_t freq = 255000);
    esp_err_t getLastPoint(Vec2h* point, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t getLastButton(TchEvent* evnt, TickType_t timeout = pdMS_TO_TICKS(1000));
    uint32_t getEvent(TchEvent* evnt, Vec2h* point, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t setCalibration(TchCalib* calib, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t setThresholds(uint16_t minPres, uint16_t maxPres, uint16_t freq, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t setButton(Button* btn, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t setNotifications(bool touch, bool button, bool flipXY, TickType_t timeout = pdMS_TO_TICKS(1000));
};

#endif

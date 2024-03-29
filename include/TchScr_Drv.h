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

#include <driver/uart.h>
#include <driver/gpio.h>
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
    HOLD_TICK,
    HOLD_END,
    RELEASE
};

struct Button
{
    Button() : enPressEv {false}, enHoldEv {false}, enHoldTickEv {false}, enReleaseEv {false} {}
    uint8_t id = 0;
    bool    enPressEv:1;
    bool    enHoldEv:1;
    bool    enHoldTickEv:1;
    bool    enReleaseEv:1;
    int8_t holdTime = 0;
	int16_t xmin = 0;
	int16_t xmax = 0;
	int16_t ymin = 0;
	int16_t ymax = 0;
};

struct TchEvent
{
    uint8_t id:5;
    TrgSrc trigger:3;
    Vec2h pos;
};

class TchScr_Drv
{
private:
    bool hw_init;
    uart_port_t uart_num;
    QueueHandle_t uart_queue;

public:
    TchScr_Drv(uart_port_t uart_num);
    ~TchScr_Drv();

    esp_err_t begin(gpio_num_t tx, gpio_num_t rx, int buf_size=512, int queu_size=32, uint32_t freq=250000);
    esp_err_t getLastEvent(TchEvent* evnt, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t getEvent(TchEvent* evnt, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t setCalibration(const TchCalib* calib, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t setThresholds(const uint16_t minPres, const uint16_t maxPres, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t setButton(const Button* btn, TickType_t timeout = pdMS_TO_TICKS(1000));
    esp_err_t setNotifications(bool touch, bool button, bool flipXY, uint16_t freq = 260, TickType_t timeout = pdMS_TO_TICKS(1000));
};

#endif

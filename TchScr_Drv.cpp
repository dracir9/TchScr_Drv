/**
 * @file TchScr_Drv.cpp
 * @author Ricard Bitriá Ribes (https://github.com/dracir9)
 * @brief Touch screen I2C driver
 * @version 0.1
 * @date 2021-08-10
 * 
 * @copyright Copyright (c) 2021 Ricard Bitriá Ribes
 * 
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

TchScr_Drv::TchScr_Drv(uart_port_t uart_num) :
    uart_num (uart_num)
{
}

TchScr_Drv::~TchScr_Drv()
{
    uart_driver_delete(uart_num);
}

esp_err_t TchScr_Drv::begin(gpio_num_t tx, gpio_num_t rx, int buf_size, int queu_size, uint32_t freq)
{
    // Configure uart initialization structure
    uart_config_t uart_config = {};
        uart_config.baud_rate = (int)freq;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_APB;

    //Install UART driver, and get the queue.
    uart_driver_install(uart_num, buf_size, buf_size, queu_size, &uart_queue, 0);
    uart_param_config(uart_num, &uart_config);

    //Set UART pins
    uart_set_pin(uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    hw_init = true;
    return ESP_OK;
}

esp_err_t TchScr_Drv::getLastEvent(TchEvent* evnt, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    // Send read command
    const char cmd[] = {0x80};
    if (uart_write_bytes(uart_num, cmd, 1) < 0)
        return ESP_FAIL;

    // Wait for data received
    uart_event_t event;
    TickType_t ticks_start = xTaskGetTickCount();
    TickType_t wait_time = 0;
    do {
        xQueueReceive(uart_queue, (void *)&event, timeout - wait_time);
        wait_time = xTaskGetTickCount() - ticks_start;
    } while (wait_time < timeout && event.type != UART_DATA);
    
    if (wait_time >= timeout)
    {
        return ESP_ERR_TIMEOUT;
    }
    else
    {
        if (event.size != 5) return ESP_ERR_INVALID_RESPONSE;
        uart_read_bytes(uart_num, (uint8_t*)evnt, 5, portMAX_DELAY);
    }

    return ESP_OK;
}

esp_err_t TchScr_Drv::getEvent(TchEvent* evnt, TickType_t timeout)
{
    uint8_t _buff[6] = {0};
    if (!hw_init) return ESP_FAIL;

    // Check if there is data avaiilable
    int bytesRead = uart_read_bytes(uart_num, _buff, sizeof(_buff), portMAX_DELAY);

    if (bytesRead < sizeof(_buff))
    {
        // Wait to receive data
        uart_event_t event;
        TickType_t ticks_start = xTaskGetTickCount();
        TickType_t wait_time = 0;
        do {
            xQueueReceive(uart_queue, (void *)&event, timeout - wait_time);
            wait_time = xTaskGetTickCount() - ticks_start;

            if (event.type == UART_DATA)
                bytesRead += uart_read_bytes(uart_num, &_buff[bytesRead], sizeof(_buff) - bytesRead, portMAX_DELAY);
            
        } while (wait_time < timeout && bytesRead < sizeof(_buff));
        
        if (wait_time >= timeout)
            return ESP_ERR_TIMEOUT;
    }

    if (_buff[0] != 5) return ESP_ERR_INVALID_RESPONSE;

    evnt->id = _buff[1] & 0x1F;
    evnt->trigger = (TrgSrc)(_buff[1] >> 5);
    evnt->pos.x = ((int16_t*)&_buff[2])[0];
    evnt->pos.y = ((int16_t*)&_buff[2])[1];

    return ESP_OK;
}

esp_err_t TchScr_Drv::setCalibration(const TchCalib* calib, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    const char data[sizeof(TchCalib) + 1] = {
        0x01,
        ((uint8_t*)calib)[0],
        ((uint8_t*)calib)[1],
        ((uint8_t*)calib)[2],
        ((uint8_t*)calib)[3],
        ((uint8_t*)calib)[4],
        ((uint8_t*)calib)[5],
        ((uint8_t*)calib)[6],
        ((uint8_t*)calib)[7],
        ((uint8_t*)calib)[8],
        ((uint8_t*)calib)[9],
        ((uint8_t*)calib)[10],
        ((uint8_t*)calib)[11]
        };

    if (uart_write_bytes(uart_num, data, sizeof(data)) >= 0) 
        return ESP_OK;
    else
        return ESP_FAIL;
}

esp_err_t TchScr_Drv::setThresholds(const uint16_t minPres, const uint16_t maxPres, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    const char data[] = {
        0x02,
        ((uint8_t*)&minPres)[0],
        ((uint8_t*)&minPres)[1],
        ((uint8_t*)&maxPres)[0],
        ((uint8_t*)&maxPres)[1]
        };

    if (uart_write_bytes(uart_num, data, sizeof(data)) >= 0) 
        return ESP_OK;
    else
        return ESP_FAIL;
}

esp_err_t TchScr_Drv::setButton(const Button* btn, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    const char data[sizeof(Button) + 1] = {
        0x03,
        ((uint8_t*)btn)[0],
        ((uint8_t*)btn)[1],
        ((uint8_t*)btn)[2],
        ((uint8_t*)btn)[3],
        ((uint8_t*)btn)[4],
        ((uint8_t*)btn)[5],
        ((uint8_t*)btn)[6],
        ((uint8_t*)btn)[7],
        ((uint8_t*)btn)[8],
        ((uint8_t*)btn)[9],
        ((uint8_t*)btn)[10],
        ((uint8_t*)btn)[11]
    };

    if (uart_write_bytes(uart_num, data, sizeof(data)) >= 0) 
        return ESP_OK;
    else
        return ESP_FAIL;
}

esp_err_t TchScr_Drv::setNotifications(bool touch, bool button, bool flipXY, uint16_t freq, TickType_t timeout)
{
    if (!hw_init) return ESP_FAIL;

    float reloadVal = roundf(65536.0f - 24.5e6f/(12.0f * freq));
    if (reloadVal < 0.0f)
        freq = 0;
    else
        freq = reloadVal;
    uint8_t data[4] = {0x04, (uint8_t)(touch | (button << 1) | (flipXY << 2)), ((uint8_t*)&freq)[0], ((uint8_t*)&freq)[1]};

    if (uart_write_bytes(uart_num, data, sizeof(data)) >= 0) 
        return ESP_OK;
    else
        return ESP_FAIL;
}

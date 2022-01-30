/**
 * @file I2C_Drv.c
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

#include "I2C_Drv.h"
#include "string.h"
#include "esp_log.h"
#include "driver/periph_ctrl.h"
#include "soc/i2c_struct.h"
#include "soc/i2c_reg.h"

static const char *I2C_TAG = "i2c";

#define I2C_CHECK(a, str, ret)  if(!(a)) {                                             \
        ESP_LOGE(I2C_TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                   \
        return (ret);                                                                  \
        }

//#define I2C_ENTER_CRITICAL_ISR(mux)    portENTER_CRITICAL_ISR(mux)
//#define I2C_EXIT_CRITICAL_ISR(mux)     portEXIT_CRITICAL_ISR(mux)
#define I2C_ENTER_CRITICAL(mux)        portENTER_CRITICAL(mux)
#define I2C_EXIT_CRITICAL(mux)         portEXIT_CRITICAL(mux)

#define I2C_DRIVER_ERR_STR             "i2c driver install error"
#define I2C_DRIVER_MALLOC_ERR_STR      "i2c driver malloc error"
#define I2C_NUM_ERROR_STR              "i2c number error"
//#define I2C_TIMEING_VAL_ERR_STR        "i2c timing value error"
#define I2C_ADDR_ERROR_STR             "i2c null address error"
#define I2C_DRIVER_NOT_INSTALL_ERR_STR "i2c driver not installed"
#define I2C_SLAVE_BUFFER_LEN_ERR_STR   "i2c buffer size too small for slave mode"
//#define I2C_EVT_QUEUE_ERR_STR          "i2c evt queue error"
#define I2C_SEM_ERR_STR                "i2c semaphore error"
#define I2C_BUF_ERR_STR                "i2c ringbuffer error"
#define I2C_MASTER_MODE_ERR_STR        "Only allowed in master mode"
#define I2C_MODE_SLAVE_ERR_STR         "Only allowed in slave mode"
//#define I2C_CMD_MALLOC_ERR_STR         "i2c command link malloc error"
//#define I2C_TRANS_MODE_ERR_STR         "i2c trans mode error"
#define I2C_MODE_ERR_STR               "i2c mode error"
//#define I2C_SDA_IO_ERR_STR             "sda gpio number error"
//#define I2C_SCL_IO_ERR_STR             "scl gpio number error"
//#define I2C_SCL_SDA_EQUAL_ERR_STR      "scl and sda gpio numbers are the same"
//#define I2C_CMD_LINK_INIT_ERR_STR      "i2c command link error"
//#define I2C_GPIO_PULLUP_ERR_STR        "this i2c pin does not support internal pull-up"
//#define I2C_ACK_TYPE_ERR_STR           "i2c ack type error"
//#define I2C_DATA_LEN_ERR_STR           "i2c data read length error"
//#define I2C_PSRAM_BUFFER_WARN_STR      "Using buffer allocated from psram"
//#define I2C_LOCK_ERR_STR               "Power lock creation error"
#define I2C_CLK_FLAG_ERR_STR           "i2c clock choice is invalid, please check flag and frequency"
#define I2C_FIFO_FULL_THRESH_VAL       (28)
#define I2C_FIFO_EMPTY_THRESH_VAL      (5)
//#define I2C_IO_INIT_LEVEL              (1)
#define I2C_CMD_ALIVE_INTERVAL_TICK    (1000 / portTICK_PERIOD_MS)
#define I2C_EVT_QUEUE_LEN              (1)
#define I2C_SLAVE_TIMEOUT_DEFAULT      (32000)     /* I2C slave timeout value, APB clock cycle number */
#define I2C_SLAVE_SDA_SAMPLE_DEFAULT   (10)        /* I2C slave sample time after scl positive edge default value */
#define I2C_SLAVE_SDA_HOLD_DEFAULT     (10)        /* I2C slave hold time after scl negative edge default value */
//#define I2C_MASTER_TOUT_CNUM_DEFAULT   (8)         /* I2C master timeout cycle number of I2C clock, after which the timeout interrupt will be triggered */
#define I2C_ACKERR_CNT_MAX             (10)
#define I2C_FILTER_CYC_NUM_DEF         (7)         /* The number of apb cycles filtered by default*/
#define I2C_CLR_BUS_SCL_NUM            (9)
#define I2C_CLR_BUS_HALF_PERIOD_US     (5)

#define I2C_RXFIFO_FULL_INT_ENA_M  (BIT(0))

#define I2C_TRANS_COMPLETE_INT_ENA_M  (BIT(7))

#define I2C_LL_INTR_MASK          (0x3fff) /*!< I2C all interrupt bitmap */

/// Use the highest speed that is available for the clock source picked by clk_flags
#define I2C_CLK_FREQ_MAX                  (-1)

// I2C master TX interrupt bitmap
#define I2C_LL_MASTER_TX_INT          (I2C_ACK_ERR_INT_ENA_M|I2C_TIME_OUT_INT_ENA_M|I2C_TRANS_COMPLETE_INT_ENA_M|I2C_ARBITRATION_LOST_INT_ENA_M|I2C_END_DETECT_INT_ENA_M)
// I2C master RX interrupt bitmap
#define I2C_LL_MASTER_RX_INT          (I2C_ACK_ERR_INT_ENA_M|I2C_TIME_OUT_INT_ENA_M|I2C_TRANS_COMPLETE_INT_ENA_M|I2C_ARBITRATION_LOST_INT_ENA_M|I2C_END_DETECT_INT_ENA_M)
// I2C slave TX interrupt bitmap
#define I2C_LL_SLAVE_TX_INT           (I2C_TXFIFO_EMPTY_INT_ENA_M)
// I2C slave RX interrupt bitmap
#define I2C_LL_SLAVE_RX_INT           (I2C_RXFIFO_FULL_INT_ENA_M | I2C_TRANS_COMPLETE_INT_ENA_M)
// I2C source clock frequency
#define I2C_LL_CLK_SRC_FREQ(src_clk)  APB_CLK_FREQ

/**
 * @brief Enums
 * 
 */
typedef enum {
    I2C_INTR_EVENT_ERR,
    I2C_INTR_EVENT_ARBIT_LOST,   /*!< I2C arbition lost event */
    I2C_INTR_EVENT_NACK,         /*!< I2C NACK event */
    I2C_INTR_EVENT_TOUT,         /*!< I2C time out event */
    I2C_INTR_EVENT_END_DET,      /*!< I2C end detected event */
    I2C_INTR_EVENT_TRANS_DONE,   /*!< I2C trans done event */
    I2C_INTR_EVENT_RXFIFO_FULL,  /*!< I2C rxfifo full event */
    I2C_INTR_EVENT_TXFIFO_EMPTY, /*!< I2C txfifo empty event */
} i2c_intr_event_t;

typedef enum {
    I2C_STATUS_READ,      /*!< read status for current master command */
    I2C_STATUS_WRITE,     /*!< write status for current master command */
    I2C_STATUS_IDLE,      /*!< idle status for current master command */
    I2C_STATUS_ACK_ERROR, /*!< ack error status for current master command */
    I2C_STATUS_DONE,      /*!< I2C command done */
    I2C_STATUS_TIMEOUT,   /*!< I2C bus status error, and operation timeout */
    I2C_STATUS_ARB_LOST
} i2c_status_t;

typedef enum {
    I2C_CMD_EVT_ALIVE,
    I2C_CMD_EVT_DONE
} i2c_cmd_evt_t;

/**
 * @brief Structures
 * 
 */
typedef struct {
    i2c_dev_t *dev;      /*!< I2C hal context */
    portMUX_TYPE spinlock;
    bool hw_enabled;
    int scl_io_num;
    int sda_io_num;
} i2c_context_t;

typedef union {
    struct {
        uint32_t byte_num:    8,
                 ack_en:      1,
                 ack_exp:     1,
                 ack_val:     1,
                 op_code:     3,
                 reserved14: 17,
                 done:        1;
    };
    uint32_t val;
} i2c_hw_cmd_t;

typedef struct {
    int i2c_num;                     /*!< I2C port number */
    i2c_mode_t mode;                 /*!< I2C mode, master or slave */
    intr_handle_t intr_handle;       /*!< I2C interrupt handle*/
    i2c_status_t status;             /*!< record current command status, for master mode */
    uint8_t data_buf[32];            /*!< a buffer to store i2c fifo data */

    QueueHandle_t cmd_evt_queue;     /*!< I2C command event queue */

    xSemaphoreHandle cmd_mux;        /*!< semaphore to lock command process */

    xSemaphoreHandle slv_rx_mux;     /*!< slave rx buffer mux */
    size_t rx_buf_length;            /*!< rx buffer length */
    RingbufHandle_t rx_ring_buf;     /*!< rx ringbuffer handler of slave mode */
} i2c_obj_t;

/**
 * @brief Global variables
 * 
 */

static i2c_context_t i2c_context[I2C_NUM_MAX] = {
    {
    .dev = &I2C0,
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
    .hw_enabled = false,
    },
    {
    .dev = &I2C1,
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
    .hw_enabled = false,
    },
};

static const i2c_hw_cmd_t start_cmd = {
    .ack_en = 0,
    .ack_exp = 0,
    .ack_val = 0,
    .byte_num = 0,
    .op_code = I2C_CMD_RESTART
};

static const i2c_hw_cmd_t stop_cmd = {
    .ack_en = 0,
    .ack_exp = 0,
    .ack_val = 0,
    .byte_num = 0,
    .op_code = I2C_CMD_STOP
};

static i2c_obj_t *p_i2c_obj[I2C_NUM_MAX] = {0};

static i2c_intr_event_t eventTrace[1024];
static uint16_t eventPtr = 0;
static uint16_t readEventPtr = 0;

bool i2cTch_getEvent(int* event)
{
    if (readEventPtr < eventPtr)
    {
        *event = eventTrace[readEventPtr++];
        return true;
    }
    return false;
}

/**
 * @brief Functions
 * 
 */

/**
 * @brief  Write the I2C hardware txFIFO
 *
 * @param  hw Beginning address of the peripheral registers
 * @param  ptr Pointer to data buffer
 * @param  len Amount of data needs to be writen
 *
 * @return None.
 */
static inline void i2cTch_write_txfifo(i2c_dev_t *hw, uint8_t *ptr, uint8_t len)
{
    uint32_t fifo_addr = (hw == &I2C0) ? 0x6001301c : 0x6002701c;
    for(int i = 0; i < len; i++) {
        WRITE_PERI_REG(fifo_addr, ptr[i]);
    }
}

/**
 * @brief  Read the I2C hardware rxFIFO
 *
 * @param  hw Beginning address of the peripheral registers
 * @param  ptr Pointer to data buffer
 * @param  len Amount of data needs read
 *
 * @return None
 */
static inline void i2cTch_read_rxfifo(i2c_dev_t *hw, uint8_t *ptr, uint8_t len)
{
    for(int i = 0; i < len; i++) {
        ptr[i] = hw->fifo_data.data;
    }
}

/**
 * @brief Write I2C hardware command register
 *
 * @param  hw Beginning address of the peripheral registers
 * @param  cmd I2C hardware command
 * @param  cmd_idx The index of the command register, should be less than 16
 *
 * @return None
 */
static inline void i2cTch_write_cmd_reg(i2c_dev_t *hw, i2c_hw_cmd_t cmd, int cmd_idx)
{
    hw->command[cmd_idx].val = cmd.val;
}

/**
 * @brief  Start I2C transfer
 *
 * @param  hw Beginning address of the peripheral registers
 *
 * @return None
 */
static inline void i2cTch_trans_start(i2c_dev_t *hw)
{
    hw->ctr.trans_start = 1;
}

/**
 * @brief  Enable I2C master TX interrupt
 *
 * @param  hw Beginning address of the peripheral registers
 *
 * @return None
 */
static inline void i2cTch_master_enable_tx_it(i2c_dev_t *hw)
{
    hw->int_clr.val = ~0;
    hw->int_ena.val =  I2C_LL_MASTER_TX_INT;
}

/**
 * @brief  Enable I2C master RX interrupt
 *
 * @param  hw Beginning address of the peripheral registers
 *
 * @return None
 */
static inline void i2cTch_master_enable_rx_it(i2c_dev_t *hw)
{
    hw->int_clr.val = ~0;
    hw->int_ena.val = I2C_LL_MASTER_RX_INT;
}

/**
 * @brief Get the state of the I2C module
 * 
 * @param hw Beginning address of the peripheral registers
 * @return true if the module is in a transaction
 * @return false if the module is idle
 */
bool i2cTch_is_bus_busy(i2c_dev_t *hw)
{
    return hw->status_reg.bus_busy;
}

/**
 * @brief Disable selected interrupts
 * 
 * @param hw Beginning address of the peripheral registers
 * @param mask Mask of interrupts to be disabled
 */
void i2cTch_disable_intr_mask(i2c_dev_t *hw, uint32_t mask)
{
    hw->int_ena.val &= (~mask);
}

void i2cTch_clr_intsts_mask(i2c_dev_t *hw, uint32_t mask)
{
    hw->int_clr.val = mask;
}

void i2cTch_set_slave_addr(i2c_dev_t *hw, uint16_t slave_addr, bool addr_10bit_en)
{
    hw->slave_addr.addr = slave_addr;
    hw->slave_addr.en_10bit = addr_10bit_en;
}

void i2cTch_set_rxfifo_full_thr(i2c_dev_t *hw, uint8_t full_thr)
{
    hw->fifo_conf.rx_fifo_full_thrhd = full_thr;
}

void i2cTch_set_txfifo_empty_thr(i2c_dev_t *hw, uint8_t empty_thr)
{
    hw->fifo_conf.tx_fifo_empty_thrhd = empty_thr;
}

void i2cTch_drv_set_stop_timing(i2c_dev_t *hw, int stop_setup, int stop_hold)
{
    hw->scl_stop_setup.time = stop_setup;
    hw->scl_stop_hold.time = stop_hold;
}

void i2cTch_drv_set_start_timing(i2c_dev_t *hw, int start_setup, int start_hold)
{
    hw->scl_rstart_setup.time = start_setup;
    hw->scl_start_hold.time = start_hold;
}

void i2cTch_set_sda_timing(i2c_dev_t *hw, int sda_sample, int sda_hold)
{
    hw->sda_hold.time = sda_hold;
    hw->sda_sample.time = sda_sample;
}

void i2cTch_set_tout(i2c_dev_t *hw, int tout_num)
{
    hw->timeout.tout = tout_num;
}

void i2cTch_enable_slave_rx_it(i2c_dev_t *hw)
{
    hw->int_ena.val |= I2C_LL_SLAVE_RX_INT;
}

void i2cTch_set_filter(i2c_dev_t *hw, uint8_t filter_num)
{
    if(filter_num > 0) {
        hw->scl_filter_cfg.thres = filter_num;
        hw->sda_filter_cfg.thres = filter_num;
        hw->scl_filter_cfg.en = 1;
        hw->sda_filter_cfg.en = 1;
    } else {
        hw->scl_filter_cfg.en = 0;
        hw->sda_filter_cfg.en = 0;
    }
}

void i2cTch_set_scl_timing(i2c_dev_t *hw, int hight_period, int low_period)
{
    hw->scl_low_period.period = low_period;
    hw->scl_high_period.period = hight_period;
}

void i2cTch_set_bus_timing(i2c_dev_t *hw, int scl_freq)
{
    uint32_t sclk = I2C_LL_CLK_SRC_FREQ(0);                       // Fixed to 80MHz in ESP32
    uint32_t scl_hw_freq = (scl_freq == I2C_CLK_FREQ_MAX) ? (sclk / 20) : (uint32_t)scl_freq; // FREQ_MAX use the highest freq of the chosen clk.

    uint32_t half_cycle = sclk / scl_hw_freq / 2;

    //scl period
    hw->scl_low_period.period = half_cycle;
    hw->scl_high_period.period = half_cycle;
    //sda sample
    hw->sda_hold.time = half_cycle / 2;
    hw->sda_sample.time = half_cycle / 2;
    //setup
    hw->scl_rstart_setup.time = half_cycle;
    hw->scl_stop_setup.time = half_cycle;
    //hold
    hw->scl_start_hold.time = half_cycle;
    hw->scl_stop_hold.time = half_cycle;
    hw->timeout.tout = half_cycle * 20;   //default we set the timeout value to 10 bus cycles.
}

void i2cTch_init(i2c_dev_t *hw, int i2c_num, i2c_mode_t mode)
{
    // Init
    typeof(hw->ctr) ctrl_reg;
    ctrl_reg.val = 0;
    ctrl_reg.ms_mode = (mode == I2C_MODE_MASTER) ? 1 : 0;  // Set mode
    ctrl_reg.sda_force_out = 1;
    ctrl_reg.scl_force_out = 1;
    hw->ctr.val = ctrl_reg.val;
    hw->fifo_conf.fifo_addr_cfg_en = 0;
    // Use fifo mode
    hw->fifo_conf.nonfifo_en = 0;
    // MSB data mode
    hw->ctr.tx_lsb_first = I2C_DATA_MODE_MSB_FIRST;
    hw->ctr.rx_lsb_first = I2C_DATA_MODE_MSB_FIRST;
    // Reset TX fifo
    hw->fifo_conf.tx_fifo_rst = 1;  // Black magic with registers
    hw->fifo_conf.tx_fifo_rst = 0;
    // Reset RX fifo
    hw->fifo_conf.rx_fifo_rst = 1;
    hw->fifo_conf.rx_fifo_rst = 0;
}

void i2cTch_master_handle_tx_event(i2c_dev_t *hw, i2c_intr_event_t *event)
{
    //i2c_ll_master_get_event(hw, event);
    typeof(hw->int_status) int_sts = hw->int_status;
    if (int_sts.arbitration_lost) {
        *event = I2C_INTR_EVENT_ARBIT_LOST;
    } else if (int_sts.ack_err) {
        *event = I2C_INTR_EVENT_NACK;
    } else if (int_sts.time_out) {
        *event = I2C_INTR_EVENT_TOUT;
    } else if (int_sts.end_detect) {
        *event = I2C_INTR_EVENT_END_DET;
    } else if (int_sts.trans_complete) {
        *event = I2C_INTR_EVENT_TRANS_DONE;
    } else {
        *event = I2C_INTR_EVENT_ERR;
    }

    if ((*event < I2C_INTR_EVENT_END_DET) ||
        (*event == I2C_INTR_EVENT_TRANS_DONE)) {
        //i2c_ll_master_disable_tx_it(hw);
        hw->int_ena.val &= (~I2C_LL_MASTER_TX_INT);
        //i2c_ll_master_clr_tx_it(hw);
        hw->int_clr.val = I2C_LL_MASTER_TX_INT;
    } else if (*event == I2C_INTR_EVENT_END_DET) {
        //i2c_ll_master_clr_tx_it(hw);
        hw->int_clr.val = I2C_LL_MASTER_TX_INT;
    }
}

void i2cTch_master_handle_rx_event(i2c_dev_t *hw, i2c_intr_event_t *event)
{
    //i2c_ll_master_get_event(hw, event);
    typeof(hw->int_status) int_sts = hw->int_status;
    if (int_sts.arbitration_lost) {
        *event = I2C_INTR_EVENT_ARBIT_LOST;
    } else if (int_sts.ack_err) {
        *event = I2C_INTR_EVENT_NACK;
    } else if (int_sts.time_out) {
        *event = I2C_INTR_EVENT_TOUT;
    } else if (int_sts.end_detect) {
        *event = I2C_INTR_EVENT_END_DET;
    } else if (int_sts.trans_complete) {
        *event = I2C_INTR_EVENT_TRANS_DONE;
    } else {
        *event = I2C_INTR_EVENT_ERR;
    }

    if ((*event < I2C_INTR_EVENT_END_DET) ||
        (*event == I2C_INTR_EVENT_TRANS_DONE)) {
        //i2c_ll_master_disable_rx_it(hw);
        hw->int_ena.val &= (~I2C_LL_MASTER_RX_INT);
        //i2c_ll_master_clr_rx_it(hw);
        hw->int_clr.val = I2C_LL_MASTER_RX_INT;
    } else if (*event == I2C_INTR_EVENT_END_DET) {
        //i2c_ll_master_clr_rx_it(hw);
        hw->int_clr.val = I2C_LL_MASTER_RX_INT;
    }
}

void i2cTch_slave_handle_event(i2c_dev_t *hw, i2c_intr_event_t *event)
{
    typeof(hw->int_status) int_sts = hw->int_status;
    if (int_sts.tx_fifo_empty) {
        *event = I2C_INTR_EVENT_TXFIFO_EMPTY;
    } else if (int_sts.trans_complete) {
        *event = I2C_INTR_EVENT_TRANS_DONE;
    } else if (int_sts.rx_fifo_full) {
        *event = I2C_INTR_EVENT_RXFIFO_FULL;
    } else {
        *event = I2C_INTR_EVENT_ERR;
    }
}

static void i2cTch_hw_disable(i2c_port_t i2c_num)
{
    I2C_ENTER_CRITICAL(&(i2c_context[i2c_num].spinlock));
    if (i2c_context[i2c_num].hw_enabled != false) {
        periph_module_disable(PERIPH_I2C0_MODULE + i2c_num);
        i2c_context[i2c_num].hw_enabled = false;
    }
    I2C_EXIT_CRITICAL(&(i2c_context[i2c_num].spinlock));
}

static void i2cTch_hw_enable(i2c_port_t i2c_num)
{
    I2C_ENTER_CRITICAL(&(i2c_context[i2c_num].spinlock));
    if (i2c_context[i2c_num].hw_enabled != true) {
        periph_module_enable(PERIPH_I2C0_MODULE + i2c_num);
        i2c_context[i2c_num].hw_enabled = true;
    }
    I2C_EXIT_CRITICAL(&(i2c_context[i2c_num].spinlock));
}

/* Some slave device will die by accident and keep the SDA in low level,
 * in this case, master should send several clock to make the slave release the bus.
 * Slave mode of ESP32 might also get in wrong state that held the SDA low,
 * in this case, master device could send a stop signal to make esp32 slave release the bus.
 **/
static esp_err_t i2cTch_master_clear_bus(i2c_port_t i2c_num)
{
    const int delay_cnt = 2000; // Counter value to get raw delay time
    int i = 0;
    int scl_io = i2c_context[i2c_num].scl_io_num;
    int sda_io = i2c_context[i2c_num].sda_io_num;
    gpio_set_direction(scl_io, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(sda_io, GPIO_MODE_INPUT_OUTPUT_OD);
    // If a SLAVE device was in a read operation when the bus was interrupted, the SLAVE device is controlling SDA.
    // The only bit during the 9 clock cycles of a READ byte the MASTER(ESP32) is guaranteed control over is during the ACK bit
    // period. If the slave is sending a stream of ZERO bytes, it will only release SDA during the ACK bit period.
    // So, this reset code needs to synchronize the bit stream with, Either, the ACK bit, Or a 1 bit to correctly generate
    // a STOP condition.
    gpio_set_level(scl_io, 0);
    gpio_set_level(sda_io, 1);
    //esp_rom_delay_us(scl_half_period);
    for(int j = 0; j < delay_cnt; j++);        // Small delay
    while (!gpio_get_level(sda_io) && (i++ < I2C_CLR_BUS_SCL_NUM)) {
        gpio_set_level(scl_io, 1);
        //esp_rom_delay_us(scl_half_period);
        for(int j = 0; j < delay_cnt; j++);
        gpio_set_level(scl_io, 0);
        //esp_rom_delay_us(scl_half_period);
        for(int j = 0; j < delay_cnt; j++);
    }
    gpio_set_level(sda_io, 0); // setup for STOP
    gpio_set_level(scl_io, 1);
    //esp_rom_delay_us(scl_half_period);
    for(int j = 0; j < delay_cnt; j++);
    gpio_set_level(sda_io, 1); // STOP, SDA low -> high while SCL is HIGH
    i2c_set_pin(i2c_num, sda_io, scl_io, 1, 1, I2C_MODE_MASTER);
    return ESP_OK;
}

/**if the power and SDA/SCL wires are in proper condition, everything works find with reading the slave.
 * If we remove the power supply for the slave during I2C is reading, or directly connect SDA or SCL to ground,
 * this would cause the I2C FSM get stuck in wrong state, all we can do is to reset the I2C hardware in this case.
 **/
static esp_err_t i2cTch_hw_fsm_reset(i2c_port_t i2c_num)
{
    //i2c_hal_get_scl_timing(i2c_context[i2c_num].dev, &scl_high_period, &scl_low_period);
    int scl_high_period = i2c_context[i2c_num].dev->scl_high_period.period;
    int scl_low_period = i2c_context[i2c_num].dev->scl_low_period.period;

    //i2c_hal_get_start_timing(i2c_context[i2c_num].dev, &scl_rstart_setup, &scl_start_hold);
    int scl_rstart_setup = i2c_context[i2c_num].dev->scl_rstart_setup.time;
    int scl_start_hold = i2c_context[i2c_num].dev->scl_start_hold.time;

    //i2c_hal_get_stop_timing(i2c_context[i2c_num].dev, &scl_stop_setup, &scl_stop_hold);
    int scl_stop_setup = i2c_context[i2c_num].dev->scl_stop_setup.time;
    int scl_stop_hold = i2c_context[i2c_num].dev->scl_stop_hold.time;

    //i2c_hal_get_sda_timing(i2c_context[i2c_num].dev, &sda_sample, &sda_hold);
    int sda_hold = i2c_context[i2c_num].dev->sda_hold.time;
    int sda_sample = i2c_context[i2c_num].dev->sda_sample.time;

    //i2c_hal_get_tout(i2c_context[i2c_num].dev, &timeout);
    int timeout = i2c_context[i2c_num].dev->timeout.tout;
    
    //i2c_hal_get_filter(i2c_context[i2c_num].dev, &filter_cfg);
    uint8_t filter_cfg = i2c_context[i2c_num].dev->sda_filter_cfg.thres;

    //to reset the I2C hw module, we need re-enable the hw
    i2cTch_hw_disable(i2c_num);
    i2cTch_master_clear_bus(i2c_num);
    i2cTch_hw_enable(i2c_num);

    i2cTch_init(i2c_context[i2c_num].dev, i2c_num, I2C_MODE_MASTER);
    i2cTch_disable_intr_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);
    i2cTch_clr_intsts_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);
    i2cTch_set_scl_timing(i2c_context[i2c_num].dev, scl_high_period, scl_low_period);
    i2cTch_drv_set_start_timing(i2c_context[i2c_num].dev, scl_rstart_setup, scl_start_hold);
    i2cTch_drv_set_stop_timing(i2c_context[i2c_num].dev, scl_stop_setup, scl_stop_hold);
    i2cTch_set_sda_timing(i2c_context[i2c_num].dev, sda_sample, sda_hold);
    i2cTch_set_tout(i2c_context[i2c_num].dev, timeout);
    i2cTch_set_filter(i2c_context[i2c_num].dev, filter_cfg);
    return ESP_OK;
}

/**
 * @brief Setup the I2C module with the selected parameters
 * 
 * @param i2c_num I2C port to configure
 * @param i2c_conf Configuration structure with the necessary parameters
 * @return esp_err_t Returns ESP_OK if successful, otherwhise it returns the error type
 */
esp_err_t i2cTch_init_params(i2c_port_t i2c_num, const i2c_full_config_t *i2c_conf)
{
    // Check parameters
    I2C_CHECK(i2c_num < I2C_NUM_MAX, I2C_NUM_ERROR_STR, ESP_ERR_INVALID_ARG);
    I2C_CHECK(i2c_conf != NULL, I2C_ADDR_ERROR_STR, ESP_ERR_INVALID_ARG);
    I2C_CHECK(i2c_conf->mode < I2C_MODE_MAX, I2C_MODE_ERR_STR, ESP_ERR_INVALID_ARG);

    esp_err_t ret = i2c_set_pin(i2c_num, i2c_conf->sda_io_num, i2c_conf->scl_io_num,
                                i2c_conf->sda_pullup_en, i2c_conf->scl_pullup_en, i2c_conf->mode);
    if (ret != ESP_OK) {
        return ret;
    }
    i2c_context[i2c_num].sda_io_num = i2c_conf->sda_io_num;
    i2c_context[i2c_num].scl_io_num = i2c_conf->scl_io_num;

    // Enable clock for I2C
    i2cTch_hw_enable(i2c_num);
    I2C_ENTER_CRITICAL(&(i2c_context[i2c_num].spinlock));
    // Disable I2C interrupts
    i2cTch_disable_intr_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);
    // Clear I2C interrupt pending flags
    i2cTch_clr_intsts_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);

    i2cTch_init(i2c_context[i2c_num].dev, i2c_num, i2c_conf->mode);
    //slave mode
    i2cTch_set_slave_addr(i2c_context[i2c_num].dev, i2c_conf->slave.slave_addr, i2c_conf->slave.addr_10bit_en);
    i2cTch_set_rxfifo_full_thr(i2c_context[i2c_num].dev, I2C_FIFO_FULL_THRESH_VAL);
    i2cTch_set_txfifo_empty_thr(i2c_context[i2c_num].dev, I2C_FIFO_EMPTY_THRESH_VAL);
    //set timing for data
    i2cTch_set_sda_timing(i2c_context[i2c_num].dev, I2C_SLAVE_SDA_SAMPLE_DEFAULT, I2C_SLAVE_SDA_HOLD_DEFAULT);
    i2cTch_set_tout(i2c_context[i2c_num].dev, I2C_SLAVE_TIMEOUT_DEFAULT);
    i2cTch_enable_slave_rx_it(i2c_context[i2c_num].dev);
    //i2c_hal_update_config(i2c_context[i2c_num].dev);      // Not needed for ESP32

    //Default, we enable hardware filter
    i2cTch_set_filter(i2c_context[i2c_num].dev, I2C_FILTER_CYC_NUM_DEF);
    i2cTch_set_bus_timing(i2c_context[i2c_num].dev, i2c_conf->master.clk_speed);

    I2C_EXIT_CRITICAL(&(i2c_context[i2c_num].spinlock));
    return ESP_OK;
}

static void IRAM_ATTR i2cTch_isr_handler(void *arg)
{
    i2c_obj_t *p_i2c = (i2c_obj_t *) arg;
    int i2c_num = p_i2c->i2c_num;
    i2c_intr_event_t evt_type = I2C_INTR_EVENT_ERR;
    portBASE_TYPE HPTaskAwoken = pdFALSE;
    if (p_i2c->mode == I2C_MODE_MASTER) {
        i2c_cmd_evt_t evt = I2C_CMD_EVT_ALIVE;
        if (p_i2c->status == I2C_STATUS_WRITE) {
            i2cTch_master_handle_tx_event(i2c_context[i2c_num].dev, &evt_type);
        } else if (p_i2c->status == I2C_STATUS_READ) {
            i2cTch_master_handle_rx_event(i2c_context[i2c_num].dev, &evt_type);
        }
        if (evt_type == I2C_INTR_EVENT_NACK) {
            p_i2c_obj[i2c_num]->status = I2C_STATUS_ACK_ERROR;
            evt = I2C_CMD_EVT_DONE;
        } else if (evt_type == I2C_INTR_EVENT_TOUT) {
            p_i2c_obj[i2c_num]->status = I2C_STATUS_TIMEOUT;
            evt = I2C_CMD_EVT_DONE;
        } else if (evt_type == I2C_INTR_EVENT_ARBIT_LOST) {
            p_i2c_obj[i2c_num]->status = I2C_STATUS_ARB_LOST;
            evt = I2C_CMD_EVT_DONE;
        } else if (evt_type == I2C_INTR_EVENT_END_DET) {
            assert(false);
        } else if (evt_type == I2C_INTR_EVENT_TRANS_DONE) {
            if (p_i2c->status != I2C_STATUS_ACK_ERROR && p_i2c->status != I2C_STATUS_IDLE) {
                evt = I2C_CMD_EVT_DONE;
            }
        }
        
        xQueueSendFromISR(p_i2c->cmd_evt_queue, &evt, &HPTaskAwoken);
    } else {
        i2cTch_slave_handle_event(i2c_context[i2c_num].dev, &evt_type);
        if (evt_type == I2C_INTR_EVENT_TRANS_DONE || evt_type == I2C_INTR_EVENT_RXFIFO_FULL) {
            uint32_t rx_fifo_cnt = i2c_context[i2c_num].dev->status_reg.rx_fifo_cnt;
            
            i2cTch_read_rxfifo(i2c_context[i2c_num].dev, p_i2c->data_buf, rx_fifo_cnt);

            xRingbufferSendFromISR(p_i2c->rx_ring_buf, p_i2c->data_buf, rx_fifo_cnt, &HPTaskAwoken);

            // Clear interrupt flags
            i2c_context[i2c_num].dev->int_clr.val = I2C_LL_SLAVE_RX_INT;
        } else if (evt_type == I2C_INTR_EVENT_TXFIFO_EMPTY) {   // Slave sending NOT USED!
            // Ensure we never reach here

            // Disable and clear interrupt flags
            i2c_context[i2c_num].dev->int_ena.val &= (~I2C_LL_SLAVE_TX_INT);
            i2c_context[i2c_num].dev->int_clr.val = I2C_LL_SLAVE_TX_INT;
        }
    }
    if (eventPtr < 1024 && evt_type != I2C_INTR_EVENT_TRANS_DONE)
        eventTrace[eventPtr++] = evt_type;
    // Check here if there is a high-priority task needs to be switched.
    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Create data structures, data queues and semaphores
 * 
 * @param i2c_num I2C port to configure
 * @param mode Initial driver mode
 * @param slv_rx_buf_len Buffer lenght in bytes for data received as slave
 * @return esp_err_t 
 */
esp_err_t i2cTch_install_driver(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len)
{
    I2C_CHECK(i2c_num < I2C_NUM_MAX, I2C_NUM_ERROR_STR, ESP_ERR_INVALID_ARG);
    I2C_CHECK(slv_rx_buf_len > 32, I2C_SLAVE_BUFFER_LEN_ERR_STR, ESP_ERR_INVALID_ARG);

    if (p_i2c_obj[i2c_num] == NULL) {
        p_i2c_obj[i2c_num] = (i2c_obj_t *) calloc(1, sizeof(i2c_obj_t));

        if (p_i2c_obj[i2c_num] == NULL) {
            ESP_LOGE(I2C_TAG, I2C_DRIVER_MALLOC_ERR_STR);
            return ESP_FAIL;
        }
        i2c_obj_t *p_i2c = p_i2c_obj[i2c_num];
        p_i2c->i2c_num = i2c_num;
        p_i2c->mode = mode;
        p_i2c->status = I2C_STATUS_IDLE;

        // Slave specific
        if (slv_rx_buf_len > 0) {
            p_i2c->rx_ring_buf = xRingbufferCreate(slv_rx_buf_len, RINGBUF_TYPE_BYTEBUF);
            if (p_i2c->rx_ring_buf == NULL) {
                ESP_LOGE(I2C_TAG, I2C_BUF_ERR_STR);
                goto err;
            }
            p_i2c->rx_buf_length = slv_rx_buf_len;
        } else {
            p_i2c->rx_ring_buf = NULL;
            p_i2c->rx_buf_length = 0;
        }
        p_i2c->slv_rx_mux = xSemaphoreCreateMutex();
        if (p_i2c->slv_rx_mux == NULL) {
            ESP_LOGE(I2C_TAG, I2C_SEM_ERR_STR);
            goto err;
        }
        // Master specific
        //semaphore to sync sending process, because we only have 32 bytes for hardware fifo.
        p_i2c->cmd_mux = xSemaphoreCreateMutex();

        p_i2c->cmd_evt_queue = xQueueCreate(I2C_EVT_QUEUE_LEN, sizeof(i2c_cmd_evt_t));

        if (p_i2c->cmd_mux == NULL || p_i2c->cmd_evt_queue == NULL) {
            ESP_LOGE(I2C_TAG, I2C_SEM_ERR_STR);
            goto err;
        }

    } else {
        ESP_LOGE(I2C_TAG, I2C_DRIVER_ERR_STR);
        return ESP_FAIL;
    }
    i2cTch_hw_enable(i2c_num);
    //Disable I2C interrupt.
    i2cTch_disable_intr_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);
    i2cTch_clr_intsts_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);
    //hook isr handler
    i2c_isr_register(i2c_num, i2cTch_isr_handler, p_i2c_obj[i2c_num], 0, &p_i2c_obj[i2c_num]->intr_handle);
    //Enable I2C slave rx interrupt
    if (mode == I2C_MODE_SLAVE) {
        i2cTch_enable_slave_rx_it(i2c_context[i2c_num].dev);
    }
    return ESP_OK;

err:
    //Some error has happened. Free/destroy all allocated things and return ESP_FAIL.
    if (p_i2c_obj[i2c_num]) {
        if (p_i2c_obj[i2c_num]->cmd_evt_queue) {
            vQueueDelete(p_i2c_obj[i2c_num]->cmd_evt_queue);
            p_i2c_obj[i2c_num]->cmd_evt_queue = NULL;
        }
        if (p_i2c_obj[i2c_num]->cmd_mux) {
            vSemaphoreDelete(p_i2c_obj[i2c_num]->cmd_mux);
        }
        if (p_i2c_obj[i2c_num]->slv_rx_mux) {
            vSemaphoreDelete(p_i2c_obj[i2c_num]->slv_rx_mux);
        }
    }
    free(p_i2c_obj[i2c_num]);
    p_i2c_obj[i2c_num] = NULL;
    return ESP_FAIL;
}

esp_err_t i2cTch_delete_driver(i2c_port_t i2c_num)
{
    I2C_CHECK(i2c_num < I2C_NUM_MAX, I2C_NUM_ERROR_STR, ESP_ERR_INVALID_ARG);
    I2C_CHECK(p_i2c_obj[i2c_num] != NULL, I2C_DRIVER_ERR_STR, ESP_FAIL);

    i2c_obj_t *p_i2c = p_i2c_obj[i2c_num];
    i2cTch_disable_intr_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);
    esp_intr_free(p_i2c->intr_handle);
    p_i2c->intr_handle = NULL;

    if (p_i2c->cmd_mux) {
        xSemaphoreTake(p_i2c->cmd_mux, portMAX_DELAY);
        vSemaphoreDelete(p_i2c->cmd_mux);
    }
    if (p_i2c_obj[i2c_num]->cmd_evt_queue) {
        vQueueDelete(p_i2c_obj[i2c_num]->cmd_evt_queue);
        p_i2c_obj[i2c_num]->cmd_evt_queue = NULL;
    }
    if (p_i2c->slv_rx_mux) {
        vSemaphoreDelete(p_i2c->slv_rx_mux);
    }

    if (p_i2c->rx_ring_buf) {
        vRingbufferDelete(p_i2c->rx_ring_buf);
        p_i2c->rx_ring_buf = NULL;
        p_i2c->rx_buf_length = 0;
    }

    free(p_i2c_obj[i2c_num]);
    p_i2c_obj[i2c_num] = NULL;

    i2cTch_hw_disable(i2c_num);
    return ESP_OK;
}

int i2cTch_slave_read_data(i2c_port_t i2c_num, uint8_t *data, size_t max_size, TickType_t ticks_to_wait)
{
    I2C_CHECK(( i2c_num < I2C_NUM_MAX ), I2C_NUM_ERROR_STR, ESP_FAIL);
    I2C_CHECK(p_i2c_obj[i2c_num] != NULL, I2C_DRIVER_ERR_STR, ESP_FAIL);
    I2C_CHECK((data != NULL), I2C_ADDR_ERROR_STR, ESP_FAIL);
    I2C_CHECK(p_i2c_obj[i2c_num]->mode == I2C_MODE_SLAVE, I2C_MODE_SLAVE_ERR_STR, ESP_FAIL);

    size_t size = 0;
    size_t size_rem = max_size;
    i2c_obj_t *p_i2c = p_i2c_obj[i2c_num];
    if (xSemaphoreTake(p_i2c->slv_rx_mux, ticks_to_wait) == pdFALSE) {
        return 0;
    }
    portTickType ticks_rem = ticks_to_wait;
    portTickType ticks_end = xTaskGetTickCount() + ticks_to_wait;
    I2C_ENTER_CRITICAL(&(i2c_context[i2c_num].spinlock));
    i2cTch_enable_slave_rx_it(i2c_context[i2c_num].dev);
    I2C_EXIT_CRITICAL(&(i2c_context[i2c_num].spinlock));

    while (size_rem && ticks_rem <= ticks_to_wait) {
        uint8_t *pdata = (uint8_t *) xRingbufferReceiveUpTo(p_i2c->rx_ring_buf, &size, ticks_to_wait, size_rem);
        if (pdata && size > 0) {
            memcpy(data, pdata, size);
            vRingbufferReturnItem(p_i2c->rx_ring_buf, pdata);
            data += size;
            size_rem -= size;
        }
        if (ticks_to_wait != portMAX_DELAY) {
            ticks_rem = ticks_end - xTaskGetTickCount();
        }
    }

    xSemaphoreGive(p_i2c->slv_rx_mux);
    return max_size - size_rem;
}

esp_err_t i2cTch_master_send_data(i2c_port_t i2c_num, uint8_t* data, uint16_t len, uint8_t address, TickType_t ticks_to_wait)
{
    I2C_CHECK(( i2c_num < I2C_NUM_MAX ), I2C_NUM_ERROR_STR, ESP_ERR_INVALID_ARG);
    I2C_CHECK(p_i2c_obj[i2c_num] != NULL, I2C_DRIVER_NOT_INSTALL_ERR_STR, ESP_ERR_INVALID_STATE);
    I2C_CHECK(p_i2c_obj[i2c_num]->mode == I2C_MODE_MASTER, I2C_MASTER_MODE_ERR_STR, ESP_ERR_INVALID_STATE);
    I2C_CHECK(len <= 32, "A maximum of 32 bytes can be sent at a time", ESP_ERR_INVALID_ARG);

    // Sometimes when the FSM get stuck, the ACK_ERR interrupt will occur endlessly until we reset the FSM and clear bus.
    esp_err_t ret = ESP_FAIL;
    i2c_obj_t *p_i2c = p_i2c_obj[i2c_num];
    TickType_t ticks_start = xTaskGetTickCount();
    BaseType_t res = xSemaphoreTake(p_i2c->cmd_mux, ticks_to_wait);
    if (res == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    i2c_hw_cmd_t data_cmd = {
        .ack_en = 1,
        .ack_exp = 0,
        .ack_val = 0,
        .byte_num = len + 1,
        .op_code = I2C_CMD_WRITE,
    };
    uint8_t ackRetry = 0;
    do
    {
        xQueueReset(p_i2c->cmd_evt_queue);
        if (p_i2c->status == I2C_STATUS_TIMEOUT
                || i2cTch_is_bus_busy(i2c_context[i2c_num].dev)) {
            i2cTch_hw_fsm_reset(i2c_num);  // Reset port if error detected
        }

        p_i2c->status = I2C_STATUS_IDLE;
        i2c_reset_tx_fifo(i2c_num);
        i2c_reset_rx_fifo(i2c_num);
        // These two interrupts some times can not be cleared when the FSM gets stuck.
        // so we disable them when these two interrupt occurs and re-enable them here.
        i2cTch_disable_intr_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);
        i2cTch_clr_intsts_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);

        p_i2c->status = I2C_STATUS_WRITE;
        // Write commands
        I2C_ENTER_CRITICAL(&(i2c_context[i2c_num].spinlock));
        
        i2cTch_write_txfifo(i2c_context[i2c_num].dev, &address, 1);
        i2cTch_write_txfifo(i2c_context[i2c_num].dev, data, len);
        i2cTch_write_cmd_reg(i2c_context[i2c_num].dev, start_cmd, 0);
        i2cTch_write_cmd_reg(i2c_context[i2c_num].dev, data_cmd, 1);
        i2cTch_write_cmd_reg(i2c_context[i2c_num].dev, stop_cmd, 2);
        i2cTch_master_enable_tx_it(i2c_context[i2c_num].dev);
        i2cTch_trans_start(i2c_context[i2c_num].dev);

        I2C_EXIT_CRITICAL(&(i2c_context[i2c_num].spinlock));

        // Wait event bits
        i2c_cmd_evt_t evt;
        while (1) {
            TickType_t wait_time = xTaskGetTickCount();
            if (wait_time - ticks_start > ticks_to_wait) { // out of time
                wait_time = I2C_CMD_ALIVE_INTERVAL_TICK;
            } else {
                wait_time = ticks_to_wait - (wait_time - ticks_start);
                if (wait_time < I2C_CMD_ALIVE_INTERVAL_TICK) {
                    wait_time = I2C_CMD_ALIVE_INTERVAL_TICK;
                }
            }
            // In master mode, since we don't have an interrupt to detect bus error or FSM state, what we do here is to make
            // sure the interrupt mechanism for master mode is still working.
            // If the command sending is not finished and there is no interrupt any more, the bus is probably dead caused by external noise.
            portBASE_TYPE evt_res = xQueueReceive(p_i2c->cmd_evt_queue, &evt, wait_time);
            if (evt_res == pdTRUE) {
                if (evt == I2C_CMD_EVT_DONE) {
                    if (p_i2c->status == I2C_STATUS_TIMEOUT) {
                        // If the I2C slave are powered off or the SDA/SCL are connected to ground, for example,
                        // I2C hw FSM would get stuck in wrong state, we have to reset the I2C module in this case.
                        i2cTch_hw_fsm_reset(i2c_num);
                        ret = ESP_ERR_TIMEOUT;
                    } else if (p_i2c->status == I2C_STATUS_ACK_ERROR) {
                        ret = ESP_ERR_INVALID_RESPONSE;
                        ackRetry++;
                    } else if (p_i2c->status == I2C_STATUS_ARB_LOST) {
                        ret = ESP_FAIL;
                    } else {
                        ret = ESP_OK;
                    }
                    break;
                }
                if (evt == I2C_CMD_EVT_ALIVE) {
                }
            } else {
                ret = ESP_ERR_TIMEOUT;
                // If the I2C slave are powered off or the SDA/SCL are connected to ground, for example,
                // I2C hw FSM would get stuck in wrong state, we have to reset the I2C module in this case.
                i2cTch_hw_fsm_reset(i2c_num);
                
                break;
            }
        }
    } while (p_i2c->status == I2C_STATUS_ACK_ERROR && ackRetry < 20);
    p_i2c->status = I2C_STATUS_DONE;
    xSemaphoreGive(p_i2c->cmd_mux);
    return ret;
}

esp_err_t i2cTch_master_read_data(i2c_port_t i2c_num, uint8_t* data, uint16_t len, uint8_t address, TickType_t ticks_to_wait)
{
    I2C_CHECK(( i2c_num < I2C_NUM_MAX ), I2C_NUM_ERROR_STR, ESP_ERR_INVALID_ARG);
    I2C_CHECK(p_i2c_obj[i2c_num] != NULL, I2C_DRIVER_NOT_INSTALL_ERR_STR, ESP_ERR_INVALID_STATE);
    I2C_CHECK(p_i2c_obj[i2c_num]->mode == I2C_MODE_MASTER, I2C_MASTER_MODE_ERR_STR, ESP_ERR_INVALID_STATE);
    I2C_CHECK(len <= 32, "A maximum of 32 bytes can be sent at a time", ESP_ERR_INVALID_ARG);

    // Sometimes when the FSM get stuck, the ACK_ERR interrupt will occur endlessly until we reset the FSM and clear bus.
    esp_err_t ret = ESP_FAIL;
    i2c_obj_t *p_i2c = p_i2c_obj[i2c_num];
    TickType_t ticks_start = xTaskGetTickCount();
    BaseType_t res = xSemaphoreTake(p_i2c->cmd_mux, ticks_to_wait);
    if (res == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    xQueueReset(p_i2c->cmd_evt_queue);
    if (p_i2c->status == I2C_STATUS_TIMEOUT
            || i2cTch_is_bus_busy(i2c_context[i2c_num].dev)) {
        i2cTch_hw_fsm_reset(i2c_num);  // Reset port if error detected
    }
    //i2c_reset_tx_fifo(i2c_num);
    //i2c_reset_rx_fifo(i2c_num);
    p_i2c->status = I2C_STATUS_IDLE;
    i2c_reset_tx_fifo(i2c_num);
    i2c_reset_rx_fifo(i2c_num);
    // These two interrupts some times can not be cleared when the FSM gets stuck.
    // so we disable them when these two interrupt occurs and re-enable them here.
    i2cTch_disable_intr_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);
    i2cTch_clr_intsts_mask(i2c_context[i2c_num].dev, I2C_LL_INTR_MASK);


    i2c_hw_cmd_t address_cmd = {
        .ack_en = 1,
        .ack_exp = 0,
        .ack_val = 0,
        .byte_num = 1,
        .op_code = I2C_CMD_WRITE,
    };
    i2c_hw_cmd_t read_cmd1 = {
        .ack_en = 0,
        .ack_exp = 0,
        .ack_val = 0,
        .byte_num = len-1,
        .op_code = I2C_CMD_READ,
    };
    i2c_hw_cmd_t read_cmd2 = {
        .ack_en = 0,
        .ack_exp = 0,
        .ack_val = 1,
        .byte_num = 1,
        .op_code = I2C_CMD_READ,
    };

    p_i2c->status = I2C_STATUS_READ;
    address |= 0x01;    // Ensure write bit is set
    // Write commands
    I2C_ENTER_CRITICAL(&(i2c_context[i2c_num].spinlock));
    
    i2cTch_write_txfifo(i2c_context[i2c_num].dev, &address, 1);
    i2cTch_write_cmd_reg(i2c_context[i2c_num].dev, start_cmd, 0);
    i2cTch_write_cmd_reg(i2c_context[i2c_num].dev, address_cmd, 1);
    i2cTch_write_cmd_reg(i2c_context[i2c_num].dev, read_cmd1, 2);
    i2cTch_write_cmd_reg(i2c_context[i2c_num].dev, read_cmd2, 3);
    i2cTch_write_cmd_reg(i2c_context[i2c_num].dev, stop_cmd, 4);
    i2cTch_master_enable_rx_it(i2c_context[i2c_num].dev);
    i2cTch_trans_start(i2c_context[i2c_num].dev);

    I2C_EXIT_CRITICAL(&(i2c_context[i2c_num].spinlock));


    // Wait event bits
    i2c_cmd_evt_t evt;
    while (1) {
        TickType_t wait_time = xTaskGetTickCount();
        if (wait_time - ticks_start > ticks_to_wait) { // out of time
            wait_time = I2C_CMD_ALIVE_INTERVAL_TICK;
        } else {
            wait_time = ticks_to_wait - (wait_time - ticks_start);
            if (wait_time < I2C_CMD_ALIVE_INTERVAL_TICK) {
                wait_time = I2C_CMD_ALIVE_INTERVAL_TICK;
            }
        }
        // In master mode, since we don't have an interrupt to detective bus error or FSM state, what we do here is to make
        // sure the interrupt mechanism for master mode is still working.
        // If the command sending is not finished and there is no interrupt any more, the bus is probably dead caused by external noise.
        portBASE_TYPE evt_res = xQueueReceive(p_i2c->cmd_evt_queue, &evt, wait_time);
        if (evt_res == pdTRUE) {
            if (evt == I2C_CMD_EVT_DONE) {
                if (p_i2c->status == I2C_STATUS_TIMEOUT) {
                    // If the I2C slave are powered off or the SDA/SCL are connected to ground, for example,
                    // I2C hw FSM would get stuck in wrong state, we have to reset the I2C module in this case.
                    i2cTch_hw_fsm_reset(i2c_num);
                    ret = ESP_ERR_TIMEOUT;
                } else if (p_i2c->status == I2C_STATUS_ACK_ERROR) {
                    ret = ESP_ERR_INVALID_RESPONSE;
                } else if (p_i2c->status == I2C_STATUS_ARB_LOST) {
                    ret = ESP_FAIL;
                } else {
                    I2C_ENTER_CRITICAL(&(i2c_context[i2c_num].spinlock));

                    uint32_t rx_fifo_cnt = i2c_context[i2c_num].dev->status_reg.rx_fifo_cnt;
                    if (rx_fifo_cnt != len) {
                        ret = ESP_ERR_INVALID_SIZE;
                    } else {
                        i2cTch_read_rxfifo(i2c_context[i2c_num].dev, data, len);
                        ret = ESP_OK;
                    }
                    
                    I2C_EXIT_CRITICAL(&(i2c_context[i2c_num].spinlock));
                }
                break;
            }
            if (evt == I2C_CMD_EVT_ALIVE) {
            }
        } else {
            ret = ESP_ERR_TIMEOUT;
            // If the I2C slave are powered off or the SDA/SCL are connected to ground, for example,
            // I2C hw FSM would get stuck in wrong state, we have to reset the I2C module in this case.
            i2cTch_hw_fsm_reset(i2c_num);
            
            break;
        }
    }
    p_i2c->status = I2C_STATUS_DONE;
    xSemaphoreGive(p_i2c->cmd_mux);
    return ret;
}

i2c_mode_t i2cTch_get_mode(i2c_port_t i2c_num)
{
    return p_i2c_obj[i2c_num]->mode;
}

esp_err_t i2cTch_set_mode(i2c_port_t i2c_num, i2c_mode_t mode)
{
    I2C_CHECK(i2c_num < I2C_NUM_MAX, I2C_NUM_ERROR_STR, ESP_ERR_INVALID_ARG);

    if (p_i2c_obj[i2c_num]->mode == mode) return ESP_OK;

    // Flush buffer
    if (mode == I2C_MODE_MASTER)
    {
        uint8_t dat;
        while (i2cTch_slave_read_data(i2c_num, &dat, 1, 0));
    }

    I2C_ENTER_CRITICAL(&(i2c_context[i2c_num].spinlock));
    p_i2c_obj[i2c_num]->mode = mode;
    i2c_context[i2c_num].dev->ctr.ms_mode = (mode == I2C_MODE_MASTER) ? 1 : 0;
    I2C_EXIT_CRITICAL(&(i2c_context[i2c_num].spinlock));
    return ESP_OK;
}

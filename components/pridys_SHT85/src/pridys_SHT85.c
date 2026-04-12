/**
 * Implementation file of pridys component for SHT85 handling.
 * 
 */


/* ====== Standard includes ====== */
#include <stdio.h>

/* ====== Framework includes ====== */
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ====== Required includes ====== */
#include "driver/i2c_master.h"

/* ====== Proprietary includes ====== */
#include "pridys_SHT85.h"

/* ====== Defines ====== */
#define CONFIG_I2C_MASTER_SCL       22
#define CONFIG_I2C_MASTER_SDA       21

#define CONFIG_I2C_MASTER_FREQUENCY 10000

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define SHT85_ADDRESS               0x44
#define SHT85_GET_SERIAL            0x3682

/* ====== Global variable declaration ====== */
static const char *TAG = "pridys_sht85";

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;


/* ====== Private functions ====== */


/* ====== Public functions ====== */
void read_sht85_serial(i2c_master_dev_handle_t *dev_handle)
{
    uint8_t data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t cmd[2] = {0x36, 0x82};

    i2c_master_transmit(*dev_handle, cmd, 2, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_master_transmit_receive(*dev_handle, cmd, 2, data, 6, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "serial byte 1: %X", data[0]);
    ESP_LOGI(TAG, "serial byte 2: %X", data[1]);
    ESP_LOGI(TAG, "CRC 1: %X", data[2]);

    ESP_LOGI(TAG, "serial byte 3: %X", data[3]);
    ESP_LOGI(TAG, "serial byte 4: %X", data[4]);
    ESP_LOGI(TAG, "CRC 2: %X", data[5]);
}

void pSHT85_F_init(void)
{
    ESP_LOGI(TAG, "Init called.\n");


    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT85_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    
    read_sht85_serial(&dev_handle);


    ESP_LOGI(TAG, "Init finished.\n");
}

void pSHT85_F_de_init(void)
{
    ESP_LOGI(TAG, "Deinitialization called.\n");


    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));


    ESP_LOGI(TAG, "Deinitialization finished.\n");
}

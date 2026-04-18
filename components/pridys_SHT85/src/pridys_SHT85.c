/**
 * Implementation file of pridys component for SHT85 handling.
 * pridys_SHT85.c
 */


/* ====== Standard includes ====== */
#include <stdio.h>
#include <string.h>

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
#define SHT85_GET_SERIAL_36         0x36u
#define SHT85_GET_SERIAL_82         0x82u

/* ====== Global variable declaration ====== */
static const char *module_tag = "pridys_sht85";

typedef struct{
    uint8_t     four_byte_serial[4];
    uint8_t     two_byte_CRC[2];
} serial_SHT85_st;

static serial_SHT85_st serial_SHT85_s;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

/* ====== Function prototypes ====== */
void pSHT85_LF_init_i2c_for_sensor_sht85(void);
void pSHT85_LF_init_sensor_sht85_variables(void);
void pSHT85_LF_read_sensor_sht85_serial(void);
void pSHT85_LF_log_sensor_sht85_serial(void);

/* ====== Private functions ====== */

/**
 * Local function to initialize I2C bus and communication to SHT85 sennsor. 
 */
void pSHT85_LF_init_i2c_for_sensor_sht85(void)
{
    esp_err_t esp_err = ESP_OK;
    
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err = i2c_new_master_bus(&bus_config, &bus_handle);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT85_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);

    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_LF_init_i2c_for_sht85() encountered a problem "
            "while initializing the I2C bus for communication with SHT85 sensor");
    }
}

/**
 * Local function to initialize variables for SHT85 handling, such as serial number. 
 * Initializes global struct serial_SHT85_s containing serial number and CRCs to 0xFFu.
 */
void pSHT85_LF_init_sensor_sht85_variables(void)
{
    memset(serial_SHT85_s.four_byte_serial, 0xFFu, 4);
    memset(serial_SHT85_s.two_byte_CRC, 0xFFu, 2);
}

/**
 * Local function to read serial number from SHT85 via I2C, 
 * storing it to a global variable (struct serial_SHT85_s);
 */
void pSHT85_LF_read_sensor_sht85_serial(void)
{
    /* Empty buffer for 6 bytes: 
        2 bytes serial, CRC; another 2 bytes serial => 4 bytes serial */
    uint8_t data[6] = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

    /* As per the datasheet, command 'Get Serial Number' 
        is 0x36 82 */
    uint8_t cmd[2] = {SHT85_GET_SERIAL_36, SHT85_GET_SERIAL_82};

    i2c_master_transmit(dev_handle, cmd, 2, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_master_transmit_receive(dev_handle, cmd, 2, data, 6, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(50));

    //TODO Add error check of framework functions above "i2c_master_transmit_receive()"
    // and "i2c_master_transmit()"

    //Store into global variable (struct serial_SHT85_s) by copying the values
    serial_SHT85_s.four_byte_serial[0] = data[0];   //straight forward, copy first byte
    serial_SHT85_s.four_byte_serial[1] = data[1];   //straight forward, copy second byte
    serial_SHT85_s.four_byte_serial[2] = data[3];   //use fourth byte, because third is CRC 1
    serial_SHT85_s.four_byte_serial[3] = data[4];   //use successor byte, fifth byte

    //Store CRC into variables in struct; CRCs are received in third and sixth byte
    serial_SHT85_s.two_byte_CRC[0] = data[2];   //straight forward, copy first byte
    serial_SHT85_s.two_byte_CRC[1] = data[5];   //straight forward, copy second byte
}

/**
 * Local function to log serial number of SHT85 via UART to console. 
 * Reading global serial_SHT85_s and printing the content via ESP_LOGI().
 */
void pSHT85_LF_log_sensor_sht85_serial(void)
{
    ESP_LOGI(module_tag, "Sensor SHT85 serial number:");
    ESP_LOGI(module_tag, "\t%X", serial_SHT85_s.four_byte_serial[0]);
    ESP_LOGI(module_tag, "\t%X", serial_SHT85_s.four_byte_serial[1]);
    ESP_LOGI(module_tag, "\t%X", serial_SHT85_s.four_byte_serial[2]);
    ESP_LOGI(module_tag, "\t%X", serial_SHT85_s.four_byte_serial[3]);
}

/* ====== Public functions ====== */

/* For documentation of public functions,
    please refer to function prototypes in header file. */

void read_sht85_serial(i2c_master_dev_handle_t *dev_handle)
{
    uint8_t data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t cmd[2] = {0x36, 0x82};

    i2c_master_transmit(*dev_handle, cmd, 2, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_master_transmit_receive(*dev_handle, cmd, 2, data, 6, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(module_tag, "serial byte 1: %X", data[0]);
    ESP_LOGI(module_tag, "serial byte 2: %X", data[1]);
    ESP_LOGI(module_tag, "CRC 1: %X", data[2]);

    ESP_LOGI(module_tag, "serial byte 3: %X", data[3]);
    ESP_LOGI(module_tag, "serial byte 4: %X", data[4]);
    ESP_LOGI(module_tag, "CRC 2: %X", data[5]);
}

void pSHT85_F_init(void)
{
    ESP_LOGI(module_tag, "Init called.\n");

    pSHT85_LF_init_i2c_for_sensor_sht85();

    pSHT85_LF_init_sensor_sht85_variables();

    pSHT85_LF_log_sensor_sht85_serial(); //experimental call to check behaviour of the module
    //expected behaviour is to see 0xFF in console and later correct serial number

    pSHT85_LF_read_sensor_sht85_serial();

    pSHT85_LF_log_sensor_sht85_serial();

    ESP_LOGI(module_tag, "Init finished.\n");
}

void pSHT85_F_de_init(void)
{
    ESP_LOGI(module_tag, "Deinitialization called.\n");


    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));


    ESP_LOGI(module_tag, "Deinitialization finished.\n");
}

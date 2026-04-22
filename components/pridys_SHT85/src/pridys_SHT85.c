/**
 * Implementation file of pridys component for SHT85 handling.
 * pridys_SHT85.c
 */


/* ======================== Standard includes ======================== */
#include <stdio.h>
#include <string.h>

/* ======================== Framework includes ======================= */
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ======================== Required includes ======================== */
#include "driver/i2c_master.h"

/* ======================== Proprietary includes ===================== */
#include "pridys_SHT85.h"

/* ======================== Defines ================================== */
#define CONFIG_I2C_MASTER_SCL           22
#define CONFIG_I2C_MASTER_SDA           21

#define CONFIG_I2C_MASTER_FREQUENCY     10000

#define I2C_MASTER_SDA_IO               CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_SCL_IO               CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_NUM                  I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ              CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE       0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE       0                           /*!< I2C master doesn't need buffer */

/** 1000ms timeout for an I2C frame. */
#define I2C_MASTER_TIMEOUT_MS           1000

/** 50ms delay after sending/receiving frames via I2C.
 * Recommended to let the system process previous communication action. 
 */
#define I2C_MASTER_POST_FRAME_DELAY     50

#define SHT85_ADDRESS                   0x44    //Sensor SHT85 I2C bus address as per the datasheet
#define SHT85_GET_SERIAL_36             0x36u   //First byte of command get serial
#define SHT85_GET_SERIAL_82             0x82u   //Second byte of command to get serial number from sensor SHT85

/* ======================== Global variable declaration ============== */
static const char *module_tag = "pridys_sht85";

typedef struct{
    uint8_t     serial_u8a[4];
    uint8_t     crc_received_u8a[2];
    uint8_t     crc_calculated_u8a[2];
} serial_SHT85_st;

static serial_SHT85_st serial_SHT85_s;

typedef struct{
    uint8_t     temperature_raw_u8a[2];
    uint16_t    temperature_raw_u16;
    float       temperature_f;
    uint8_t     humidity_raw_u8a[2];
    float       humidity_f;
    uint8_t     crc_received_u8a[2];
    uint8_t     crc_calculated_u8a[2];
} measurement_SHT85_st;

measurement_SHT85_st measurement_SHT85_s;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

/* ======================== Function prototypes ====================== */
static void pSHT85_LF_init_i2c_for_sensor_sht85(void);
static void pSHT85_LF_init_sensor_sht85_variables(void);
static void pSHT85_LF_read_sensor_sht85_serial(void);
static void pSHT85_LF_log_sensor_sht85_serial(void);
static uint8_t pSHT85_LF_calculate_CRC8_31(const uint8_t * data, size_t len);

/* ======================== Private functions ======================== */

/**
 * Local function to initialize I2C bus and communication to SHT85 sennsor. 
 */
static void pSHT85_LF_init_i2c_for_sensor_sht85(void)
{
    esp_err_t esp_err = ESP_FAIL;
    
    //Set up I2C bus as master, report error in case framework does not return ESP_OK
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err = i2c_new_master_bus(&bus_config, &bus_handle);
    
    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_LF_init_i2c_for_sensor_sht85() encountered a problem "
            "while initializing the I2C bus for communication with SHT85 sensor");
    }


    //Set up I2C device, sensor SHT85 in this case; Report error in case framework does not return ESP_OK
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT85_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);

    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_LF_init_i2c_for_sensor_sht85() encountered a problem "
            "while initializing the I2C bus for communication with SHT85 sensor");
    }

}

/**
 * Local function to initialize variables for SHT85 handling, such as serial number. 
 * Initializes global struct serial_SHT85_s containing serial number and CRCs to 0xFFu.
 */
static void pSHT85_LF_init_sensor_sht85_variables(void)
{
    memset(serial_SHT85_s.serial_u8a, 0xFFu, 4);
    memset(serial_SHT85_s.crc_received_u8a, 0xFFu, 2);
    memset(serial_SHT85_s.crc_calculated_u8a, 0xFFu, 2);

    memset(measurement_SHT85_s.temperature_raw_u8a, 0xFFu, 2);
    memset(measurement_SHT85_s.humidity_raw_u8a, 0xFFu, 2);
    memset(measurement_SHT85_s.crc_received_u8a, 0xFFu, 2);
    memset(measurement_SHT85_s.crc_calculated_u8a, 0xFFu, 2);

    measurement_SHT85_s.temperature_raw_u16 = 0xFFFFu;

    measurement_SHT85_s.temperature_f = NAN;
    measurement_SHT85_s.humidity_f = NAN;
}

/**
 * Local function to read serial number from SHT85 via I2C, 
 * storing it to a global variable (struct serial_SHT85_s);
 */
static void pSHT85_LF_read_sensor_sht85_serial(void)
{
    esp_err_t esp_err = ESP_FAIL;

    /* Empty buffer for 6 bytes: 
        2 bytes serial, CRC; another 2 bytes serial => 4 bytes serial */
    uint8_t i2c_receive_data[6] = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

    /* As per the datasheet, command 'Get Serial Number' 
        is 0x36 82 */
    uint8_t cmd[2] = {SHT85_GET_SERIAL_36, SHT85_GET_SERIAL_82};


#if 0 /* remove unnecessary first transmit call */
    esp_err = i2c_master_transmit(dev_handle, cmd, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    vTaskDelay(pdMS_TO_TICKS(I2C_MASTER_POST_FRAME_DELAY));

    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_LF_read_sensor_sht85_serial() encountered a problem "
            "while communicating with SHT85 sensor");
    }
#endif /* 0 */


    esp_err = i2c_master_transmit_receive(dev_handle, cmd, 2, i2c_receive_data, 6, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    vTaskDelay(pdMS_TO_TICKS(I2C_MASTER_POST_FRAME_DELAY));

    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_LF_read_sensor_sht85_serial() encountered a problem "
            "while communicating with SHT85 sensor");
    }


    //Store into global variable (struct serial_SHT85_s) by copying the values
    serial_SHT85_s.serial_u8a[0] = i2c_receive_data[0];   //straight forward, copy first byte
    serial_SHT85_s.serial_u8a[1] = i2c_receive_data[1];   //straight forward, copy second byte
    serial_SHT85_s.serial_u8a[2] = i2c_receive_data[3];   //use fourth byte, because third is CRC 1
    serial_SHT85_s.serial_u8a[3] = i2c_receive_data[4];   //use successor byte, fifth byte

    //Store CRC into variables in struct; CRCs are received in third and sixth byte
    serial_SHT85_s.crc_received_u8a[0] = i2c_receive_data[2];   //straight forward, copy first byte
    serial_SHT85_s.crc_received_u8a[1] = i2c_receive_data[5];   //straight forward, copy second byte

    //Calculate CRC based on received data and store into global variable struct
    serial_SHT85_s.crc_calculated_u8a[0] = pSHT85_LF_calculate_CRC8_31(&i2c_receive_data[0], 2);
    serial_SHT85_s.crc_calculated_u8a[1] = pSHT85_LF_calculate_CRC8_31(&i2c_receive_data[3], 2);


    //Compare received and calculated CRC and log status
    if( serial_SHT85_s.crc_received_u8a[0] == serial_SHT85_s.crc_calculated_u8a[0] )
    {

        if( serial_SHT85_s.crc_received_u8a[1] == serial_SHT85_s.crc_calculated_u8a[1] )
        {
            //Received and calculated CRCs match, nothing to do besides logging
            ESP_LOGI(module_tag, "While reading serial number, the received and the calculated CRC matched.");
        }
        else
        {
            //Index 1 of CRC comparison failed, log/report error
            ESP_LOGE(module_tag, "Function pSHT85_LF_read_sensor_sht85_serial() encountered a problem "
                "while checking CRC; CRC index 1 did not match");
            ESP_LOGI(module_tag, "CRC index 1 is for serial number word 2");
            ESP_LOGI(module_tag, "Received CRC: %X", serial_SHT85_s.crc_received_u8a[1]);
            ESP_LOGI(module_tag, "Calculated CRC: %X", serial_SHT85_s.crc_calculated_u8a[1]);
        }
        
    }
    else
    {
        //Index 0 of CRC comparison failed, log/report error
        ESP_LOGE(module_tag, "Function pSHT85_LF_read_sensor_sht85_serial() encountered a problem "
            "while checking CRC; CRC index 0 did not match");
        ESP_LOGI(module_tag, "CRC index 0 is for serial number word 1");
        ESP_LOGI(module_tag, "Received CRC: %X", serial_SHT85_s.crc_received_u8a[0]);
        ESP_LOGI(module_tag, "Calculated CRC: %X", serial_SHT85_s.crc_calculated_u8a[0]);
    }

}

/**
 * Local function to log serial number of SHT85 via UART to console. 
 * Reading global serial_SHT85_s and printing the content via ESP_LOGI().
 */
static void pSHT85_LF_log_sensor_sht85_serial(void)
{
    ESP_LOGI(module_tag, "Sensor SHT85 serial number:");
    ESP_LOGI(module_tag, "\t%X", serial_SHT85_s.serial_u8a[0]);
    ESP_LOGI(module_tag, "\t%X", serial_SHT85_s.serial_u8a[1]);
    ESP_LOGI(module_tag, "\t%X", serial_SHT85_s.serial_u8a[2]);
    ESP_LOGI(module_tag, "\t%X", serial_SHT85_s.serial_u8a[3]);
}

/**
 * Local function to calculate CRC for sensor SHT85. 
 * According to the datasheet, SHT85 uses a CRC with following specifications:
 * - Width: 8
 * - Polynomial: 0x31
 * - Initialization value: 0xFF 
 * - Reflect input: No
 * - Reflect output: No
 * - Final XOR: No
 * Example: CRC(0xBEEF) = 0x92
 * Another example: 2C 2E 55 A7 is the serial of a sensor SHT85
 *                  Received CRC is 70 for first and 7A for second byte
 *                  Calculated by this function and an online calculator is the same
 * Note: Function will return 0xFF as CRC in case of problem 
 * like null pointer as data or length set to zero. 
 */
static uint8_t pSHT85_LF_calculate_CRC8_31(const uint8_t * data, size_t len)
{
    uint8_t crc = 0xFF;

    if (data == NULL || len == 0)
    {
        //we have a problem here, do not do CRC calculation
        ///TODO Log error via ESP_LOGE()
    }
    else
    {
        //Data pointer and length is not null, proceed with CRC calculation

        for (size_t byte_index = 0; byte_index < len; byte_index++)
        {
            uint8_t byte = data[byte_index];
            crc = crc ^ byte;

            for (int bit_index = 0; bit_index < 8; bit_index++)
            {
                if( (crc & 0x80) == 0x80 )
                {
                    //MSB is set, shift left apply the polynomial with XOR
                    crc = (crc << 1) ^ 0x31;
                }
                else
                {
                    //MSB is not set, just shift left
                    crc = (crc << 1);
                }
            }
        }

    }

    return crc;
}

static void pSHT85_LF_plausibility_check(void)
{
    esp_err_t esp_err = ESP_FAIL;

    /* Empty buffer for 6 bytes: 
        2 bytes serial, CRC; another 2 bytes serial => 4 bytes serial */
    uint8_t i2c_receive_data[6] = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

    /* As per the datasheet, command 'Get Serial Number' 
        is 0x36 82 */
    uint8_t cmd[2] = {0x24u, 0x00u};


    //Soft Reset
    ///TODO Do the reset before even reading serial


    //Issue single shot measurement
    i2c_master_transmit(dev_handle, cmd, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    //Wait minimum 15ms
    vTaskDelay(pdMS_TO_TICKS(I2C_MASTER_POST_FRAME_DELAY));

    //Read temperature and humidity (=> current ambient values)
    i2c_master_receive(dev_handle, i2c_receive_data, 6, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    measurement_SHT85_s.temperature_raw_u8a[0] = i2c_receive_data[0];
    measurement_SHT85_s.temperature_raw_u8a[1] = i2c_receive_data[1];
    measurement_SHT85_s.crc_received_u8a[0] = i2c_receive_data[2];

    measurement_SHT85_s.humidity_raw_u8a[0] = i2c_receive_data[3];
    measurement_SHT85_s.humidity_raw_u8a[1] = i2c_receive_data[4];
    measurement_SHT85_s.crc_received_u8a[1] = i2c_receive_data[5];

    //Check CRC
    measurement_SHT85_s.crc_calculated_u8a[0] = pSHT85_LF_calculate_CRC8_31(&i2c_receive_data[0], 2);
    measurement_SHT85_s.crc_calculated_u8a[1] = pSHT85_LF_calculate_CRC8_31(&i2c_receive_data[3], 2);

    if  (       ( measurement_SHT85_s.crc_calculated_u8a[0] != measurement_SHT85_s.crc_received_u8a[0] )
            ||  ( measurement_SHT85_s.crc_calculated_u8a[1] != measurement_SHT85_s.crc_received_u8a[1] )
        )
    {
        ESP_LOGE(module_tag, "Function pSHT85_LF_plausibility_check() encountered a problem "
            "while reading temperature and humidity (=> current ambient values) "
            "a CRC did not match!");
    }
    else
    {
        ESP_LOGI(module_tag, "While reading first ambient values, CRCs are good.");
    }

    ESP_LOGI(module_tag, "First measurement for current ambient values.");
    ESP_LOGI(module_tag, "\tTemperature RAW value: %X", measurement_SHT85_s.temperature_raw_u8a[0]);
    ESP_LOGI(module_tag, "\tTemperature RAW value: %X", measurement_SHT85_s.temperature_raw_u8a[1]);
    ESP_LOGI(module_tag, "\tHumidity RAW value: %X", measurement_SHT85_s.humidity_raw_u8a[0]);
    ESP_LOGI(module_tag, "\tHumidity RAW value: %X", measurement_SHT85_s.humidity_raw_u8a[1]);

    measurement_SHT85_s.temperature_raw_u16 = (measurement_SHT85_s.temperature_raw_u8a[0] << 8) + (measurement_SHT85_s.temperature_raw_u8a[1]);
    
    ESP_LOGI(module_tag, "\tTemperature RAW value: %X", measurement_SHT85_s.temperature_raw_u16);
    
    measurement_SHT85_s.temperature_f = -45.0f + ( 175.0f * ((float)measurement_SHT85_s.temperature_raw_u16 / 65535.0f) );

    ESP_LOGI(module_tag, "\tTemperature: %.2f °C", measurement_SHT85_s.temperature_f);

    //Check range of current ambient values (-40...+125°C; 0...100%)

    //Enable heater

    //Wait 1 or 2 seconds

    //Issue single shot measurement

    //Wait minimum 15ms

    //Read temperature and humidity 

    //Check CRC


    //Plausibility check (delta to previous ambient values)
    /* PASS: dT >= 5°C; dRH <= -2% */
    /* FAIL: dT < 3°C (heater not working or sensor wet); dRH > 0% (condensation or contamination) */

    
    //Disable heater

    //Wait about 100ms

    //Issue single shot measurement (recovery measurement) 
    /* T and RH should get back near to previous ambient values */
}


/* ======================== Public functions ========================= */

/* For documentation of public functions,
    please refer to function prototypes in header file. */

void pSHT85_F_init(void)
{
    ESP_LOGI(module_tag, "Init called.\n");


    pSHT85_LF_init_i2c_for_sensor_sht85();

    pSHT85_LF_init_sensor_sht85_variables();

    pSHT85_LF_read_sensor_sht85_serial();

    pSHT85_LF_log_sensor_sht85_serial();


    pSHT85_LF_plausibility_check();


    ESP_LOGI(module_tag, "Init finished.\n");
}

void pSHT85_F_de_init(void)
{
    ESP_LOGI(module_tag, "Deinitialization called.\n");


    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));


    ESP_LOGI(module_tag, "Deinitialization finished.\n");
}

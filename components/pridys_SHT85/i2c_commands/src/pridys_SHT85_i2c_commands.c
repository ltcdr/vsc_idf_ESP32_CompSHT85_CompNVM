/**
 * Implementation file of pridys component for SHT85 handling.
 * I2C Commands Submodule
 * pridys_SHT85_i2c_commands.c
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
#include "pridys_SHT85_i2c_commands.h"
#include "pridys_SHT85_crc_handler.h"

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


#define SHT85_ADDRESS                   0x44    //Sensor SHT85 I2C bus address as per the datasheet
#define SHT85_GET_SERIAL_36             0x36u   //First byte of command get serial
#define SHT85_GET_SERIAL_82             0x82u   //Second byte of command to get serial number from sensor SHT85

/* ======================== Global variable declaration ============== */
static const char *module_tag = "pridys_SHT85_i2c_commands";


typedef struct{
    four_byte_serial_SHT85_st   four_byte_serial_SHT85_s;
    uint8_t                     crc_received_u8a[2];
    uint8_t                     crc_calculated_u8a[2];
} serial_SHT85_st;

typedef struct{
    uint8_t     temperature_raw_u8a[2];
    uint16_t    temperature_raw_u16;
    float       temperature_f;
    uint8_t     humidity_raw_u8a[2];
    uint16_t    humidity_raw_u16;
    float       humidity_f;
    uint8_t     crc_received_u8a[2];
    uint8_t     crc_calculated_u8a[2];
} measurement_SHT85_st;


static serial_SHT85_st              serial_SHT85_s;
static measurement_SHT85_st         measurement_SHT85_s;
static measurement_pair_SHT85_f_st  measurement_pair_SHT85_f_s;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;


/* ======================== Function prototypes ====================== */
/**
 * Function for initialization of all global variables. 
 * All submodule internal global variables are initialized here.
 * Global variables like dev_handle are initialized elsewhere and 
 * they are intentionally without the keyword 'static'. They are 
 * used in other modules and bound in via 'extern' keyword. 
 * File pridys_SHT85_status_register.c also needs to send I2C frames
 * and therefore requires it. 
 */
void pSHT85_i2c_commands_LF_init_variables(void);

void pSHT85_i2c_commands_LF_init_handles_for_i2c_bus(void);


/* ======================== Private functions ======================== */
void pSHT85_i2c_commands_LF_init_variables(void)
{
    //Initialize struct variable for serial number of sensor SHT85
    memset(serial_SHT85_s.four_byte_serial_SHT85_s.serial_u8a, 0xFFu, 4);
    memset(serial_SHT85_s.crc_received_u8a, 0xFFu, 2);
    memset(serial_SHT85_s.crc_calculated_u8a, 0xFFu, 2);

    //Initialize struct variable for measurement values
    memset(measurement_SHT85_s.temperature_raw_u8a, 0xFFu, 2);
    memset(measurement_SHT85_s.humidity_raw_u8a, 0xFFu, 2);
    memset(measurement_SHT85_s.crc_received_u8a, 0xFFu, 2);
    memset(measurement_SHT85_s.crc_calculated_u8a, 0xFFu, 2);
    measurement_SHT85_s.temperature_raw_u16 = 0xFFFFu;
    measurement_SHT85_s.humidity_raw_u16 = 0xFFFFu;
    measurement_SHT85_s.temperature_f = NAN;
    measurement_SHT85_s.humidity_f = NAN;

    //Initialize struct variable for measurement pair
    measurement_pair_SHT85_f_s.temperature_f = NAN;
    measurement_pair_SHT85_f_s.humidity_f = NAN;
}

void pSHT85_i2c_commands_LF_init_handles_for_i2c_bus(void)
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
        ESP_LOGE(module_tag, "Function pSHT85_i2c_commands_F_init() encountered a problem "
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
        ESP_LOGE(module_tag, "Function pSHT85_i2c_commands_F_init() encountered a problem "
            "while initializing the I2C bus for communication with SHT85 sensor");
    }
}


/* ======================== Public functions ========================= */

/* For documentation of public functions,
        please refer to function prototypes in header file. */

void pSHT85_i2c_commands_F_read_serial(void)
{
    esp_err_t esp_err = ESP_FAIL;

    bool CRC1_OK = false;
    bool CRC2_OK = false;

    /* Empty buffer for 6 bytes: 
        2 bytes serial, CRC; another 2 bytes serial => 4 bytes serial */
    uint8_t i2c_receive_data[6] = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

    /* As per the datasheet, command 'Get Serial Number' 
        is 0x36 82 */
    uint8_t cmd[2] = {SHT85_GET_SERIAL_36, SHT85_GET_SERIAL_82};


    esp_err = i2c_master_transmit_receive(dev_handle, cmd, 2, i2c_receive_data, 6, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    vTaskDelay(pdMS_TO_TICKS(I2C_MASTER_POST_FRAME_DELAY));

    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_LF_read_sensor_sht85_serial() encountered a problem "
            "while communicating with SHT85 sensor");
    }


    //Store into global variable, copy the values from i2c buffer
    serial_SHT85_s.four_byte_serial_SHT85_s.serial_u8a[0] = i2c_receive_data[0];   //straight forward, copy first byte
    serial_SHT85_s.four_byte_serial_SHT85_s.serial_u8a[1] = i2c_receive_data[1];   //straight forward, copy second byte
    serial_SHT85_s.four_byte_serial_SHT85_s.serial_u8a[2] = i2c_receive_data[3];   //use fourth byte, because third is CRC 1
    serial_SHT85_s.four_byte_serial_SHT85_s.serial_u8a[3] = i2c_receive_data[4];   //use successor byte, fifth byte

    //Store CRC into variables global variable; CRCs are received in third and sixth byte
    serial_SHT85_s.crc_received_u8a[0] = i2c_receive_data[2];   //straight forward, copy first byte
    serial_SHT85_s.crc_received_u8a[1] = i2c_receive_data[5];   //straight forward, copy second byte


    //Calculate CRC and compare
    serial_SHT85_s.crc_calculated_u8a[0] = pSHT85_crc_handler_F_calculate_CRC(&serial_SHT85_s.four_byte_serial_SHT85_s.serial_u8a[0], 2);
    serial_SHT85_s.crc_calculated_u8a[1] = pSHT85_crc_handler_F_calculate_CRC(&serial_SHT85_s.four_byte_serial_SHT85_s.serial_u8a[2], 2);
    
    CRC1_OK = pSHT85_crc_handler_F_is_CRC_equal(serial_SHT85_s.crc_received_u8a[0], serial_SHT85_s.crc_calculated_u8a[0]);
    CRC2_OK = pSHT85_crc_handler_F_is_CRC_equal(serial_SHT85_s.crc_received_u8a[1], serial_SHT85_s.crc_calculated_u8a[1]);

    if( (false == CRC1_OK) || ((false == CRC2_OK)) )
    {
        memset(serial_SHT85_s.four_byte_serial_SHT85_s.serial_u8a, 0xFFu, 4);

        ESP_LOGE(module_tag, "Function pSHT85_i2c_commands_LF_read_serial() "
            "encountered a problem: Due to CRC mismatch, serial number reset to 0xFF!");
    }
}

four_byte_serial_SHT85_st pSHT85_i2c_commands_F_get_serial(void)
{
    return serial_SHT85_s.four_byte_serial_SHT85_s;
}


void pSHT85_i2c_commands_F_activate_heater(void)
{
    esp_err_t esp_err = ESP_FAIL;

    /* As per the datasheet, command 'Heater Enable' is 0x30 6D */
    uint8_t cmd[2] = {0x30u, 0x6Du};

    esp_err = i2c_master_transmit(dev_handle, cmd, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_i2c_commands_F_activate_heater()"
            "encountered a problem: i2c_master_transmit() did not return ESP_OK!");
    }
}

void pSHT85_i2c_commands_F_de_activate_heater(void)
{
    esp_err_t esp_err = ESP_FAIL;

    /* As per the datasheet, command 'Heater Enable' is 0x30 66 */
    uint8_t cmd[2] = {0x30u, 0x66u};

    esp_err = i2c_master_transmit(dev_handle, cmd, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_i2c_commands_F_activate_heater()"
            "encountered a problem: i2c_master_transmit() did not return ESP_OK!");
    }
}


bool pSHT85_i2c_commands_F_retrieve_measurement_pair(measurement_pair_SHT85_f_st * out_measurement_pair_s)
{
    bool return_value_loc = false;      //Initialize indicator of successful processing with FALSE

    bool CRC1_OK = false;
    bool CRC2_OK = false;

    esp_err_t esp_err = ESP_FAIL;
    ///TODO Check return error status from IDF framework functions I2C!

    /* Empty buffer for 6 bytes: 
        2 bytes temperature, CRC; another 2 bytes humidity, CRC => 6 bytes total */
    uint8_t i2c_receive_data[6] = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

    /* As per the datasheet, command 'single shot data acquisition mode', high repeatibility
        is 0x24 00 */
    uint8_t cmd[2] = {0x24u, 0x00u};


    if(NULL == out_measurement_pair_s)
    {
        //Caller provided a null pointer => Error and do not proceed!
        return_value_loc = false;

        ESP_LOGE(module_tag, "Function pSHT85_i2c_commands_F_retrieve_measurement_pair() "
            " Null pointer provided!");
    }
    else
    {
        //At least not a null pointer, proceed..

        //Trigger a single shot measurement on the sensor SHT85
        i2c_master_transmit(dev_handle, cmd, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

        //Wait minimum 15ms => 50ms
        vTaskDelay(pdMS_TO_TICKS(I2C_MASTER_POST_FRAME_DELAY));

        //Read temperature and humidity (=> current ambient values)
        i2c_master_receive(dev_handle, i2c_receive_data, 6, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

        //Store received values into struct variable
        measurement_SHT85_s.temperature_raw_u8a[0] = i2c_receive_data[0];
        measurement_SHT85_s.temperature_raw_u8a[1] = i2c_receive_data[1];
        measurement_SHT85_s.crc_received_u8a[0] = i2c_receive_data[2];

        measurement_SHT85_s.humidity_raw_u8a[0] = i2c_receive_data[3];
        measurement_SHT85_s.humidity_raw_u8a[1] = i2c_receive_data[4];
        measurement_SHT85_s.crc_received_u8a[1] = i2c_receive_data[5];

        //Calculate CRC
        measurement_SHT85_s.crc_calculated_u8a[0] = pSHT85_crc_handler_F_calculate_CRC(&i2c_receive_data[0], 2);
        measurement_SHT85_s.crc_calculated_u8a[1] = pSHT85_crc_handler_F_calculate_CRC(&i2c_receive_data[3], 2);

        //Compare CRC
        CRC1_OK = pSHT85_crc_handler_F_is_CRC_equal(measurement_SHT85_s.crc_received_u8a[0], measurement_SHT85_s.crc_calculated_u8a[0]);
        CRC2_OK = pSHT85_crc_handler_F_is_CRC_equal(measurement_SHT85_s.crc_received_u8a[1], measurement_SHT85_s.crc_calculated_u8a[1]);

        if( (false == CRC1_OK) || ((false == CRC2_OK)) )
        {
            return_value_loc = false;

            out_measurement_pair_s->temperature_f = NAN;
            out_measurement_pair_s->humidity_f = NAN;

            ESP_LOGE(module_tag, "Function pSHT85_i2c_commands_F_retrieve_measurement_pair() "
                "encountered a problem: Due to CRC mismatch, measurement pair is invalid!");
        }
        else
        {
            return_value_loc = true;

            //Calculate physical values using raw data

            measurement_SHT85_s.temperature_raw_u16 = (measurement_SHT85_s.temperature_raw_u8a[0] << 8) + (measurement_SHT85_s.temperature_raw_u8a[1]);
            measurement_SHT85_s.humidity_raw_u16 = (measurement_SHT85_s.humidity_raw_u8a[0] << 8) + (measurement_SHT85_s.humidity_raw_u8a[1]);

            measurement_SHT85_s.temperature_f = -45.0f + ( 175.0f * ((float)measurement_SHT85_s.temperature_raw_u16 / 65535.0f) );
            measurement_SHT85_s.humidity_f = 100.0f * ((float)measurement_SHT85_s.humidity_raw_u16 / 65535.0f);
            ///TODO Consider proposal of Copilot: for maximum precision, use 1.0f / 65535.0f as a constant => this avoids a division at runtime

            out_measurement_pair_s->temperature_f = measurement_SHT85_s.temperature_f;
            out_measurement_pair_s->humidity_f = measurement_SHT85_s.humidity_f;
        }
    }

    return return_value_loc;
}


void pSHT85_i2c_commands_F_init(void)
{
    ESP_LOGI(module_tag, "Init called.\n");


    pSHT85_i2c_commands_LF_init_variables();

    pSHT85_i2c_commands_LF_init_handles_for_i2c_bus();
    


    ESP_LOGI(module_tag, "Init finished.\n");
}


void pSHT85_i2c_commands_F_de_init(void)
{
    ESP_LOGI(module_tag, "Deinitialization called.\n");


    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));


    ESP_LOGI(module_tag, "Deinitialization finished.\n");
}

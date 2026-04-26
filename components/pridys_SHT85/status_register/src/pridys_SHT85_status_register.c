/**
 * Implementation file of pridys component for SHT85 handling.
 * Status Register Submodule
 * pridys_SHT85_status_register.c
 */


/* ======================== Standard includes ======================== */


/* ======================== Framework includes ======================= */
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/* ======================== Required includes ======================== */
#include "driver/i2c_master.h"


/* ======================== Proprietary includes ===================== */
#include "pridys_SHT85_status_register.h"
#include "pridys_SHT85_i2c_commands.h"
#include "pridys_SHT85_crc_handler.h"


/* ======================== Defines ================================== */
extern i2c_master_dev_handle_t dev_handle;


/* ======================== Global variable declaration ============== */
static const char *module_tag = "pridys_SHT85_status_register";

typedef struct{
    uint8_t     i2c_receive_data_u8a[3];
    uint8_t     raw_bytes_u8a[2];
    uint8_t     crc_received_u8;
    uint8_t     crc_calculated_u8;
} status_register_sht85_st;

status_register_sht85_st status_register_sht85_s; 


/* ======================== Function prototypes ====================== */

/**
 * Function to clear status register of sensor SHT85. 
 * This function will clear the status register of sensor SHT85 
 * via I2C command 0x30 41. 
 * This will clear all of the following bits to zero: 
 *      15  Alert pending status
 *      11  RH tracking alert
 *      10  T tracking alert
 *      4   System reset detected
 */
void pSHT85_status_register_LF_clear_SR(void);

/**
 * Function to read status register of sensor SHT85. 
 * This function will read status register of SHT85 
 * via I2C command 0xF3 2D.
 * The status register consists of two bytes, followed by a CRC.
 * Function will also do a CRC check and return the result.
 * @return:         bool indicating result of CRC check; 
 *                  => false:   CRC mismatch 
 *                  => true:    CRC check was OK
 */
//bool pSHT85_status_register_LF_read_SR(void);
//TODO Check if move to public was really senseful! 


/* ======================== Private functions ======================== */
void pSHT85_status_register_LF_clear_SR(void)
{
    esp_err_t esp_err = ESP_FAIL;

    /* As per the datasheet, command 'clear status register' is 0x30 41 */
    uint8_t cmd[2] = {0x30u, 0x41u};

    esp_err = i2c_master_transmit(dev_handle, cmd, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    if(ESP_OK != esp_err)
    {
        ESP_LOGE(module_tag, "Function pSHT85_status_register_LF_clear_SR()"
            "encountered a problem: i2c_master_transmit() did not return ESP_OK!");
    }
    else
    {
        ESP_LOGI(module_tag, "Status register of sensor SHT85 cleared.");
    }
}


/* ======================== Public functions ========================= */

/* For documentation of public functions,
        please refer to function prototypes in header file. */

bool pSHT85_status_register_F_read_SR(void)
{
    bool return_value_loc = false;      //Initialize indicator of successful processing with FALSE

    bool CRC_OK = false;                //Initialize with CRC Check not OK

    esp_err_t esp_err = ESP_FAIL;

    /* As per the datasheet, command 'read status register' is 0xF3 2D */
    uint8_t cmd[2] = {0xF3u, 0x2Du};

    esp_err = i2c_master_transmit_receive(
        dev_handle, cmd, 2, 
        status_register_sht85_s.i2c_receive_data_u8a, 3, 
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );

    vTaskDelay(pdMS_TO_TICKS(I2C_MASTER_POST_FRAME_DELAY));

    if(ESP_OK != esp_err)
    {
        return_value_loc = false;

        ESP_LOGE(module_tag, "Function pSHT85_status_register_F_read_SR()"
            "encountered a problem: i2c_master_transmit() did not return ESP_OK!");
    }

    status_register_sht85_s.raw_bytes_u8a[0] = status_register_sht85_s.i2c_receive_data_u8a[0];
    status_register_sht85_s.raw_bytes_u8a[1] = status_register_sht85_s.i2c_receive_data_u8a[1];
    status_register_sht85_s.crc_received_u8 = status_register_sht85_s.i2c_receive_data_u8a[2];

    status_register_sht85_s.crc_calculated_u8 = pSHT85_crc_handler_F_calculate_CRC(
        &status_register_sht85_s.raw_bytes_u8a[0], 2
    );
    CRC_OK = pSHT85_crc_handler_F_is_CRC_equal(
        status_register_sht85_s.crc_calculated_u8, 
        status_register_sht85_s.crc_received_u8
    );

    if(false == CRC_OK)
    {
        return_value_loc = false;

        memset(&status_register_sht85_s.raw_bytes_u8a[0], 0xFFu, 2);

        ESP_LOGE(module_tag, "Function pSHT85_status_register_F_read_SR() "
            "encountered a problem: Due to CRC mismatch, status register is invalid!");
    }
    else
    {
        return_value_loc = true;
    }


    /*temporary debug code*/
    ESP_LOGI(module_tag, "Raw SR bytes: %02X %02X (CRC OK: %d)",
         status_register_sht85_s.raw_bytes_u8a[0],
         status_register_sht85_s.raw_bytes_u8a[1],
         CRC_OK);


    return return_value_loc;
}


bool pSHT85_status_register_F_is_heater_active(void)
{
    bool return_value_loc = true;

    if(     (0xFFu == status_register_sht85_s.raw_bytes_u8a[0])
        ||  (0xFFu == status_register_sht85_s.raw_bytes_u8a[1])     )
    {
        //Status register was never read or CRC was wrong
        // => Let the caller believe heater is active to draw attention
        return_value_loc = true;
    }
    else
    {
        //Status register seems to be successfully read previously
        // => Evaluate heater status

        if( (status_register_sht85_s.raw_bytes_u8a[0] & 0x20u) == 0x20u)
        {
            return_value_loc = true;
        }
        else
        {
            return_value_loc = false;
        }

    }

    return return_value_loc;
}


void pSHT85_status_register_F_init(void)
{
    ESP_LOGI(module_tag, "Init called.\n");


    memset(&status_register_sht85_s, 0xFFu, sizeof(status_register_sht85_s));

    pSHT85_status_register_LF_clear_SR();
    pSHT85_status_register_F_read_SR();


    ESP_LOGI(module_tag, "Init finished.\n");
}

void pSHT85_status_register_F_de_init(void)
{
    ESP_LOGI(module_tag, "Deinitialization called.\n");


    


    ESP_LOGI(module_tag, "Deinitialization finished.\n");
}

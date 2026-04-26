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


/* ======================== Proprietary includes ===================== */
#include "pridys_SHT85.h"
#include "pridys_SHT85_i2c_commands.h"
#include "pridys_SHT85_status_register.h"


/* ======================== Defines ================================== */


/* ======================== Global variable declaration ============== */
static const char *module_tag = "pridys_sht85";


four_byte_serial_SHT85_st       four_byte_serial_SHT85_s;
measurement_pair_SHT85_f_st     measurement_pair_SHT85_f_s;


/* ======================== Function prototypes ====================== */
static void pSHT85_LF_init_variables(void);
static void pSHT85_LF_log_sensor_sht85_serial(void);


/* ======================== Private functions ======================== */

/**
 * Local function to initialize variables for SHT85 handling, such as serial number. 
 * Initializes 
 *      - global struct serial_SHT85_s containing serial number and CRCs to 0xFFu
 *      - global struct measurement_pair_SHT85_f_s with NAN
 */
static void pSHT85_LF_init_variables(void)
{
    memset(four_byte_serial_SHT85_s.serial_u8a, 0xFFu, 4);
    measurement_pair_SHT85_f_s.temperature_f = NAN;
    measurement_pair_SHT85_f_s.humidity_f = NAN;
}

/**
 * Local function to log serial number of SHT85 via UART to console. 
 * Reading global serial_SHT85_s and printing the content via ESP_LOGI().
 */
static void pSHT85_LF_log_sensor_sht85_serial(void)
{
    four_byte_serial_SHT85_s = pSHT85_i2c_commands_F_get_serial();

    ESP_LOGI(   module_tag, "Sensor SHT85 serial number: %02X %02X %02X %02X",
        four_byte_serial_SHT85_s.serial_u8a[0], four_byte_serial_SHT85_s.serial_u8a[1], 
        four_byte_serial_SHT85_s.serial_u8a[2], four_byte_serial_SHT85_s.serial_u8a[3]  );
}


/* ======================== Public functions ========================= */

/* For documentation of public functions,
    please refer to function prototypes in header file. */

void pSHT85_F_init(void)
{
    ESP_LOGI(module_tag, "Init called.\n");


    //Call module internal init functions
    pSHT85_LF_init_variables();


    //Call submodule init functions
    pSHT85_i2c_commands_F_init();
    pSHT85_status_register_F_init();


    //Kinda runtime calls... 
    ///TODO Create runtime function and move this behaviour there!
    pSHT85_i2c_commands_F_read_serial();

    pSHT85_status_register_F_read_SR();
    ESP_LOGI(module_tag, "Heater Status Bit: %d", pSHT85_status_register_F_is_heater_active());

    pSHT85_LF_log_sensor_sht85_serial();

    (void)pSHT85_i2c_commands_F_retrieve_measurement_pair(&measurement_pair_SHT85_f_s);
    ESP_LOGI(module_tag, "Temperature: %.2f", measurement_pair_SHT85_f_s.temperature_f);
    ESP_LOGI(module_tag, "Humidity: %.2f", measurement_pair_SHT85_f_s.humidity_f);


    //pSHT85_LF_plausibility_check();

    ESP_LOGI(module_tag, "Activating heater...");
    pSHT85_i2c_commands_F_activate_heater();

    
    ESP_LOGI(module_tag, ".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(module_tag, ".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(module_tag, ".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(module_tag, "Waited eight seconds..here comes a measurement...");


    pSHT85_status_register_F_read_SR();
    ESP_LOGI(module_tag, "Heater Status Bit: %d", pSHT85_status_register_F_is_heater_active());


    (void)pSHT85_i2c_commands_F_retrieve_measurement_pair(&measurement_pair_SHT85_f_s);
    ESP_LOGI(module_tag, "Temperature: %.2f", measurement_pair_SHT85_f_s.temperature_f);
    ESP_LOGI(module_tag, "Humidity: %.2f", measurement_pair_SHT85_f_s.humidity_f);


    ESP_LOGI(module_tag, "Deactivating heater...");
    pSHT85_i2c_commands_F_de_activate_heater();


    ESP_LOGI(module_tag, ".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(module_tag, ".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(module_tag, ".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(module_tag, "Waited eight seconds..here comes a measurement...");


    pSHT85_status_register_F_read_SR();
    ESP_LOGI(module_tag, "Heater Status Bit: %d", pSHT85_status_register_F_is_heater_active());


    (void)pSHT85_i2c_commands_F_retrieve_measurement_pair(&measurement_pair_SHT85_f_s);
    ESP_LOGI(module_tag, "Temperature: %.2f", measurement_pair_SHT85_f_s.temperature_f);
    ESP_LOGI(module_tag, "Humidity: %.2f", measurement_pair_SHT85_f_s.humidity_f);



    ESP_LOGI(module_tag, "Init finished.\n");
}

void pSHT85_F_de_init(void)
{
    ESP_LOGI(module_tag, "Deinitialization called.\n");


    pSHT85_i2c_commands_F_de_init();


    ESP_LOGI(module_tag, "Deinitialization finished.\n");
}

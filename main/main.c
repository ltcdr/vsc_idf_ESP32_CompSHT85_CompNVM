/**
 * Pridys temperature and humidity measurement project V001
 * Using a SHT85 via I2C and Non Volatile Memory (FATFS, wear levelling)
 */

/* ====== Standard includes ====== */
#include <stdio.h>

/* ====== Framework includes ====== */
#include "esp_log.h"

/* ====== Proprietary includes ====== */


/* ====== Defines ====== */
//#define asdf 0x01

/* ====== Global variable declaration ====== */
static const char * module_tag = "ESP_SHT85_NVM_main";


/**
 * Application main function
 */
void app_main(void)
{
    ESP_LOGI(module_tag, "\n");
    ESP_LOGI(module_tag, "Application started...\n");

    /* ====== Init section ====== */
    

    /* ====== Runtime section ====== */


    /* ====== De init section ====== */
    


    ESP_LOGI(TAG, "Application stopped.\n");
}

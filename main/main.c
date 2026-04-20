/**
 * Pridys temperature and humidity measurement project V001
 * Using a SHT85 via I2C and Non Volatile Memory (FATFS, wear levelling)
 */

/* ======================== Standard includes ======================== */
#include <stdio.h>

/* ======================== Framework includes ======================= */
#include "esp_log.h"

/* ======================== Proprietary includes ===================== */
#include "pridys_SHT85.h"

/* ======================== Defines ================================== */
//#define asdf 0x01

/* ======================== Global variable declaration ============== */
static const char * module_tag = "ESP_SHT85_NVM_main";


/**
 * Application main function
 */
void app_main(void)
{
    ESP_LOGI(module_tag, "\n");
    ESP_LOGI(module_tag, "Application started...\n");

    /* ====== Init section ====== */
    pSHT85_F_init();

    /* ====== Runtime section ====== */


    /* ====== De init section ====== */
    pSHT85_F_de_init();


    ESP_LOGI(module_tag, "Application stopped.\n");
}

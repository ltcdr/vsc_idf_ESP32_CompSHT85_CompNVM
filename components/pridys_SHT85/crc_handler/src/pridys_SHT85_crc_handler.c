/**
 * Implementation file of pridys component for SHT85 handling.
 * CRC Handler Submodule
 * pridys_SHT85_crc_handler.c
 */

 /* ======================== Standard includes ======================== */
#include <stdio.h>
#include <string.h>

/* ======================== Framework includes ======================= */
#include "esp_log.h"

/* ======================== Required includes ======================== */


/* ======================== Proprietary includes ===================== */
#include "pridys_SHT85_crc_handler.h"

/* ======================== Defines ================================== */


/* ======================== Global variable declaration ============== */
static const char *module_tag = "pridys_SHT85_crc_handler";


/* ======================== Function prototypes ====================== */


/* ======================== Private functions ======================== */


/* ======================== Public functions ========================= */

/* For documentation of public functions,
    please refer to function prototypes in header file. */

uint8_t pSHT85_crc_handler_F_calculate_CRC(const uint8_t * data, size_t len)
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

/**
 * Function to compare two CRC bytes.
 * @return bool     =>  TRUE if given CRC matched
 *                  =>  FALSE if provided CRC did not match
 */
bool pSHT85_crc_handler_F_compare_CRC(uint8_t CRC1, uint8_t CRC2)
{
    bool loc_CRC_are_same_return_b = false;

    //Compare CRC1 and CRC2 given as parameters
    if( CRC1 != CRC2 )
    {
        loc_CRC_are_same_return_b = false;

        ESP_LOGE(module_tag, "Function pSHT85_crc_handler_F_compare_CRC() "
            "encountered a problem: CRC1 is not same as CRC2!");
        ESP_LOGI(module_tag, "\tCRC1: %X", CRC1);
        ESP_LOGI(module_tag, "\tCRC2: %X", CRC2);
    }
    else
    {
        loc_CRC_are_same_return_b = true;
    }

    return loc_CRC_are_same_return_b;
}

/**
 * Header file of pridys component for SHT85 handling.
 * pridys_SHT85_crc_handler.h
 */


#ifndef PRIDYS_SHT85_CRC_HANDLER_H
#define PRIDYS_SHT85_CRC_HANDLER_H

/**
 * Function to calculate CRC for sensor SHT85. 
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
uint8_t pSHT85_crc_handler_F_calculate_CRC(const uint8_t * data, size_t len);

/**
 * Function to compare two CRC bytes.
 * @return bool     =>  TRUE if given CRC matched
 *                  =>  FALSE if provided CRC did not match
 */
bool pSHT85_crc_handler_F_compare_CRC(uint8_t CRC1, uint8_t CRC2);


#endif /* PRIDYS_SHT85_CRC_HANDLER_H */

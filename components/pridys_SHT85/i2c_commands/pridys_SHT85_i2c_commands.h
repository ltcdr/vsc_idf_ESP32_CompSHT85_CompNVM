/**
 * Header file of pridys component for SHT85 handling.
 * pridys_SHT85_i2c_commands.h
 */


#ifndef PRIDYS_SHT85_I2C_COMMANDS_H
#define PRIDYS_SHT85_I2C_COMMANDS_H


/** 1000ms timeout for an I2C frame. */
#define I2C_MASTER_TIMEOUT_MS           1000

/** 50ms delay after sending/receiving frames via I2C.
 * Recommended to let the system process previous communication action. 
 */
#define I2C_MASTER_POST_FRAME_DELAY     50


typedef struct{
    uint8_t     serial_u8a[4];
} four_byte_serial_SHT85_st;

typedef struct{
    float       temperature_f;
    float       humidity_f;
} measurement_pair_SHT85_f_st;


/**
 * Initialization function of pridys SHT85 I2C commands submodule. 
 * Sets up I2C communication channel.
 */
void pSHT85_i2c_commands_F_init(void);

/**
 * Deinitialization function of pridys SHT85 I2C commands submodule. 
 * Removes and deletes I2C, freeing up this channel or pins. 
 */
void pSHT85_i2c_commands_F_de_init(void);

/**
 * Function to read serial number of sensor SHT85. 
 * Via I2C, read serial number of sensor SHT85, calculate CRC based on received data
 * and compare received and calculated CRC. In case of CRC mismatch, serial number
 * will be reset to initialization value 0xFF. 
 */
void pSHT85_i2c_commands_F_read_serial(void);

/**
 * Getter function for struct four_byte_serial_SHT85_st. 
 * Only member of the struct is a four byte array "serial_u8a". 
 * This is a pure getter function, simply returning current value of submodule internal global variable. 
 * In case content is 0xFFu, variable was either only initialized and not set or CRC was wrong.
 */
four_byte_serial_SHT85_st pSHT85_i2c_commands_F_get_serial(void);

/**
 * Function to turn ON the heater of sensor SHT85. 
 * This function turns on heater of SHT85 via I2C command, 
 * command is 0x30 6D as per the datasheet. 
 * This function does not do any verification if heater was turned on successfully
 * or whether or not is already on. It simply sends the command to the sensor. 
 */
void pSHT85_i2c_commands_F_activate_heater(void);

/**
 * Function to turn OFF the heater of sensor SHT85. 
 * This function turns off heater of SHT85 via I2C command, 
 * command is 0x30 66 as per the datasheet. 
 * This function does not do any verification if heater was turned off successfully
 * or whether or not is already off. It simply sends the command to the sensor. 
 */
void pSHT85_i2c_commands_F_de_activate_heater(void);

/**
 * Function to retrieve temperature and humidity via single shot measurement.
 * This function triggers a single shot measurement, waits 50ms 
 * and receives the measurement values via I2C.
 * Calculation from raw values to physical values is done in this function.
 * After verifying data integrity via CRC, 
 * it provides the physical values (float) via out parameters: 
 * @temperature:    [out] Pointer to buffer (float) for temperature value
 * @humidity:       [out] Pointer to buffer (float) for humidity value
 * @return:         bool indicating result of CRC check; 
 *                  => false:   CRC mismatch 
 *                  => true:    CRC check was OK 
 */
bool pSHT85_i2c_commands_F_retrieve_measurement_pair(measurement_pair_SHT85_f_st * out_measurement_pair_s);

#endif /* PRIDYS_SHT85_I2C_COMMANDS_H */

/**
 * Header file of pridys component for SHT85 handling.
 * pridys_SHT85.h
 */


#ifndef PRIDYS_SHT85_H
#define PRIDYS_SHT85_H


/**
 * Initialization function of pridys SHT85 component. 
 * Sets up I2C, reads serial number and status register.
 */
void pSHT85_F_init(void);

/**
 * Deinitialization function of pridys SHT85 component. 
 * Removes and deletes I2C, freeing up this channel or pins. 
 */
void pSHT85_F_de_init(void);


#endif /* PRIDYS_SHT85_H */

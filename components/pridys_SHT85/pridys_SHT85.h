/**
 * Header file of pridys component for SHT85 handling.
 * pridys_SHT85.h
 */


#ifndef PRIDYS_SHT85_H
#define PRIDYS_SHT85_H


/**
 * Initialization function of pridys SHT85 component. 
 * Calls init functions from submodules, 
 * initializes module internal global variables.
 */
void pSHT85_F_init(void);

/**
 * Deinitialization function of pridys SHT85 component. 
 * Calls de-init functions from submodules. 
 */
void pSHT85_F_de_init(void);

/**
 * Cyclic/runtime function of pridys SHT85 component.
 * Behaviour and workflow is defined here
 *      - Read serial number of sensor SHT85
 *      - Reset sensor SHT85
 *      - Clear and read status register of sensor SHT85
 *      - Do a plausibility check via heater and measurement readings
 *      - TBD
 */
void pSHT85_F_cyclic(void);


#endif /* PRIDYS_SHT85_H */

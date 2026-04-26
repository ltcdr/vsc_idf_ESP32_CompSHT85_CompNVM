/**
 * Header file of pridys component for SHT85 handling.
 * pridys_SHT85_i2c_commands.h
 */


#ifndef PRIDYS_SHT85_STATUS_REGISTER_H
#define PRIDYS_SHT85_STATUS_REGISTER_H


void pSHT85_status_register_F_init(void);

void pSHT85_status_register_F_de_init(void);

bool pSHT85_status_register_F_read_SR(void);

bool pSHT85_status_register_F_is_heater_active(void);

#endif /* PRIDYS_SHT85_STATUS_REGISTER_H */

/*
 * SHT20_driver.h
 *
 *  Created on: Feb 11, 2026
 *      Author: aboba
 */

#ifndef INC_SHT20_DRIVER_H_
#define INC_SHT20_DRIVER_H_

#define SHT_ADDR 0x40

#define SHT_TRIGGER_T_HOLD        0xE3  // Trigger T measurement, hold master
#define SHT_TRIGGER_RH_HOLD       0xE5  // Trigger RH measurement, hold master

#define SHT_WRITE_USER_REG        0xE6  // Write user register
#define SHT_READ_USER_REG         0xE7  // Read user register

#define SHT_SOFT_RESET            0xFE  // Soft reset

float sht20_Temp_Mess();
float sht20_Hum_Mess();

#endif /* INC_SHT20_DRIVER_H_ */

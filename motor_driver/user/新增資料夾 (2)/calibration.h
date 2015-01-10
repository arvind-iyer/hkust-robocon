#ifndef _CALIBRATION_H
#define _CALIBRATION_H

#include "configuration.h"

void data_size_initialize(void);
void flash_write_halfword(uint16_t data);
void flash_write_array(uint16_t* data, uint16_t data_length);
void test_write(void);
void print_test(void);
void max_voltage_calibration(void);
void min_voltage_calibration(void);
void store_calibration_data(void);
void read_calibration_data(void);
void linearization(void);

#define MAX_VOL_DATA_LENGTH 32
#define MIN_VOL_DATA_LENGTH 32
#define CAL_DATA_LENGTH 64

#endif


#ifndef _BMP280_TEMP_H
#define _BMP280_TEMP_H

bool bmp280_temp_init(void);
bool bmp280_temp_read(double* temp);
void bmp280_temp_start_reading(void);

#endif
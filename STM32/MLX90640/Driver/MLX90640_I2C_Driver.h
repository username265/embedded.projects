#ifndef MLX90640_I2C_DRIVER_H
#define MLX90640_I2C_DRIVER_H

#include <stdint.h>

void MLX90640_I2CInit(void);
void MLX90640_I2CFreqSet(int freq);

int MLX90640_I2CGeneralReset(void);

int MLX90640_I2CRead(uint8_t slaveAddr,
                     uint16_t startAddress,
                     uint16_t nMemAddressRead,
                     uint16_t *data);

int MLX90640_I2CWrite(uint8_t slaveAddr,
                      uint16_t writeAddress,
                      uint16_t data);

#endif

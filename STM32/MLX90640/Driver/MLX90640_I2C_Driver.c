#include "MLX90640_I2C_Driver.h"
#include "main.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;

#define MLX90640_I2C_TIMEOUT  1000U
#define MLX90640_READ_WORDS   32U

void MLX90640_I2CInit(void)
{
    /*
     * I2C1은 CubeMX가 MX_I2C1_Init()에서 초기화하므로
     * 이 함수에서는 별도 작업을 하지 않습니다.
     */
}

void MLX90640_I2CFreqSet(int freq)
{
    /*
     * I2C 속도 역시 CubeMX에서 설정합니다.
     * 실행 중 변경하지 않으므로 비워 둡니다.
     */
    (void)freq;
}

int MLX90640_I2CGeneralReset(void)
{
    uint8_t command = 0x06;

    /*
     * General Call 주소는 0x00입니다.
     * HAL은 8비트 형태의 주소를 받으므로 0x00 << 1도 0x00입니다.
     */
    if (HAL_I2C_Master_Transmit(&hi2c1,
                               0x00,
                               &command,
                               1,
                               MLX90640_I2C_TIMEOUT) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

int MLX90640_I2CRead(uint8_t slaveAddr,
                     uint16_t startAddress,
                     uint16_t nMemAddressRead,
                     uint16_t *data)
{
    uint16_t wordsRemaining = nMemAddressRead;
    uint16_t currentAddress = startAddress;
    uint16_t dataIndex = 0;

    uint8_t receiveBuffer[MLX90640_READ_WORDS * 2];

    while (wordsRemaining > 0)
    {
        uint16_t wordsToRead;

        if (wordsRemaining > MLX90640_READ_WORDS)
        {
            wordsToRead = MLX90640_READ_WORDS;
        }
        else
        {
            wordsToRead = wordsRemaining;
        }

        uint16_t bytesToRead = wordsToRead * 2;

        if (HAL_I2C_Mem_Read(&hi2c1,
                             (uint16_t)(slaveAddr << 1),
                             currentAddress,
                             I2C_MEMADD_SIZE_16BIT,
                             receiveBuffer,
                             bytesToRead,
                             MLX90640_I2C_TIMEOUT) != HAL_OK)
        {
            return -1;
        }

        for (uint16_t i = 0; i < wordsToRead; i++)
        {
            data[dataIndex + i] =
                ((uint16_t)receiveBuffer[2 * i] << 8)
                | receiveBuffer[2 * i + 1];
        }

        currentAddress += wordsToRead;
        dataIndex += wordsToRead;
        wordsRemaining -= wordsToRead;
    }

    return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr,
                      uint16_t writeAddress,
                      uint16_t data)
{
    uint8_t writeBuffer[2];

    writeBuffer[0] = (uint8_t)(data >> 8);
    writeBuffer[1] = (uint8_t)(data & 0xFF);

    if (HAL_I2C_Mem_Write(&hi2c1,
                          (uint16_t)(slaveAddr << 1),
                          writeAddress,
                          I2C_MEMADD_SIZE_16BIT,
                          writeBuffer,
                          2,
                          MLX90640_I2C_TIMEOUT) != HAL_OK)
    {
        return -1;
    }

    HAL_Delay(5);

    return 0;
}

#include "ICM42688_Middleware.h"
#include "main.h"

SPI_HandleTypeDef *ICM42688_SPI;


uint8_t ICM42688_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
//    HAL_SPI_TransmitReceive(ICM42688_SPI, &txdata, &rx_data, 1, 1000);
		HAL_SPI_TransmitReceive_DMA(ICM42688_SPI, &txdata, &rx_data, 1);
    return rx_data;
}

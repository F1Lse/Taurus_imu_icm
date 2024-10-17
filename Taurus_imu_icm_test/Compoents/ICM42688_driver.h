#ifndef __ICM42688_DRIVER_H__
#define __ICM42688_DRIVER_H__

#include "stdint.h"
#include "main.h"



typedef struct
{
    float Accel[3];

    float Gyro[3];

    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];

    float gNorm;
} IMU_Data_t;


float bsp_Icm42688GetAres(uint8_t Ascale);

float bsp_Icm42688GetGres(uint8_t Gscale);

void bsp_IcmGetRawData(IMU_Data_t *ICM42688);

int8_t bsp_IcmGetTemperature(int16_t* pTemp);

int16_t ICM42688_init(void);

extern uint8_t init_flag;
extern IMU_Data_t IMU_Data;
extern int16_t temp;
#endif

#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "stm32g4xx.h"

typedef struct{
    union
    {
        uint8_t array[8];
        struct
        {
            float pit;
            float wx;
        }e;
    } pit_msg;//pit
    union
    {
        uint8_t array[8];
        struct
        {
            float yaw;
            float wz;
        } e;
    } yaw_msg;//yaw
		    union
    {
        uint8_t array[8];
        struct
        {
            float rol;
            float wy;
        } e;
    } rol_msg;//roll
}imu_msg_t;	

void can_comm_init(void);
uint8_t can_std_transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

extern imu_msg_t imu_msg_send;

#endif

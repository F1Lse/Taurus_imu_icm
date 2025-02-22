#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#include "stm32g4xx_hal.h"
//#include "FreeRTOS.h"


#include <math.h>
#include <stdint.h>

#define PI 3.14159265358979323846f

#define STATIC_DETECT_WINDOW 20    // ��ֹ��ⴰ�ڴ�С
#define STATIC_THRESHOLD      0.02f // ��ֹ״̬�ж���ֵ(rad/s)
#define GYRO_DT 0.001f  // 1ms��������
#define MOTION_THRESHOLD 2.0f  // �˶������ֵ(rad/s)

typedef struct {
    float q;    // ��������Э����
    float r;    // ��������Э����
    float p;    // �������Э����
    float k;    // ����������
    float x;    // ���Ź���ֵ
    float b;    // ��ƫ����
} KalmanFilter;

typedef struct {
    float window[5];    // ��������
    uint8_t index;      // ��ǰ����
    float sum;          // �������
} MovingAverage;

typedef struct {
    KalmanFilter kf[3];  // XYZ���Ῠ�����˲���
    MovingAverage ma[3]; // XYZ���Ử��ƽ��
    float adaptive_gain; // ����Ӧ����ϵ��
    float last_output[3];// �ϴ����ֵ
	
	    float static_bias[3];      // ��̬��ƫ����ֵ
    float variance[3];         // ������㻺��
    uint8_t static_counter;    // ��ֹ״̬������
	
	
} GyroFilter;


void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void GyroFilter_Update(GyroFilter* gf, float* raw_data, float* filtered_data);
void GyroFilter_Init(GyroFilter* gf) ;
float invSqrt(float x);

#endif


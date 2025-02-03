#ifndef MAHONY_H
#define MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define PI 3.14159265358979323846f
#define DEFAULT_ZERO_TOLERANCE 1e-6f


typedef struct {
    float q0, q1, q2, q3;   // ��Ԫ��
    float integralFBx, integralFBy, integralFBz; // �������
    float Kp;               // ��������
    float Ki;               // ��������
    float sampleFreq;       // ����Ƶ��(Hz)
} MahonyFilter;
 
// ��ʼ���˲���
void Mahony_Init(MahonyFilter* filter, float sampleFreq, float Kp, float Ki);

// ������̬���㣨�趨ʱ���ã�
void Mahony_Update(MahonyFilter* filter, 
                  float gx, float gy, float gz,  // ����������(rad/s)
                  float ax, float ay, float az); // ���ٶȼ�����(��һ��)

// ��ȡŷ����(����)
void Mahony_GetEulerAngles(const MahonyFilter* filter, 
                          float* roll, 
                          float* pitch, 
                          float* yaw);

// ��Ԫ��תŷ����(ֱ��ʹ��)
void QuaternionToEuler(float q0, float q1, float q2, float q3,
                      float* roll, float* pitch, float* yaw);

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_FILTER_H */

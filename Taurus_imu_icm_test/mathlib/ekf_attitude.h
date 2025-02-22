#ifndef EKF_ATTITUDE_H
#define EKF_ATTITUDE_H

#include "arm_math.h"

typedef struct {
    // ״̬����Ԫ�� + ����ƫ�� [q0, q1, q2, q3, bias_x, bias_y, bias_z]
    float x[7];
    float dt;  // ��������
    
    // Э������� (7x7)
    arm_matrix_instance_f32 P;
    float P_data[49];
    
    // �������� (7x7)
    arm_matrix_instance_f32 Q;
    float Q_data[49];
    
    // �۲����� (3x3)
    arm_matrix_instance_f32 R;
    float R_data[9];
    
    // ��ʱ����
    arm_matrix_instance_f32 F, H, K, S, y;
    float F_data[49], H_data[21], K_data[21], S_data[9], y_data[3];
} EKF_Instance;

// ��ʼ���˲���
void EKF_Init(EKF_Instance* ekf, float dt);

// Ԥ�ⲽ�裨���������룺rad/s��
void EKF_Predict(EKF_Instance* ekf, float wx, float wy, float wz,float dt);

// ���²��裨���ٶȼ����룺m/s2��
void EKF_Update(EKF_Instance* ekf, float ax, float ay, float az);

// ��ȡŷ���ǣ����ȣ�
void EKF_GetEulerAngles(const EKF_Instance* ekf, float* roll, float* pitch, float* yaw);

#endif

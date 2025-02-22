/**
	* @file data_processing.c
	* @version 1.0
	* @date 2019.11.26
  *
  * @brief  һЩ���ݴ�����
  *
  *	@author YY
  *
  */
	
#include "data_processing.h"


/***************************************************************/
/*
 * ��������Float2Byte
 * ����  ���������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ
 * ����  ��target:Ŀ�굥��������
					 buf:��д������
					 beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
 * ���  ����
 */ 
/***************************************************************/
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
// Fast inverse square-root
*******************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


// �������˲�����ʼ��
void Kalman_Init(KalmanFilter* kf, float q, float r) {
    kf->q = q;
    kf->r = r;
    kf->p = 1.0f;
    kf->k = 0.0f;
    kf->x = 0.0f;
    kf->b = 0.0f;
}

// �������˲�������
float Kalman_Update(KalmanFilter* kf, float measurement) {
    // Ԥ��׶�
    kf->p += kf->q;
    
    // ���½׶�
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x - kf->b);
    kf->p *= (1 - kf->k);
    
    // ��ƫ����Ӧ����
    kf->b += 0.001f * (measurement - kf->x - kf->b);
    
    return kf->x;
}

// ����ƽ����ʼ��
void MA_Init(MovingAverage* ma) {
    for(int i=0; i<5; i++) ma->window[i] = 0;
    ma->index = 0;
    ma->sum = 0;
}

// ����ƽ������
float MA_Update(MovingAverage* ma, float input) {
    ma->sum -= ma->window[ma->index];
    ma->window[ma->index] = input;
    ma->sum += input;
    ma->index = (ma->index + 1) % 5;
    return ma->sum / 5.0f;
}

// �������˲�����ʼ��
void GyroFilter_Init(GyroFilter* gf) {
    for(int i=0; i<3; i++) {
        Kalman_Init(&gf->kf[i], 0.001f, 0.03f);
        MA_Init(&gf->ma[i]);
        gf->last_output[i] = 0.0f;
    }
    gf->adaptive_gain = 1.0f;
		
		    for(int i=0; i<3; i++) {
        gf->static_bias[i] = 0.0f;
        gf->variance[i] = 0.0f;
    }
    gf->static_counter = 0;
		
		
}

// ���˲�����
void GyroFilter_Update(GyroFilter* gf, float* raw_data, float* filtered_data) {
  float motion_level = 0;
    float ma_output[3];
    static float history[STATIC_DETECT_WINDOW][3];
    
    // �׶�1������ƽ��Ԥ�������ֲ��䣩
    for(int i=0; i<3; i++) {
        ma_output[i] = MA_Update(&gf->ma[i], raw_data[i]);
    }

    // ������Yaw��ר��� -----------------------------------
    // ��⾲ֹ״̬�����ڻ������ڷ�����㣩
    static uint8_t history_index = 0;
    for(int i=0; i<3; i++){
        // ������ʷ���ݶ���
        history[history_index][i] = ma_output[i];
        
        // ���㴰�ڷ���
        float mean = gf->ma[i].sum / 5.0f;
        gf->variance[i] = 0.0f;
        for(int j=0; j<STATIC_DETECT_WINDOW; j++){
            gf->variance[i] += powf(history[j][i] - mean, 2);
        }
        gf->variance[i] /= STATIC_DETECT_WINDOW;
    }
    history_index = (history_index + 1) % STATIC_DETECT_WINDOW;

    // ��ֹ״̬�жϣ������᷽���������ֵ��
    uint8_t is_static = 1;
    for(int i=0; i<3; i++){
        if(gf->variance[i] > STATIC_THRESHOLD) {
            is_static = 0;
            break;
        }
    }

    // Yaw����ƫ��̬���������ھ�ֹʱ���£�
    if(is_static) {
        gf->static_counter = (gf->static_counter < 255) ? gf->static_counter+1 : 255;
        
        // ָ������ƽ��������ƫ
        float alpha = (gf->static_counter > 50) ? 0.01f : 0.0f;
        for(int i=0; i<3; i++){
            gf->static_bias[i] = (1-alpha)*gf->static_bias[i] + alpha*ma_output[i];
        }
        
        // �ر���ǿYaw�Ჹ����Z�ᣩ
        gf->kf[2].b = 0.95f*gf->kf[2].b + 0.05f*gf->static_bias[2];
    } else {
        gf->static_counter = 0;
    }

    // �׶�2���������Ŀ������˲� -----------------------------
    for(int i=0; i<3; i++) {
        // ע�뾲̬����ֵ
        float compensated_meas = ma_output[i] - gf->static_bias[i];
        filtered_data[i] = Kalman_Update(&gf->kf[i], compensated_meas);
        motion_level += fabsf(filtered_data[i] - gf->last_output[i]);
    }
    
    // �����׶Σ���̬��Ӧ����
    motion_level /= 3.0f * GYRO_DT;
    gf->adaptive_gain = (motion_level > MOTION_THRESHOLD) ? 
                        0.9f : 0.1f;
    
    // ������
    for(int i=0; i<3; i++) {
        filtered_data[i] = gf->adaptive_gain * filtered_data[i] + 
                          (1 - gf->adaptive_gain) * gf->last_output[i];
        gf->last_output[i] = filtered_data[i];
    }
}

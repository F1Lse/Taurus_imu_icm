#ifndef __KALMAN_FILTERS_H__
#define __KALMAN_FILTERS_H__

#ifndef  __KALMAN_FILTERS_H_GLOBALS
#define __KALMAN_FILTERS_EXT
#else
#define __KALMAN_FILTERS_EXT extern
#endif

#include "arm_math.h"
//#include "bsp_T_imu.h"
#include "stm32g4xx_hal.h"

#define mat         arm_matrix_instance_f32
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

//一阶卡尔曼滤波结构体定义,都是标量
typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
    float B;      //输入项参数
    float Q;      //状态方程噪声
    float R;      //观测方程噪声
    float H;
} Kalman1_param_t;

//二阶卡尔曼五条重要参数中间变量定义，用于计算，创建函数的第一个输入
typedef struct
{
    float raw_value;
    float filtered_value[2];     //卡尔曼滤波的返回值，0是位置1是速度
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman2_filter_t;

//二阶卡尔曼数组初始化，创建函数的第二个输入
typedef struct
{
    float raw_value;
    float filtered_value[2];        //卡尔曼滤波的返回值，0是位置1是速度
    float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];
} Kalman2_param_t;

#define TEST_LENGTH_SAMPLES  512    /* 采样点数 */
#define BLOCK_SIZE           512    	 /* 调用一次arm_lms_norm_f32处理的采样点个数 */
#define NUM_TAPS             20      /* 滤波器系数个数 */
typedef struct  //视觉目标速度测量
{
    int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
    int freq;
    int last_time;//上次受到目标角度的时间
    float last_position;//上个目标角度
    float speed;//速度
    float last_speed;//上次速度
    float processed_speed;//速度计算结果
} speed_calc_data_t;

void Kalman_init(void);

extern Kalman1_param_t kalman_yaw_angle_error;
extern Kalman1_param_t kalman_yaw_aim_speed;
extern Kalman1_param_t kalman_yaw_abs_speed;
extern Kalman1_param_t kalman_yaw_imu_speed;
extern Kalman1_param_t kalman_bullet_time;

extern Kalman1_param_t kalman_pit_angle_error;

extern Kalman1_param_t kalman_yaw_energy;
extern Kalman1_param_t kalman_pit_energy;
extern Kalman1_param_t kalman_gyro_z_stop;


extern kalman2_filter_t kalman2_pit_filter;
extern kalman2_filter_t kalman2_yaw_filter;
extern float *pit_kf_result;
extern float *yaw_kf_result;


extern kalman2_filter_t kalman2_test1_filter;
extern float *test1_kf_result;
extern kalman2_filter_t kalman2_test2_filter;
extern float *test2_kf_result;
extern float result;


__KALMAN_FILTERS_EXT void CreateKalman1Filter(Kalman1_param_t *p,float Q,float R);
__KALMAN_FILTERS_EXT void CreateKalman2Filter(kalman2_filter_t *F,  Kalman2_param_t *I);
__KALMAN_FILTERS_EXT float Kalman1Filter_calc(Kalman1_param_t* p,float dat);
__KALMAN_FILTERS_EXT float *Kalman2Filter_calc(kalman2_filter_t *F, float signal1, float signal2);
__KALMAN_FILTERS_EXT float Kalman2Filter_calc2(kalman2_filter_t *F, float signal1, float signal2);
__KALMAN_FILTERS_EXT float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
__KALMAN_FILTERS_EXT void arm_lms_f32_test2(void);
__KALMAN_FILTERS_EXT void kalman_R_calcu(void);
__KALMAN_FILTERS_EXT float my_Kalman1Filter_calc(Kalman1_param_t* p,float dat,float R);
//__KALMAN_FILTERS_EXT void Target_Accelerate_Calc(void);
#endif


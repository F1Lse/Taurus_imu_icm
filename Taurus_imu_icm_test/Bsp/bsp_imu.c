/** 
  * @file     bsp_imu.c
  * @version  v1.0
  * @date     2020.1.10
	*
  * @brief    IMU�������
	*
  *	@author   YY
  *
  */

#include "bsp_imu.h"
#include "KalmanFilter.h"
#include "Calibration.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "ICM42688_driver.h"
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f



imu_mode_e imu_mode;
//#define Kp 1.35f    // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.002f   // integral gain governs rate of convergence of gyroscope biases
extern float AccRatioOffset;
volatile float exInt, eyInt, ezInt;  // ������
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
//float halfT=0.001;	 //�������ڵ�һ��
float halfT = 0.0005;
float Kp=0.1f;
//float Ki=0.002f;
float Ki=0.01f;
float ahrs_count;
float ahrs_norm;
float ez_test;
float norm_test;
float imu_odom_vx,odom_x,last_time,d_time,time,last_time;
//float test_vz;
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;

volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ ms

float pitch_angle_test,kf2_pitch_angle;
float *kf2_pitch;


/**
  * @brief IMU_Values_Convert
  * @param 
  * @attention  
  * @note  IMU���ݵ�λ����
  */
void IMU_Values_Convert(void)
{
	
//	time = HAL_GetTick();
//	d_time = time-last_time;
//	imu_real_data.Gyro.X = imu_output_data.Gyro.X/16.384f/57.29577951308f; //���ٶȵ�λLSB->rad/sBMI088_AccelScale
//	imu_real_data.Gyro.Y = imu_output_data.Gyro.Y/16.384f/57.29577951308f; //���ٶȵ�λLSB->rad/sBMI088_AccelScale
//	imu_real_data.Gyro.Z = imu_output_data.Gyro.Z/16.384f/57.29577951308f; //���ٶȵ�λLSB->rad/sBMI088_AccelScale
////	imu_real_data.Accel.X = imu_output_data.Accel.X/1365.0f-Bias.Accel.X;
////	imu_real_data.Accel.Y = imu_output_data.Accel.Y/1365.0f-Bias.Accel.Y;
////	imu_real_data.Accel.Z = imu_output_data.Accel.Z/1365.0f-Bias.Accel.Z; 			//���ٶ�AD->g
//	imu_real_data.Accel.X = imu_output_data.Accel.X * BMI088_ACCEL_3G_SEN * BMI088_AccelScale;
//	imu_real_data.Accel.Y = imu_output_data.Accel.Y * BMI088_ACCEL_3G_SEN * BMI088_AccelScale;
//	imu_real_data.Accel.Z = imu_output_data.Accel.Z * BMI088_ACCEL_3G_SEN * BMI088_AccelScale;
//	last_time = time;
//	
//	
//	imu_real_data.Mag.X = -imu_output_data.Mag.X;
//	imu_real_data.Mag.Y = imu_output_data.Mag.Y;
//	imu_real_data.Mag.Z = imu_output_data.Mag.Z; //����������ϵ��һ

}





/*wanghongxi 方案*/
INS_t INS;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
 float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;



void IMU_AHRS_Calcu_task(void){

		bsp_IcmGetRawData(&IMU_Data);
		
    const float gravity[3] = {0, 0, 9.7833f};
//		dt = DWT_GetDeltaT(&INS_DWT_Count);
//    t += dt;
		dt = 0.001f;
    t += dt;
		
		INS.AccelLPF = 0.01f;
		
        INS.Accel[X_axis] = IMU_Data.Accel[X_axis];
        INS.Accel[Y_axis] =  IMU_Data.Accel[Y_axis];
        INS.Accel[Z_axis] =  IMU_Data.Accel[Z_axis];
        INS.Gyro[X_axis] = IMU_Data.Gyro[X_axis];
        INS.Gyro[Y_axis] = IMU_Data.Gyro[Y_axis];
        INS.Gyro[Z_axis] = IMU_Data.Gyro[Z_axis];

              // demo function,用于修正安装误差,可以不管,本demo暂时没用
        // IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        INS.atanxz = -atan2f(INS.Accel[X_axis], INS.Accel[Z_axis]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Y_axis], INS.Accel[Z_axis]) * 180 / PI;
				//扩展卡尔曼核心函数
        IMU_QuaternionEKF_Update(INS.Gyro[X_axis], INS.Gyro[Y_axis], INS.Gyro[Z_axis], INS.Accel[X_axis], INS.Accel[Y_axis], INS.Accel[Z_axis], dt); 
				
        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        
        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);


		    float gravity_b[3];    
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
				BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
                // 获取最终数据
        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;  
        //赋值给输出接口

}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

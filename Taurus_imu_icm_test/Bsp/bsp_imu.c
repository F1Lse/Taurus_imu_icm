/** 
  * @file     bsp_imu.c
  * @version  v1.0
  * @date     2024.10.21
	*
  * @brief    IMU����icm_42688
	*
  *	@author  	ZGH
  *
  */

#include "bsp_imu.h"
#include "KalmanFilter.h"
#include "Calibration.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "ICM42688_driver.h"
#include "pid.h"
#include "bsp_PWM.h"
#include "can_comm.h"

#define gNORM 9.83293118f
float icm088_AccelScale = 9.81f / gNORM;
/* ����ϵת���м���� */
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

/* У׼��־λ */
bias_gyro_mode_e bias_gyro_mode;


/* ״̬��־λ */
int16_t caliCount = 0;
uint8_t califlag = 0;
uint8_t imu_init;



extern 	float TempWheninit;
/* DWT��ʱ������ */
uint32_t INS_DWT_Count = 0; 
float dt = 0, t = 0;
float pitch_hubu,roll_hubu,yaw_hubu;
/* IMU�ṹ�� */
INS_t INS; 

void IMU_AHRS_Calcu_task(void){
		
		static uint32_t count;
		static int16_t temp;
		count++;
		
		
		if(!imu_init)
	  {PID_struct_init(&pid_temperature,POSITION_PID,2000, 300,1000, 20,0);		
		imu_init = 1;
		}
		bsp_IcmGetRawData(&IMU_Data);
		ICM42688P_ConvertToPhysical(&IMU_Data);
    const float gravity[3] = {0, 0, 9.81f};
		dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

		
		INS.AccelLPF = 0.0085f;
		
        INS.Accel[X_axis] = IMU_Data.Accel[X_axis]*icm088_AccelScale;
        INS.Accel[Y_axis] =  IMU_Data.Accel[Y_axis]*icm088_AccelScale;
        INS.Accel[Z_axis] =  IMU_Data.Accel[Z_axis]*icm088_AccelScale;
		
        INS.Gyro[X_axis] = IMU_Data.Gyro[X_axis];
        INS.Gyro[Y_axis] = IMU_Data.Gyro[Y_axis];
        INS.Gyro[Z_axis] = IMU_Data.Gyro[Z_axis];
		
		

        INS.atanxz = -atan2f(INS.Accel[X_axis], INS.Accel[Z_axis]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Y_axis], INS.Accel[Z_axis]) * 180 / PI;

        IMU_QuaternionEKF_Update(INS.Gyro[X_axis], INS.Gyro[Y_axis], INS.Gyro[Z_axis], INS.Accel[X_axis], INS.Accel[Y_axis], INS.Accel[Z_axis], dt); 
				
        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));


        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);


		    float gravity_b[3];    
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++)
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
				BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); 
             

				
        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;  
				
				//����������			
				//���������
				imu_msg_send.rol_msg.e.rol = INS.Roll;
				imu_msg_send.rol_msg.e.wx = INS.Gyro[X_axis];
				
				imu_msg_send.pit_msg.e.pit = INS.Pitch;
				imu_msg_send.pit_msg.e.wy = INS.Gyro[Y_axis];
				
				imu_msg_send.yaw_msg.e.yaw = INS.Yaw;
				imu_msg_send.yaw_msg.e.wz = INS.Gyro[Z_axis];
				//�����ͷ�
				
				imu_msg_send.gim_w_msg.e.wy = INS.Gyro[Y_axis]/gyro_scale;
				imu_msg_send.gim_w_msg.e.wz = INS.Gyro[Z_axis]/gyro_scale;
				
				imu_msg_send.gim_angle_msg.e.pit = INS.Pitch;
				imu_msg_send.gim_angle_msg.e.yaw = INS.Yaw;
				
				if(count % 2 == 0)
				{
					bsp_IcmGetTemperature(&temp);
					pid_calc(&pid_temperature,temp,IMU_Data.TempWhenCali);
//					TIM_Set_PWM(&htim1, TIM_CHANNEL_1,pid_temperature.pos_out);
				}
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


void Calibrate_MPU_Offset(IMU_Data_t *ICM42688)
{
	float Gyro_Bias_X,Gyro_Bias_Y,Gyro_Bias_Z,Accel_Bias_X,Accel_Bias_Y,Accel_Bias_Z;
	float gNormTemp;


	bias_gyro_mode = Calibration_successful_mode;
	
	for(uint16_t i=0;i<10000;i++)
	{
		bsp_IcmGetRawData(ICM42688);		

		if(fabs(ICM42688->Gyro_raw[0])>=25||fabs(ICM42688->Gyro_raw[1])>=25||fabs(ICM42688->Gyro_raw[2])>=25)
		{
		   bias_gyro_mode = Calibration_error_mode;
			
		}
		Gyro_Bias_X  += ICM42688->Gyro_raw[X_axis];
		Gyro_Bias_Y  += ICM42688->Gyro_raw[Y_axis];
		Gyro_Bias_Z  += ICM42688->Gyro_raw[Z_axis];
		
	  Accel_Bias_X += ICM42688->Accel_raw[X_axis];
		Accel_Bias_Y += ICM42688->Accel_raw[Y_axis];
		Accel_Bias_Z += ICM42688->Accel_raw[Z_axis];
		
		gNormTemp = sqrtf(ICM42688->Accel_raw[X_axis] * ICM42688->Accel_raw[X_axis] +
                              ICM42688->Accel_raw[Y_axis] * ICM42688->Accel_raw[Y_axis] +
                              ICM42688->Accel_raw[Z_axis] * ICM42688->Accel_raw[Z_axis]);
   ICM42688->gNorm += gNormTemp;
		
		bsp_IcmGetTemperature(&temp);
		pid_calc(&pid_temperature,temp,TempWheninit);
	 TIM_Set_PWM(&htim1, TIM_CHANNEL_1,pid_temperature.pos_out);		
		
		HAL_Delay(1);
	}
			bsp_IcmGetTemperature(&temp);
	 ICM42688->TempWhenCali = temp ;
	Accel_Bias_X = Accel_Bias_X/10000.0f;
	Accel_Bias_Y = Accel_Bias_Y/10000.0f;
	Accel_Bias_Z = Accel_Bias_Z/10000.0f;
	ICM42688->GyroOffset[X_axis] = Gyro_Bias_X/10000.0f;
	ICM42688->GyroOffset[Y_axis] = Gyro_Bias_Y/10000.0f;
	ICM42688->GyroOffset[Z_axis] = Gyro_Bias_Z/10000.0f;	
	ICM42688->gNorm /= (float)10000.0f;
	ICM42688->AccelScale = 9.78f / ICM42688->gNorm;
	
	ICM42688->Calidata[X_axis] = ICM42688->GyroOffset[X_axis];
		ICM42688->Calidata[Y_axis] = ICM42688->GyroOffset[Y_axis];
	ICM42688->Calidata[Z_axis] = ICM42688->GyroOffset[Z_axis];
		ICM42688->Calidata[Accel_c] = ICM42688->AccelScale;
	ICM42688->Calidata[Temp_Cali] = ICM42688->TempWhenCali;
	

	
	califlag = 1;
	

	
	HAL_Delay(50);
}




/**
 * @brief �����˲��㷨���ں������Ǻͼ��ٶȼ����ݣ�
 * @param imu      IMU���ݽṹ�壨����������Ǻͼ��ٶȼ����ݣ�
 * @param angles   �����̬�ǣ�pitch��roll����λ�����ȣ�
 * @param alpha    �����˲�ϵ����0.0~1.0���Ƽ�0.98��
 * @param dt       ����ʱ��������λ���룩
 */
void ComplementaryFilter(const IMU_Data_t *ICM42688 ,float alpha, float dt) {
    // ---------------------- 1. �Ӽ��ٶȼƼ�����̬�� ----------------------
    // ��һ�����ٶȼ����ݣ��������Լ��ٶ�Ӱ�죩
		float norm = sqrtf(ICM42688->Accel_raw[X_axis] * ICM42688->Accel_raw[X_axis] +
                              ICM42688->Accel_raw[Y_axis] * ICM42688->Accel_raw[Y_axis] +
                              ICM42688->Accel_raw[Z_axis] * ICM42688->Accel_raw[Z_axis]);
		
    float ax = ICM42688->Accel_raw[X_axis] / norm;
    float ay = ICM42688->Accel_raw[Y_axis] / norm;
    float az = ICM42688->Accel_raw[Z_axis] / norm;

    // ���㸩���ǣ�pitch���ͺ���ǣ�roll��
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float accel_roll  = atan2f( ay, az);

    // ---------------------- 2. �������ǻ��ּ�����̬�� ----------------------
    // ע�⣺�������������ȥ��ƫ��δ�ں����ڴ�������ǰУ׼��
    // ������ٶȻ��֣���̬�Ǳ仯��
    float gyro_pitch = pitch_hubu + ICM42688->Gyro_raw[Y_axis] * dt;  // ��Y����ת -> pitch
    float gyro_roll  = roll_hubu  + ICM42688->Gyro_raw[X_axis] * dt;  // ��X����ת -> roll

    // ---------------------- 3. �����˲��ں����� ----------------------
    // �ںϹ�ʽ���Ƕ� = �� * (�����ǻ��ֽǶ�) + (1-��) * (���ٶȼƽǶ�)
    pitch_hubu = alpha * gyro_pitch + (1 - alpha) * accel_pitch;
    roll_hubu  = alpha * gyro_roll  + (1 - alpha) * accel_roll;

    // ---------------------- 4. �Ƕȷ�ΧԼ������ѡ�� ----------------------
    // ���Ƕ������� -�� ~ �� ֮��
    pitch_hubu = atan2f(sinf(pitch_hubu), cosf(pitch_hubu)) * 180.0f / PI;
    roll_hubu  = atan2f(sinf(roll_hubu),  cosf(roll_hubu))  * 180.0f / PI;
}

// �����˲�����ŷ����
void calculate_euler_angles(IMU_Data_t *ICM42688,float alpha,float dt) {
    // ���ٶȼƽ��㸩���Ǻͺ����
    float acc_pitch = atan2(ICM42688->Accel_raw[Y_axis], sqrt(ICM42688->Accel_raw[X_axis] * ICM42688->Accel_raw[X_axis] + ICM42688->Accel_raw[Z_axis] * ICM42688->Accel_raw[Z_axis])) * 180.0f / PI;
    float acc_roll = atan2(-ICM42688->Accel_raw[X_axis], ICM42688->Accel_raw[Z_axis]) * 180.0f / PI;

    // �����ǻ��ּ���Ƕȱ仯
    roll_hubu += ICM42688->Gyro_raw[X_axis] * dt;
    pitch_hubu+= ICM42688->Gyro_raw[Y_axis] * dt;
    yaw_hubu += ICM42688->Gyro_raw[Z_axis] * dt;

    // �����˲��ں�����
    roll_hubu = alpha * roll_hubu + (1 - alpha) * acc_roll;
    pitch_hubu = alpha * pitch_hubu + (1 - alpha) * acc_pitch;

    // ƫ�����޷�ͨ�����ٶȼƽ��㣬��ʹ�������ǻ���
    // ע�⣺ƫ���ǻ���ʱ��Ư�ƣ���Ҫ�����ƻ�����������У��
}

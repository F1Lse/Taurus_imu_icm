/**
	* @file Filter.c
	* @version 1.0
	* @date 2019.12.1
  *
  * @brief  �˲�����
  *
  *	@author YY/BZW(Part of codes reference ��������)
  *
  */

#include "Filter.h"
#include "KalmanFilter.h"
/*******************************************************************************/
//------------------------Butterworth�˲��м�����-----------------------//
Butter_Parameter Gyro_Parameter;
Butter_Parameter Accel_Parameter;
Butter_BufferData gyro_filter_buf_bug[3],gyro_filter_buf[3];
Butter_BufferData accel_filter_buf[3];
//---------------------------IST�˲�buffer---------------------------//
float	Data_X_MAG[N2],Data_Y_MAG[N2],Data_Z_MAG[N2];
//----------------------------�˲�������-----------------------------//
S_FLOAT_XYZ	gyro_filter,accel_filter,mag_filter,gyro_filter_bug;
//----------------------------�˲�������-----------------------------//
Butter_Parameter Bandstop_Filter_Parameter_30_98={
  //200hz---30hz-98hz  ����-���
  1,   0.627040f,  -0.290527f,
  0.354737f,   0.627040f,    0.354737f
};
Butter_Parameter Bandstop_Filter_Parameter_30_94={
  //200hz---30hz-94hz  ����-���
  1,   0.5334540355829,  -0.2235264828971,
  0.3882367585514,   0.5334540355829,   0.3882367585514
};
/*******************************************************************************/

/***************************************************************/
/*
 * ��������BMI088_Filter
 * ����  ��BMI088�˲�����
 * ����  ����
 * ���  ����
 */ 
/***************************************************************/
float imu_Gyro_z;
void BMI088_Filter(void)
{


}

/***************************************************************/
/*
 * ��������IST8310_Filter
 * ����  ��IST8310�˲�����
 * ����  ����
 * ���  ����
 */ 
/***************************************************************/
void IST8310_Filter(void)
{

}

/****************************************
Butterworth��ͨ�˲���������ʼ����http://blog.csdn.net/u011992534/article/details/73743955
***************************************/
/***********************************************************
@��������Butterworth_Parameter_Init
@��ڲ�������
@���ڲ�������
����������������˹��ͨ�˲�����ʼ��
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
void Butterworth_Parameter_Init(void)
{

}

/*************************************************
������:	LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
˵��:	���ٶȼƵ�ͨ�˲���
���:	float curr_input ��ǰ������ٶȼ�,�˲����������˲�������
����:	��
��ע:	2��Butterworth��ͨ�˲���
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* ���ٶȼ�Butterworth�˲� */
  /* ��ȡ����x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth�˲� */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) ���б��� */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) ���б��� */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}

/***********************************************************
@��������Set_Cutoff_Frequency
@��ڲ�����float ����Ƶ��, float ��ֹƵ��,
Butter_Parameter *LPF
@���ڲ�������
����������������˹��ͨ�˲�����ʼ��
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(PI / fr);
  float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
}

/***********************************************************
@��������GildeAverageValueFilter_MAG
@��ڲ�����float NewValue,float *Data
@���ڲ�������
�������������������˲�
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
float GildeAverageValueFilter_MAG(float NewValue,float *Data)
{
  float max,min;
  float sum;
  unsigned char i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=N2-1;i!=0;i--)
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];
  }
  i=N2-2;
  sum=sum-max-min;
  sum=sum/i;
  return(sum);
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/



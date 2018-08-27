#include "delay.h"
#include "value.h"



//typedef struct
//{
//	int16_t Top_Mode;//�ϲ�ģʽָ��
//	int16_t Top_Direction;//�ϲ㷽��ָ��
//	int16_t Top_Speed ;//�ϲ��ٶ�ָ��
//	int16_t Top_Yaw;   //�ϲ�Yaw����תָ��
//	int16_t Top_Pitch; //�ϲ�Pitch��Ƕ�ָ��
//	int16_t Top_dx;    //�ϲ�ͼ������X���ֵ
//	int16_t Top_dy;    //�ϲ�ͼ������Y���ֵ

//}Top_Data_t;

typedef struct
{
  int16_t ax;
	int16_t ay;
	int16_t az;
	
	int16_t mx;
	int16_t my;
	int16_t mz;
	
	int16_t temp;
	
	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;

}mpu_data_t;


typedef struct
{
  int16_t ax;
	int16_t ay;
	int16_t az;
	
	int16_t mx;
	int16_t my;
	int16_t mz;
	
	float temp;
	float temp_ref;
	
	float wx;
	float wy;
	float wz;
	
	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;

}imu_data_t;


extern mpu_data_t     mpu_data;
extern imu_data_t     imu;





uint8_t mpu_device_init(void);
void mpu_get_data(void);
void mpu_offset_cal(void);











#include "value.h"



//����ģʽ
int control_mode ;

//���̵��������
s16 chassis_Motor_M1[4]  , chassis_Motor_M2[4]  , chassis_Motor_M3[4]  , chassis_Motor_M4[4];
s16 chassis_Motor_M1ofset, chassis_Motor_M2ofset, chassis_Motor_M3ofset, chassis_Motor_M4ofset;
s16 chassis_Motor_M1_last, chassis_Motor_M2_last, chassis_Motor_M3_last, chassis_Motor_M4_last;
s16 chassis_Motor_M1_cnt , chassis_Motor_M2_cnt , chassis_Motor_M3_cnt , chassis_Motor_M4_cnt;
int32_t chassis_Motor_M1_buf[5] , chassis_Motor_M2_buf[5] , chassis_Motor_M3_buf[5] , chassis_Motor_M4_buf[5];
float chassis_Motor1_loco, chassis_Motor2_loco  , chassis_Motor3_loco  , chassis_Motor4_loco;
float chassis_Motor1_loco_last, chassis_Motor2_loco_last  , chassis_Motor3_loco_last  , chassis_Motor4_loco_last;
s16 chassis_Motor_M1_set , chassis_Motor_M2_set , chassis_Motor_M3_set , chassis_Motor_M4_set;

//��̨���ٶȣ�
float gimbal_speed_yaw , gimbal_speed_pitch;
float gimbal_speed_yaw_set , gimbal_speed_pitch_set;

//��̨�Ƕȣ�
float yaw_angle;
float pitch_angle;
float pitch_angle_set ;
//Yaw��Ƕ�
float ZGyroModuleAngle , Gyro_palstance;
float yaw_angle_set;

int Yaw_motorloco_init = 6453;
int Pitch_motorloco_init = 2800;


//��̨���������
s16 Yaw_Motor[4] , Pitch_Motor[4];

//�������������
s16 Trigger_Motor[4];
s16 Trigger_Motor_Set ;

//LOOP��ʱcnt
int Tim_circle_flag_1ms , Tim_circle_flag_2ms , Tim_circle_flag_3ms;

//�����ϲ�ָ�
char communication_temp;

//�����ϲ����
u8 automatic_control_mode  ;
s16 top_Mode;//�ϲ�ģʽָ��
s16 top_Direction;//�ϲ㷽��ָ��
s16 top_Speed ;//�ϲ��ٶ�ָ��
s16 top_Yaw;   //�ϲ�Yaw����תָ��
s16 top_Pitch; //�ϲ�Pitch��Ƕ�ָ��
s16 top_dx;    //�ϲ�ͼ������X���ֵ
s16 top_dy;    //�ϲ�ͼ������Y���ֵ





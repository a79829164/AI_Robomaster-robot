#include "stm32f4xx.h" 


//����ģʽ
extern int control_mode ;

//���̵����
extern s16 chassis_Motor_M1[4]  , chassis_Motor_M2[4]  , chassis_Motor_M3[4]  , chassis_Motor_M4[4];
extern s16 chassis_Motor_M1ofset, chassis_Motor_M2ofset, chassis_Motor_M3ofset, chassis_Motor_M4ofset;
extern s16 chassis_Motor_M1_last, chassis_Motor_M2_last, chassis_Motor_M3_last, chassis_Motor_M4_last;
extern s16 chassis_Motor_M1_cnt , chassis_Motor_M2_cnt , chassis_Motor_M3_cnt , chassis_Motor_M4_cnt;
extern int32_t chassis_Motor_M1_buf[5] , chassis_Motor_M2_buf[5] , chassis_Motor_M3_buf[5] , chassis_Motor_M4_buf[5];
extern float chassis_Motor1_loco, chassis_Motor2_loco  , chassis_Motor3_loco  , chassis_Motor4_loco;
extern float chassis_Motor1_loco_last, chassis_Motor2_loco_last  , chassis_Motor3_loco_last  , chassis_Motor4_loco_last;
extern s16 chassis_Motor_M1_set , chassis_Motor_M2_set , chassis_Motor_M3_set , chassis_Motor_M4_set; 

//��̨���ٶȣ�
extern float gimbal_speed_yaw , gimbal_speed_pitch;
extern float gimbal_speed_yaw_set , gimbal_speed_pitch_set;

//��̨�Ƕȣ�
extern float yaw_angle;
extern float pitch_angle;
extern float pitch_angle_set;
//��̨���������
extern s16 Yaw_Motor[4] , Pitch_Motor[4];

extern int Yaw_motorloco_init ;
extern int Pitch_motorloco_init ;

//�������������
extern s16 Trigger_Motor[4];
extern s16 Trigger_Motor_Set ;

//LOOP��ʱcnt
extern int Tim_circle_flag_1ms , Tim_circle_flag_2ms , Tim_circle_flag_3ms;

//Yaw��Ƕ�
extern float ZGyroModuleAngle , Gyro_palstance;
extern float yaw_angle_set;

//�����ϲ�ָ�
extern char communication_temp;
extern u8 automatic_control_mode  ;
extern s16 top_Mode;//�ϲ�ģʽָ��
extern s16 top_Direction;//�ϲ㷽��ָ��
extern s16 top_Speed ;//�ϲ��ٶ�ָ��
extern s16 top_Yaw;   //�ϲ�Yaw����תָ��
extern s16 top_Pitch; //�ϲ�Pitch��Ƕ�ָ��
extern s16 top_dx;    //�ϲ�ͼ������X���ֵ
extern s16 top_dy;    //�ϲ�ͼ������Y���ֵ



 

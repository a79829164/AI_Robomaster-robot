#include "value.h"



//����ģʽ
int control_mode ;

//��̼��ٶȣ�
float speed_x , speed_y;

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
float ZGyroModule_realAngle ;
float yaw_angle_set;

//�Զ�ģʽ�µĽǶ�ƫ�
float yaw_angle_dirta;
float pitch_angle_dirta;

int Yaw_motorloco_init = 6164;
int Pitch_motorloco_init = 2800;
int Pitch_horizontal_level = 3050;
int Yaw_motor_realloco ;
int Pitch_motor_realloco ;

//��̨���������
s16 Yaw_Motor[4] , Pitch_Motor[4];
u8 Search_enemy_turning_flag ;
float fixed_turn_angle_value = 0;

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
float top_distance;    //�ϲ�ͼ������X���ֵ
s16 top_dy;    //�ϲ�ͼ������Y���ֵ

//Ŀ��ƽ�Ƶ�����ٶȣ�
float target_yawmove_speed;
float target_yawmove_speed_para = 0.08;

//�����ϲ����
char tx_buf_toROS[36];

//���������������
u8 last_rc_s1 = 0;
u8 shoot_status = 0;//���״̬
int TIM2_CCR1_value = 83 ;  // 80+900->16.77m/s / 67+700->3.47m/s / 66 + 740 -> 5.6m/s
int TIM4_CCR1_value = 925; //
int Trigger_motor_frequence = 2500;
u8 KEY_STATE , LAST_KEY_STATE;//����Ϊ0 �����ӵ�Ϊ1
u8 shoot_start_flag = 0;//���������־
u8 enemy_found_flag ;//����Ŀ���־
int shoot_bullet_cnt = 0 , shoot_bullet_everycnt;//����ӵ���
int shoot_frequence_cnt , shoot_frequence_cnt;

//�Զ�ģʽ�µ��˶�������
s16 move_for_behind_value , move_left_right_value;


//�����㷨������������
//ǰ�������״�����룺
float yaw_angle_2D; 

//���е��ٶȣ�
int grab_centerpoint_speedvalue = 220; // 210
int grab_centerpoint_TIMER      = 1000;






//void  dwffef()
//{
//	floatnum.num = 1424.35353;
//}
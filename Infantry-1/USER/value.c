#include "value.h"



//控制模式
int control_mode ;

//里程计速度：
float speed_x , speed_y;

//底盘电机参数：
s16 chassis_Motor_M1[4]  , chassis_Motor_M2[4]  , chassis_Motor_M3[4]  , chassis_Motor_M4[4];
s16 chassis_Motor_M1ofset, chassis_Motor_M2ofset, chassis_Motor_M3ofset, chassis_Motor_M4ofset;
s16 chassis_Motor_M1_last, chassis_Motor_M2_last, chassis_Motor_M3_last, chassis_Motor_M4_last;
s16 chassis_Motor_M1_cnt , chassis_Motor_M2_cnt , chassis_Motor_M3_cnt , chassis_Motor_M4_cnt;
int32_t chassis_Motor_M1_buf[5] , chassis_Motor_M2_buf[5] , chassis_Motor_M3_buf[5] , chassis_Motor_M4_buf[5];
float chassis_Motor1_loco, chassis_Motor2_loco  , chassis_Motor3_loco  , chassis_Motor4_loco;
float chassis_Motor1_loco_last, chassis_Motor2_loco_last  , chassis_Motor3_loco_last  , chassis_Motor4_loco_last;
s16 chassis_Motor_M1_set , chassis_Motor_M2_set , chassis_Motor_M3_set , chassis_Motor_M4_set;

//云台角速度：
float gimbal_speed_yaw , gimbal_speed_pitch;
float gimbal_speed_yaw_set , gimbal_speed_pitch_set;

//云台角度：
float yaw_angle;
float pitch_angle;
float pitch_angle_set ;
//Yaw轴角度
float ZGyroModuleAngle , Gyro_palstance;
float ZGyroModule_realAngle ;
float yaw_angle_set;

//自动模式下的角度偏差：
float yaw_angle_dirta;
float pitch_angle_dirta;

int Yaw_motorloco_init = 6164;
int Pitch_motorloco_init = 2800;
int Pitch_horizontal_level = 3050;
int Yaw_motor_realloco ;
int Pitch_motor_realloco ;

//云台电机参数：
s16 Yaw_Motor[4] , Pitch_Motor[4];
u8 Search_enemy_turning_flag ;
float fixed_turn_angle_value = 0;

//拨弹电机参数：
s16 Trigger_Motor[4];
s16 Trigger_Motor_Set ;

//LOOP定时cnt
int Tim_circle_flag_1ms , Tim_circle_flag_2ms , Tim_circle_flag_3ms;

//来自上层指令：
char communication_temp;

//来自上层命令：
u8 automatic_control_mode  ;
s16 top_Mode;//上层模式指令
s16 top_Direction;//上层方向指令
s16 top_Speed ;//上层速度指令
s16 top_Yaw;   //上层Yaw轴旋转指令
s16 top_Pitch; //上层Pitch轴角度指令
float top_distance;    //上层图像数据X轴差值
s16 top_dy;    //上层图像数据Y轴差值

//目标平移的相对速度：
float target_yawmove_speed;
float target_yawmove_speed_para = 0.08;

//发给上层命令：
char tx_buf_toROS[36];

//射击参数及变量：
u8 last_rc_s1 = 0;
u8 shoot_status = 0;//射击状态
int TIM2_CCR1_value = 83 ;  // 80+900->16.77m/s / 67+700->3.47m/s / 66 + 740 -> 5.6m/s
int TIM4_CCR1_value = 925; //
int Trigger_motor_frequence = 2500;
u8 KEY_STATE , LAST_KEY_STATE;//上膛为0 ，无子弹为1
u8 shoot_start_flag = 0;//射击启动标志
u8 enemy_found_flag ;//发现目标标志
int shoot_bullet_cnt = 0 , shoot_bullet_everycnt;//射出子弹数
int shoot_frequence_cnt , shoot_frequence_cnt;

//自动模式下的运动变量：
s16 move_for_behind_value , move_left_right_value;


//导航算法参数及变量：
//前方激光雷达测距距离：
float yaw_angle_2D; 

//抢中点速度：
int grab_centerpoint_speedvalue = 220; // 210
int grab_centerpoint_TIMER      = 1000;






//void  dwffef()
//{
//	floatnum.num = 1424.35353;
//}
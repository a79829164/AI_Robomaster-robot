#include "stm32f4xx.h" 


 
//控制模式
extern int control_mode ;

//里程计速度：
extern float speed_x , speed_y;

//底盘电机：
extern s16 chassis_Motor_M1[4]  , chassis_Motor_M2[4]  , chassis_Motor_M3[4]  , chassis_Motor_M4[4];
extern s16 chassis_Motor_M1ofset, chassis_Motor_M2ofset, chassis_Motor_M3ofset, chassis_Motor_M4ofset;
extern s16 chassis_Motor_M1_last, chassis_Motor_M2_last, chassis_Motor_M3_last, chassis_Motor_M4_last;
extern s16 chassis_Motor_M1_cnt , chassis_Motor_M2_cnt , chassis_Motor_M3_cnt , chassis_Motor_M4_cnt;
extern int32_t chassis_Motor_M1_buf[5] , chassis_Motor_M2_buf[5] , chassis_Motor_M3_buf[5] , chassis_Motor_M4_buf[5];
extern float chassis_Motor1_loco, chassis_Motor2_loco  , chassis_Motor3_loco  , chassis_Motor4_loco;
extern float chassis_Motor1_loco_last, chassis_Motor2_loco_last  , chassis_Motor3_loco_last  , chassis_Motor4_loco_last;
extern s16 chassis_Motor_M1_set , chassis_Motor_M2_set , chassis_Motor_M3_set , chassis_Motor_M4_set; 

//云台角速度：
extern float gimbal_speed_yaw , gimbal_speed_pitch;
extern float gimbal_speed_yaw_set , gimbal_speed_pitch_set;

//云台角度：
extern float yaw_angle;
extern float pitch_angle;
extern float pitch_angle_set;

//自动模式下的角度偏差：
extern float yaw_angle_dirta;
extern float pitch_angle_dirta;

//云台电机参数：
extern s16 Yaw_Motor[4] , Pitch_Motor[4];
extern u8 Search_enemy_turning_flag ;

extern int Yaw_motorloco_init ;
extern int Pitch_motorloco_init ;
extern int Pitch_horizontal_level ;
extern int Yaw_motor_realloco ;
extern int Pitch_motor_realloco ;

//拨弹电机参数：
extern s16 Trigger_Motor[4];
extern s16 Trigger_Motor_Set ;

//LOOP定时cnt
extern int Tim_circle_flag_1ms , Tim_circle_flag_2ms , Tim_circle_flag_3ms;

//Yaw轴角度
extern float ZGyroModuleAngle , Gyro_palstance;
extern float ZGyroModule_realAngle ;
extern float yaw_angle_set;

//来自上层指令：
extern char communication_temp;
extern u8 automatic_control_mode  ;
extern s16 top_Mode;//上层模式指令
extern s16 top_Direction;//上层方向指令
extern s16 top_Speed ;//上层速度指令
extern s16 top_Yaw;   //上层Yaw轴旋转指令
extern s16 top_Pitch; //上层Pitch轴角度指令
extern float top_distance;    //上层图像数据X轴差值
extern s16 top_dy;    //上层图像数据Y轴差值

//目标平移的相对速度：
extern float target_yawmove_speed;
extern float target_yawmove_speed_para  ;

//发给上层命令：
extern char tx_buf_toROS[36];

//射击参数及变量：
extern u8 last_rc_s1 ;
extern u8 shoot_status ;
extern int shoot_speed_value ;  // 80+900->16.77m/s / 67+700->3.47m/s / 66 + 740 -> 5.6m/s
 
extern int Trigger_motor_frequence ;
extern u8 KEY_STATE , LAST_KEY_STATE;//上膛为0 ，无子弹为1
extern u8 shoot_start_flag ;
extern u8 enemy_found_flag ;//发现目标标志
extern int shoot_bullet_cnt , shoot_bullet_everycnt;
extern int shoot_frequence_cnt , shoot_frequence_cnt;


//自动模式下的运动变量：
extern s16 move_for_behind_value , move_left_right_value;

//比赛开始标志：
extern int game_start_flag , game_start_cnt;


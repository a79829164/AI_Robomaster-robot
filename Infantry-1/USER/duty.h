#include "value.h"
#include "imu_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"


                
 
#define BSP_USART1_RX_BUF_SIZE_IN_FRAMES         (BSP_USART1_RX_BUF_SIZE / RC_FRAME_LENGTH)
#define  RC_FRAME_LENGTH                            18u
// 
//#define uwb_Selfcheck_flag      0        //1为刚上电  0为没断电
#define pidcalc_start 1000

#define CAN_3510_M1_ID          0x201
#define CAN_3510_M2_ID          0x202
#define CAN_3510_M3_ID          0x203
#define CAN_3510_M4_ID          0x204
#define CAN_YAW_MOTOR_ID        0x205
#define CAN_PITCH_MOTOR_ID      0x206
#define CAN_TRIGGER_MOTOR_ID    0x207

#define FILTER_BUF 5

#define CAN_GIMBAL_ZGYRO_ID     0x401
#define UWB_LOCALIZATION_ID     0x259


void loop(void);

void automatic_control(void);
void ranger_movement(void);
void attack_movement(void) ;










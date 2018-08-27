#include "stm32f4xx.h" 
#include "dataprocess.h"
#include "value.h"


#define CAN_3510_M1_ID          0x201
#define CAN_3510_M2_ID          0x202
#define CAN_3510_M3_ID          0x203
#define CAN_3510_M4_ID          0x204
#define CAN_YAW_MOTOR_ID        0x205
#define CAN_PITCH_MOTOR_ID      0x206
#define CAN_TRIGGER_MOTOR_ID    0x207



#define FILTER_BUF 5


void Send_motorvalue_CAN1_4(s16 data1 , s16 data2 , s16 data3 , s16 data4);
void Send_motorvalue_CAN1_3(s16 data1 , s16 data2 , s16 data3 , s16 data4);

void Can1_Configuration(void);



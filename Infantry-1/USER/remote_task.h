#include "stm32f4xx.h" 
#include "value.h"


#define NULL 0
#define REMOTE_INPUT 1
#define STOP 3
#define AUTOMATIC_INPUT 2

////////////////
#define ranger_mode 1
#define attack_mode 2
//#define STOP 2


typedef struct
{
 uint16_t ch0;
 uint16_t ch1;
 uint16_t ch2;
 uint16_t ch3;	
	
 uint16_t s1;
 uint16_t s2;	
}rc_t;

typedef struct
{
 uint16_t x;
 uint16_t y;
 uint16_t z;
 uint16_t press_l; 
 uint16_t press_r;	
}mouse_t;

typedef struct
{
 uint8_t v;
}key_t;

typedef struct
{
 rc_t rc;
 mouse_t mouse;
 key_t key;	
}RC_CtrlData_t;

extern RC_CtrlData_t RC_CtrlData;
extern float yaw_angle_set;
extern float pitch_angle_set ;
extern float ZGyroModuleAngle , Gyro_palstance;

void RemoteDataProcess(uint8_t *pData);
void remote_control(void);





#include "stm32f4xx.h" 
#include "value.h"

typedef struct
{
	int16_t x;
	int16_t y;
}point_t;

 

typedef struct
{
	//地图初始化信息：
	int16_t Map_X;// 
	int16_t Map_Y;// 

	////////
	point_t point_init;
	point_t point1;
	point_t point2; 
	point_t point3;
	point_t point4;
	point_t point5; 
	point_t point6; 
	point_t point7;
	point_t point8; 
	point_t point9;
	point_t point10;
	point_t point11; 
	point_t point12;	
	point_t point13;
	point_t point14; 
	point_t point15;
	point_t point16;
	point_t point17; 
	point_t point18; 
	point_t point19;
	point_t point20; 
	point_t point21;
	point_t point22;
	point_t point23; 
	point_t point24;
	
	//机器人位置信息：
	int16_t Robot_Map_targetX;// 
	int16_t Robot_Map_targetY;// 
 	int16_t Robot_Map_dX;// 
	int16_t Robot_Map_dY;// 	
	
}Map_data_t;


extern Map_data_t Map_data;

int f(int a);
void Set_Initpoint_forNavigation(void);
void field_navigation_apply(void);


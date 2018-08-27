#include "navigation_task.h"
#include "can2.h"
#include "remote_task.h"


int f(int a)
{
	if(a >= 0)
	{
		return a;
	}
	else
	{
		return -a;
	}
}

Map_data_t Map_data;
void Map_data_init(void)
{
	Map_data.Map_X = 800;
	Map_data.Map_Y = 500;
 ////////////////////////////////
		
//	Map_data.point1.x = 112;
//	Map_data.point1.y = 296;
//	Map_data.point2.x = 104;
//	Map_data.point2.y = 192;
//	
//	Map_data.point3.x = 139;
//	Map_data.point3.y = 310;
//	Map_data.point4.x = 239;
//	Map_data.point4.y = 311;
//	Map_data.point5.x = 120;
//	Map_data.point5.y = 125;
//	Map_data.point6.x = 240;
//	Map_data.point6.y = 120;
//	
//	Map_data.point7.x = 63;
//	Map_data.point7.y = 441;
//	Map_data.point8.x = 220;
//	Map_data.point8.y = 432;
//	Map_data.point9.x = 73;
//	Map_data.point9.y = 345;
//	Map_data.point10.x = 240;
//	Map_data.point10.y = 320;
//	
//	Map_data.point11.x = 250;
//	Map_data.point11.y = 250;
//	Map_data.point12.x = 357;
//	Map_data.point12.y = 238;
//	
//	Map_data.point13.x = 400;
//	Map_data.point13.y = 267;
//	Map_data.point14.x = 510;
//	Map_data.point14.y = 250;

//	Map_data.point15.x = 520;
//	Map_data.point15.y = 390;
//	Map_data.point16.x = 635;
//	Map_data.point16.y = 375;
//	Map_data.point17.x = 530;
//	Map_data.point17.y = 210;
//	Map_data.point18.x = 640;
//	Map_data.point18.y = 200;

//	Map_data.point19.x = 535;
//	Map_data.point19.y = 167;
//	Map_data.point20.x = 700;
//	Map_data.point20.y = 175;
//	Map_data.point21.x = 532;
//	Map_data.point21.y = 80;
//	Map_data.point22.x = 710;
//	Map_data.point22.y = 78;	
//	
//  Map_data.point23.x = 660;
//	Map_data.point23.y = 335;
//	Map_data.point24.x = 661;
//	Map_data.point24.y = 222;
	/////////////////////////////////////
	
	Map_data.Robot_Map_targetX = 0;
	Map_data.Robot_Map_targetY = 0;
}

u8 Set_Navipoint_status , Set_Navipoint_cnt;
void Set_Initpoint_forNavigation(void)
{
	if(RC_CtrlData.rc.s2 == REMOTE_INPUT)
	{
		if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && Set_Navipoint_status == 0) 
		{
			Set_Navipoint_status = 1;
		}
		else if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && Set_Navipoint_status == 1) 
		{
			Set_Navipoint_status = 0;
		}
		if(RC_CtrlData.rc.s1 == 2 && last_rc_s1 == 3 && Set_Navipoint_status == 1)  
		{
			Set_Navipoint_cnt++;
			
			switch(Set_Navipoint_cnt)
			{
				case 1: 
				{
					Map_data.point1.x = Uwb_data.x_uwb;
					Map_data.point1.y = Uwb_data.y_uwb;
				}
				break;
				
				case 2: 
				{
					Map_data.point2.x = Uwb_data.x_uwb;
					Map_data.point2.y = Uwb_data.y_uwb;
				}
				break;
				
				case 3: 
				{
					Map_data.point3.x = Uwb_data.x_uwb;
					Map_data.point3.y = Uwb_data.y_uwb;
				}
				break;
				
				case 4: 
				{
					Map_data.point4.x = Uwb_data.x_uwb;
					Map_data.point4.y = Uwb_data.y_uwb;
				}
				break;
				
				case 5: 
				{
					Map_data.point5.x = Uwb_data.x_uwb;
					Map_data.point5.y = Uwb_data.y_uwb;
				}
				break;
				
				case 6: 
				{
					Map_data.point6.x = Uwb_data.x_uwb;
					Map_data.point6.y = Uwb_data.y_uwb;
				}
				break;
				
				case 7: 
				{
					Map_data.point7.x = Uwb_data.x_uwb;
					Map_data.point7.y = Uwb_data.y_uwb;
				}
				break;
				
				case 8: 
				{
					Map_data.point8.x = Uwb_data.x_uwb;
					Map_data.point8.y = Uwb_data.y_uwb;
				}
				break;
				
				case 9: 
				{
					Map_data.point9.x = Uwb_data.x_uwb;
					Map_data.point9.y = Uwb_data.y_uwb;
				}
				break;
				
				case 10: 
				{
					Map_data.point10.x = Uwb_data.x_uwb;
					Map_data.point10.y = Uwb_data.y_uwb;
				}
				break;
			}
		}
	}
	last_rc_s1 = RC_CtrlData.rc.s1;
		
}
	
//全场定位导航算法：

//1.记录每个障碍块四个顶点的坐标,机器人底盘方向始终恒定

//2.以出发点为原点，在第一象限中求出以目标点和当前位置 确定的直线方程

//3.利用这个一维方程将每个顶点遍历一遍，即找出直线穿过的障碍块，确定路径上的位置点（判断点是否位于该直线上）

//4.根据要经过的位置点，将这些坐标排序，让机器人依次按这些坐标运动，找到最佳路径


void navigation_algorithm(void)
{
	float k;//直线斜率
	
  
	
}



u8 location_point_cnt = 4 , X_arrive_flag = 0 , Y_arrive_flag = 0;
void field_navigation_apply(void)
{
 
//	//////////////////////
//	Map_data.Robot_Map_dX = Map_data.Robot_Map_targetX - Uwb_data.x_uwb;
//	Map_data.Robot_Map_dY = Map_data.Robot_Map_targetY - Uwb_data.y_uwb;	

	
	//到达第一个点：
	if(location_point_cnt == 1)
	{
		Map_data.Robot_Map_targetX = Map_data.point1.x;
		Map_data.Robot_Map_targetY = Map_data.point1.y;
		Map_data.Robot_Map_dX = Map_data.Robot_Map_targetX - Uwb_data.x_uwb;
  	Map_data.Robot_Map_dY = Map_data.Robot_Map_targetY - Uwb_data.y_uwb;	
		
		move_for_behind_value = 150;
		move_left_right_value = 0;//Map_data.Robot_Map_dY * -1;
	}
	if(location_point_cnt == 1 && f(Map_data.Robot_Map_dX) <= 5)
	{
		location_point_cnt = 2;
	}

	//到达第三个点：
	if(location_point_cnt == 2)
	{
		Map_data.Robot_Map_targetX = Map_data.point2.x;
		Map_data.Robot_Map_targetY = Map_data.point2.y;
		Map_data.Robot_Map_dX = Map_data.Robot_Map_targetX - Uwb_data.x_uwb;
  	Map_data.Robot_Map_dY = Map_data.Robot_Map_targetY - Uwb_data.y_uwb;	
		
		move_left_right_value = -150;
		move_for_behind_value = 0;//Map_data.Robot_Map_dX * 1;
	}
	if(location_point_cnt == 2 && f(Map_data.Robot_Map_dY) <= 5)
	{
		location_point_cnt = 3;
	}		
	
	//到达第五个点：
	if(location_point_cnt == 3)
	{
		Map_data.Robot_Map_targetX = Map_data.point3.x;
		Map_data.Robot_Map_targetY = Map_data.point3.y;
		Map_data.Robot_Map_dX = Map_data.Robot_Map_targetX - Uwb_data.x_uwb;
  	Map_data.Robot_Map_dY = Map_data.Robot_Map_targetY - Uwb_data.y_uwb;	
		
		move_for_behind_value = 150;
		move_left_right_value = Map_data.Robot_Map_dY * -2;
	}
	if(location_point_cnt == 3 && f(Map_data.Robot_Map_dX) <= 5)
	{
		location_point_cnt = 4;
	}		
	if(location_point_cnt == 4)
	{
	  Map_data.Robot_Map_targetX = Map_data.point3.x;
		Map_data.Robot_Map_targetY = Map_data.point3.y;
		Map_data.Robot_Map_dX = Map_data.Robot_Map_targetX - Uwb_data.x_uwb;
  	Map_data.Robot_Map_dY = Map_data.Robot_Map_targetY - Uwb_data.y_uwb;	
		
		move_for_behind_value = Map_data.Robot_Map_dX *  2;
		move_left_right_value = Map_data.Robot_Map_dY * -2;
	}
	 
	
}
	 
	
	
	
	
	
	
	
	
	
	
	
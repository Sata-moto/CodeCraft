#pragma once

struct Desk
{
	int type;				//工作台的类型
	double x, y;			//工作台的坐标
	int remain_time = -1;	//工作台剩余生产时间（帧数，-1 代表没有生产，0 代表因为输出格满了而阻塞）
	bool input_status[7];	//工作台原材料格状态（1 表示有物品），
	bool output_status;		//工作台输出格状态（1 表示有物品）
};
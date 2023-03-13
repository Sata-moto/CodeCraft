#pragma once

struct Car
{
	int workbench, goods;	                 // 所处工作台编号，携带物品编号
	double timerate, hitrate;                // 时间价值系数，碰撞价值系数
	double vx, vy, w;		                 // 二维线速度向量，角速度
	double ang, x, y;		                 // 朝向角度，坐标
	double CalcAng(double, double);          // 计算当前朝向与目标点的偏向角
	double CalcForward(double);              // 根据偏向角计算前进速度
	void MarginCheck(double&);                      // 
	pair<double, double> mov(double, double);// 将小车向目标点移动
};

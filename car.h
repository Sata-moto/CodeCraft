#pragma once

struct Car{
	int workbench,goods;//所处工作台编号，携带物品编号 
	double timerate,hitrate;//时间价值系数，碰撞价值系数 
	double vx,vy,w;//二维线速度向量，角速度 
	double ang,x,y;//朝向角度，坐标 
	pair<double,double> mov(double,double);
};

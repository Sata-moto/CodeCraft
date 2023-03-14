#pragma once

struct Car
{
	int workbench, goods;                        // 所处工作台编号，携带物品编号
	double timerate, hitrate;                    // 时间价值系数，碰撞价值系数
	double vx, vy, w;                            // 二维线速度向量，角速度
	double ang, x, y;                            // 朝向角度，坐标
	double Sign(double);                         // 提取输入参数的符号
	double Dot(double, double, double, double);  // 点积
	double Cross(double, double, double, double);// 叉积
	double Dist(double, double, double, double); // 计算两个点之间的距离
	double GetR(int);                            // 根据是否持有物品返回当前半径
	double CombineV(double, double);             // 计算合速度（非负）
	double CalcAng(double, double);              // 计算当前朝向与目标点的偏向角
	double CalcRotate(double, double, double);   // 根据偏向角计算角速度
	double CalcForward(double);                  // 根据偏向角计算前进速度
	void CarCrashCheck(double&, double&);        // 小车碰撞判断并修改速度与角速度
	void MarginCheck(double&);                   // 边界碰撞判断并修改前进速度 
	pair<double, double> mov(double, double);    // 将小车向目标点移动
};
<<<<<<< HEAD
extern Car car[5];
=======
extern Car car[5];
>>>>>>> 6cb5cf973f8ba906006b5bae05433831a67f9223

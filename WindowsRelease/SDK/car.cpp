#include "Global.h"
#include "car.h"

using namespace std;
const double eps = 1e-2;
const double Pi = 3.1415926536;
double Car::CalcAng(double nx, double ny) {
	double res = atan2(ny - y, nx - x) - ang;
	if (res >= Pi)
		res -= 2 * Pi;
	if (res <= -Pi)
		res += 2 * Pi;
	return res;
}
double Car::CalcForward(double DeltaAng) {
	return cos(DeltaAng) * (fabs(DeltaAng) > Pi / 2 ? 0 : 6);//可以调整这里使用的函数
}
void Car::MarginCheck(double& forwar) {
	//计算质量和加速度
	double M = (goods == 0 ? 0.2025 : 0.2809) * Pi * 20, A = 250.0 / M, Ax, Ay;
	//加速度矢量分解
	if (fabs(vx) <= 0)
		Ax = 0, Ay = A;
	else if (fabs(vy) <= 0)
		Ax = A, Ay = 0;
	else
		Ax = A / sqrt(vx * vx + vy * vy) * fabs(vx), Ay = A / sqrt(vx * vx + vy * vy) * fabs(vy);
	//判断是否可能撞墙并设定速度
	double Margin = 0.2 + (goods == 0 ? 0.45 : 0.53);
	if (vx > 0 && vx * vx / Ax * 0.5 > 50 - x - Margin)
		forwar = 0;
	if (vx < 0 && vx * vx / Ax * 0.5 > x - Margin)
		forwar = 0;
	if (vy > 0 && vy * vy / Ay * 0.5 > 50 - y - Margin)
		forwar = 0;
	if (vy < 0 && vy * vy / Ay * 0.5 > y - Margin)
		forwar = 0;
}
pair<double, double> Car::mov(double nx, double ny)
{
	double DeltaAng = CalcAng(nx, ny);//计算当前朝向与目标点的偏向角
	double forwar = CalcForward(DeltaAng), rot;//速度与角速度的设定值
	double I = 0.5 * (goods == 0 ? 0.04100625 : 0.07890481) * Pi * 20, B = 50.0 / I;//计算转动惯量和角加速度
	bool Check = (fabs(DeltaAng) < 1.56) &&
		(tan(fabs(DeltaAng)) * sqrt((nx - x) * (nx - x) + (ny - y) * (ny - y)) <= 0.4 - eps);//判断当前朝向直行是否能到目标点
	//设定rotate
	if (Check)
		rot = 0;
	else if (DeltaAng > 0)//是否存在DeltaAng*w<0的情况，影响是什么
	{
		if (w * w / B * 0.5 < DeltaAng - eps)
			rot = Pi;//这边可以根据w与DeltaAng的大小关系适当调整rot
		else
			rot = 0;
	}
	else if (DeltaAng < 0)
	{
		if (w * w / B * 0.5 < fabs(DeltaAng) - eps)
			rot = -Pi;
		else
			rot = 0;
	}
	//撞墙判定
	if (fabs(vx) > eps || fabs(vy) > eps)
		MarginCheck(forwar);
	return pair<double, double>(forwar, rot);
}
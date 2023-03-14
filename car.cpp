#include "Global.h"
#include "car.h"

using namespace std;
const double eps = 1e-2;
const double Pi = 3.1415926536;
Car car[5];
double Car::Sign(double k) {
	if (fabs(k) < eps)return 0;
	return k > 0 ? 1 : -1;
}
double Car::Dot(double x1, double y1, double x2, double y2) {
	return x1 * x2 + y1 * y2;
}
double Car::Cross(double x1, double y1, double x2, double y2) {
	return x1 * y2 - x2 * y1;
}
double Car::Dist(double x1, double y1, double x2, double y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
double Car::GetR(int k) {
	return k == 0 ? 0.45 : 0.53;
}
double Car::CombineV(double p, double q) {
	return sqrt(p * p + q * q);
}
double Car::CalcAng(double nx, double ny){
	double res = atan2(ny - y, nx - x) - ang;
	if (res >= Pi)
		res -= 2 * Pi;
	if (res <= -Pi)
		res += 2 * Pi;
	return res;
}
double Car::CalcRotate(double nx,double ny,double DeltaAng) {
	//计算转动惯量和角加速度
	double I = 0.5 * pow(GetR(goods),4) * Pi * 20, B = 50.0 / I;
	//判断当前朝向直行是否能到目标点
	bool Check = (fabs(DeltaAng) < 1.56) && (tan(fabs(DeltaAng) * Dist(nx, ny, x, y) <= 0.4 - eps));
	//根据当前偏向角和角速度决定加速旋转或减速旋转
	double res = 0;
	if (Check)
		res = 0;
	else if (DeltaAng > 0)//是否存在DeltaAng*w<0的情况，影响是什么
	{
		if (w * w / B * 0.5 < DeltaAng - eps)
			res = Pi;//这边可以根据w与DeltaAng的大小关系适当调整rot
		else
			res = 0;
	}
	else if (DeltaAng < 0)
	{
		if (w * w / B * 0.5 < fabs(DeltaAng) - eps)
			res = -Pi;
		else
			res = 0;
	}
	return res;
}
double Car::CalcForward(double DeltaAng) {
	return cos(DeltaAng)* (fabs(DeltaAng) > Pi / 2 ? 0 : 6);//可以调整这里使用的函数
}
void Car::CarCrashCheck(double& forwar, double& rot) {
	//定义警戒范围
	double AlertRange = 5;
	//判定与每个小车的相交情况
	for (int i = 0; i < 4; i++) {
		//判定是否是同一辆车
		if (car[i].x == x && car[i].y == y)continue;
		//判定该小车是否进入警戒范围
		double dis = Dist(car[i].x, car[i].y, x, y);
		if (dis > AlertRange)continue;
		//计算两个小车速度夹角的正弦和余弦值
		double SinAng = Cross(vx, vy, car[i].vx, car[i].vy) / (CombineV(car[i].vx, car[i].vy) * CombineV(vx, vy));
		double CosAng = Dot(vx, vy, car[i].vx, car[i].vy) / (CombineV(car[i].vx, car[i].vy) * CombineV(vx, vy));
		//似平行情况判定
		if (fabs(SinAng) < sin(Pi / 60)) {
			//路径不重合判定（点到直线距离）
			if (fabs(Cross(x - car[i].x, y - car[i].y, car[i].vx, car[i].vy)) / CombineV(car[i].vx, car[i].vy) > 0.5 + GetR(goods) + GetR(car[i].goods))
				continue;
			else {
				//对碰判定
				if (CosAng < 0) {
					rot = -Pi / 3;
				}
				//追及判定
				else {
					if (Sign(Dot(car[i].x - x, car[i].y - y, vx, vy)) > 0 && Dist(x, y, car[i].x, car[i].y) <= AlertRange / 1.5)
						forwar = max(CombineV(car[i].vx, car[i].vy) - 1, 0.0);
				}
			}
		}
		//相交情况判定
		else {
			//求解交点
			double ux = x + vx - car[i].x - car[i].vx, uy = y + vy - car[i].y - car[i].vy;
			double T = Cross(ux, uy, car[i].vx, car[i].vy) / Cross(vx, vy, car[i].vx, car[i].vy);
			double tx = x + vx - T * vx, ty = y + vy - T * vy;
			//交点在地图外
			if (tx < 0 || tx>50 || ty < 0 || ty>50)
				continue;
			//交点在一定范围外
			if (Dist(x, y, tx, ty) >= AlertRange * 3 && Dist(car[i].x, car[i].y, tx, ty) >= AlertRange * 3)
				continue;
			//计算抵达交点所需时间
			double TimeCost1 = Dist(x, y, tx, ty) / CombineV(vx, vy) * Sign(Dot(tx - x, ty - y, vx, vy));
			double TimeCost2 = Dist(car[i].x, car[i].y, tx, ty) / CombineV(car[i].vx, car[i].vy) * Sign(Dot(tx - car[i].x, car[i].y - y, car[i].vx, car[i].vy));
			//交点在反方向
			if (TimeCost1 <= eps || TimeCost2 <= eps)
				continue;
			//抵达交点时间差较大
			if (fabs(TimeCost1 - TimeCost2) > 0.5 + 2 * fabs(CosAng))
				continue;
			//当前小车先到达交点
			if (TimeCost1 < TimeCost2)
				continue;
			//当前小车后到达交点
			forwar = 0;
		}
	}
}
void Car::MarginCheck(double& forwar) {
	//计算质量和加速度
	double M = pow(GetR(goods), 2) * Pi * 20, A = 250.0 / M, Ax, Ay;
	//加速度矢量分解
	if (fabs(vx) <= 0)
		Ax = 0, Ay = A;
	else if (fabs(vy) <= 0)
		Ax = A, Ay = 0;
	else
		Ax = A / CombineV(vx, vy) * fabs(vx), Ay = A / CombineV(vx, vy) * fabs(vy);
	//判断是否可能发生边界碰撞并修改速度设定值
	double Margin = 0.2 + GetR(goods);//这里可以考虑增大缓冲带长度
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
	//计算当前朝向与目标点的偏向角
	double DeltaAng = CalcAng(nx, ny);
	//计算角速度与速度的设定值
	double rot = CalcRotate(nx, ny, DeltaAng), forwar = CalcForward(DeltaAng);
	//小车碰撞判定
	if (fabs(vx) > eps || fabs(vy) > eps)
		CarCrashCheck(forwar, rot);
	//边界碰撞判定
	if (fabs(vx) > eps || fabs(vy) > eps)
		MarginCheck(forwar);
	return pair<double, double>(forwar, rot);
}

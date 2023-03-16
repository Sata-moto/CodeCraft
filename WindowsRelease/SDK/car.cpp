#include "Global.h"
#include "car.h"

using namespace std;
const double eps = 1e-2;
Car car[5];
pair<double, double> Car::getVec(double NowAng) {
	return make_pair(cos(NowAng), sin(NowAng));
}
bool Car::JudgeCross(double l1, double r1, double l2, double r2) {
	if (l1 <= l2 && r2 >= l1)return true;
	if (r1 <= r2 && r1 >= l2)return true;
	if (l1 <= l2 && r2 <= r1)return true;
	if (l2 <= l1 && r1 <= r2)return true;
	return false;
}
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
void Car::AdjuAng(double& InAng) {
	if (InAng >= Pi)
		InAng -= 2 * Pi;
	if (InAng <= -Pi)
		InAng += 2 * Pi;
}
double Car::GetR(int k) {
	return k == 0 ? 0.45 : 0.53;
}
double Car::CombineV(double p, double q) {
	return sqrt(p * p + q * q);
}
double Car::CombineV(pair<double, double>SpeedVec) {
	return sqrt(SpeedVec.first * SpeedVec.first + SpeedVec.second * SpeedVec.second);
}
double Car::CalcAng(double nx, double ny){
	double res = atan2(ny - y, nx - x) - ang;
	AdjuAng(res);
	return res;
}
double Car::CalcRotate(double nx,double ny,double DeltaAng) {
	//计算转动惯量和角加速度
	double I = 0.5 * pow(GetR(goods),4) * Pi * 20, B = 50.0 / I;
	//判断当前朝向直行是否能到目标点
	bool Check = (fabs(DeltaAng) < 1.56) && (tan(fabs(DeltaAng)) * Dist(nx, ny, x, y) <= 0.4 - eps);
	//根据当前偏向角和角速度决定加速旋转或减速旋转
	double res = 0;
	if (Check)
		res = 0;
	else if (DeltaAng > 0 && w >= 0) {//是否存在DeltaAng*w<0的情况，影响是什么
		if (w * w / B * 0.5 < DeltaAng - eps)
			res = Pi;//这边可以根据w与DeltaAng的大小关系适当调整rot
		else
			res = 0;
	}
	else if (DeltaAng > 0 && w < 0) {
		res = Pi;
	}
	else if (DeltaAng < 0 && w <= 0) {
		if (w * w / B * 0.5 < fabs(DeltaAng) - eps)
			res = -Pi;
		else
			res = 0;
	}
	else if (DeltaAng < 0 && w > 0) {
		res = -Pi;
	}
	return res;
}
double Car::CalcForward(double DeltaAng) {
	return cos(DeltaAng)* (fabs(DeltaAng) > Pi / 2 ? 0 : 6);//可以调整这里使用的函数
}
void Car::CarCrashCheck(double& forwar, double& rot) {
	//定义警戒范围
	double AlertRange = 5, AlertTime = 3;
	int numID = -1;
	double SinAng, CosAng, ux, uy, v, d, v2, d2, Setforwar = forwar;
	double v1x, v1y, v2x, v2y, AddAng, PredAng1, PredAng2, PredAng3, newd1, newd2, newd3;
	double I = 0.5 * pow(GetR(goods), 4) * Pi * 20, B = 50.0 / I;
	pair<double, double>Vecp;
	//判定与每个小车的相交情况
	for (int i = 0; i < 4; i++) {
		if (car[i].x == x && car[i].y == y) {
			numID = i;
			break;
		}
	}
	forwar = 7;
	for (int i = 0; i < 4; i++) {
		//判定是否是同一辆车
		if (car[i].x == x && car[i].y == y)continue;
		//判定该小车是否进入警戒范围
		if (Dist(car[i].x, car[i].y, x, y) > AlertRange)continue;
		//计算两小车到彼此路径直线的距离
		Vecp = getVec(ang); v = CombineV(Vecp); v1x = Vecp.first; v1y = Vecp.second;
		d = fabs(Cross(car[i].x - x, car[i].y - y, Vecp.first, Vecp.second)) / v;
		Vecp = getVec(car[i].ang); v2 = CombineV(Vecp); v2x = Vecp.first; v2y = Vecp.second;
		d2 = fabs(Cross(x - car[i].x, y - car[i].y, Vecp.first, Vecp.second)) / v2;
		//计算两个小车速度方向的正弦和余弦值
		SinAng = fabs(Cross(v2x, v2y, v1x, v1y)) / (v2 * v);
		CosAng = Dot(v1x, v1y, v2x, v2y) / (v2 * v);
		//沿着当前小车速度方向分解另一小车的速度
		ux = v2 * CosAng;//ux为正表示同向，为负表示反向
		uy = v2 * SinAng;//uy为正表示靠近，为负表示远离
		if (Sign(Cross(car[i].x - x, car[i].y - y, v1x, v1y)) * Sign(Cross(v1x, v1y, v2x, v2y)) <= 0)
			uy *= -1;
		//永远不会处在路径直线上（d/uy大于某一值或uy<0）
		if ((d >= GetR(goods) + GetR(car[i].goods) + 1 && uy < eps) ||
			(uy >= eps && max(d - GetR(goods) - GetR(car[i].goods) - 1, 0.0) / uy >= AlertTime))
			continue;
		//前方180+“一定角度”度视角内无车
		if (Dot(car[i].x - x, car[i].y - y, v1x, v1y) / (v * CombineV(car[i].x - x, car[i].y - y)) < 0)continue;
		//已经处在路径直线上但不会发生碰撞
		//已经处在路径直线上并会发生碰撞
		if (d < GetR(goods) + GetR(car[i].goods) + 1) {
			if (v * ux >= 0) {
				if (Sign(Dot(car[i].x - x, car[i].y - y, v1x, v1y)) > 0 && Dist(x, y, car[i].x, car[i].y) <= AlertRange / 1.2)//微调高级紧急范围
					forwar = min(forwar, max(ux - 1, 0.0));
			}
			else {
				//forwar应乘进速度夹角参数
				if (d > GetR(goods) + GetR(car[i].goods))forwar = 6;
				else forwar = min(forwar, 6 * cos((1 - max(Dist(x, y, car[i].x, car[i].y) - 1.5, 0.0) / (AlertRange - 1.5)) * (Pi / 2)));//需要计算
				double Ang1 = Cross(v1x, v1y, car[i].x - x, car[i].y - y) / (v * CombineV(car[i].x - x, car[i].y - y));
				/*if (Ang1 > sin(Pi / 9) || (Ang1 >= -sin(Pi / 9) && Ang1 <= 0))//调整Pi/6
					rot = max(rot, (1 - (max(Dist(x, y, car[num].x, car[num].y) - 2, 0.0) / AlertRange)) * (-Pi));
				else if (Ang1 < -sin(Pi / 9) || (Ang1 >= 0 && Ang1 <= sin(Pi / 9)))
					rot = min(rot, (1 - (max(Dist(x, y, car[num].x, car[num].y) - 2, 0.0) / AlertRange)) * Pi);
				*/
				if (goods > car[i].goods || (goods == car[i].goods && numID > i))
					continue;
				if (uy > 0.2) {//加入uy与当前距离比较参数
					if (fabs(w) > eps) {
						AddAng = w * w / B * 0.5;
						PredAng1 = ang + Sign(w) * (AddAng + Pi / 15);
						PredAng2 = ang + Sign(w) * AddAng;
						PredAng3 = ang + Sign(w) * (AddAng - Pi / 15);
						AdjuAng(PredAng1); AdjuAng(PredAng2); AdjuAng(PredAng3);
						Vecp = getVec(PredAng1);
						newd1 = fabs(Cross(car[i].x - x, car[i].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
						Vecp = getVec(PredAng2);
						newd2 = fabs(Cross(car[i].x - x, car[i].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
						Vecp = getVec(PredAng3);
						newd3 = fabs(Cross(car[i].x - x, car[i].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
						if (newd3 > GetR(goods) + GetR(car[i].goods))rot = 0;
						else if (newd1 <= GetR(goods) + GetR(car[i].goods) && newd2 > GetR(goods) + GetR(car[i].goods))rot = 0;
						else rot = Sign(w) * Pi;
					}
					else if (Ang1 >= 0)
						rot = Pi;
					else
						rot = -Pi;
				}
				else {
					if (fabs(w) > eps) {
						AddAng = w * w / B * 0.5;
						PredAng1 = ang + Sign(w) * (AddAng + Pi / 15);
						PredAng2 = ang + Sign(w) * AddAng;
						PredAng3 = ang + Sign(w) * (AddAng - Pi / 15);
						AdjuAng(PredAng1); AdjuAng(PredAng2); AdjuAng(PredAng3);
						Vecp = getVec(PredAng1);
						newd1 = fabs(Cross(car[i].x - x, car[i].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
						Vecp = getVec(PredAng2);
						newd2 = fabs(Cross(car[i].x - x, car[i].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
						Vecp = getVec(PredAng3);
						newd3 = fabs(Cross(car[i].x - x, car[i].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
						if (newd3 > GetR(goods) + GetR(car[i].goods))rot = 0;
						else if (newd1 <= GetR(goods) + GetR(car[i].goods) && newd2 > GetR(goods) + GetR(car[i].goods))rot = 0;
						else rot = Sign(w) * Pi;
					}
					if (Ang1 >= 0)
						rot = -Pi;
					else
						rot = Pi;
				}
			}
			/*
			if (fabs(uy) < eps) {
			}
			else {
				double RestTime = (GetR(goods) + GetR(car[i].goods) + 0.1) / fabs(uy);
			}
			*/
		}
		//即将处在路径直线上但不会发生碰撞
		//即将处在路径直线上并会发生碰撞
		else if (d2 < GetR(goods) + GetR(car[i].goods) + 1) {
			if (goods > car[i].goods || (goods == car[i].goods && numID > i))
				continue;
			rot = 0;
			forwar = 6;
		}
		/*
		else if (uy >= eps && (d - GetR(goods) - GetR(car[i].goods) - 1) / uy < AlertTime) {
			double vcx = x + v1x - car[i].x - car[i].v1x, vcy = y + vy - car[i].y - car[i].vy;
			double T = Cross(vcx, vcy, car[i].v1x, car[i].vy) / Cross(vcx, vcy, car[i].v1x, car[i].vy);
			double tx = x + v1x - T * v1x, ty = y + vy - T * vy;
			double RestLen1 = Dist(car[i].x, car[i].y, tx, ty);
			double RestLen2 = Dist(tx, ty, x, y);
			double RestTime1 = (RestLen1 - GetR(goods) - GetR(car[i].goods)) / v2;
			double RestTime2 = (RestLen1 + GetR(goods) + GetR(car[i].goods)) / v2;
			double CostTime1 = (RestLen2 - GetR(goods) - GetR(car[i].goods)) / v;//这边可能还要修改
			double CostTime2 = (RestLen2 + GetR(goods) + GetR(car[i].goods)) / v;
			if (!JudgeCross(RestTime1, RestTime2, CostTime1, CostTime2))
				continue;
			else {
				if (goods > car[i].goods || (goods == car[i].goods && numID > i))
					forwar = min(forwar, 6.0);
				else forwar = min(forwar, 0.0);//此处是否需要加入旋转
			}
		}*/
	}
	if (forwar == 7)
		forwar = Setforwar;
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
	double Margin = 0.4 + GetR(goods);//这里可以考虑增大缓冲带长度
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

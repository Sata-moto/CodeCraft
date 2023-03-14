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
	return cos(DeltaAng) * (fabs(DeltaAng) > Pi / 2 ? 0 : 6);//���Ե�������ʹ�õĺ���
}
void Car::MarginCheck(double& forwar) {
	//���������ͼ��ٶ�
	double M = (goods == 0 ? 0.2025 : 0.2809) * Pi * 20, A = 250.0 / M, Ax, Ay;
	//���ٶ�ʸ���ֽ�
	if (fabs(vx) <= 0)
		Ax = 0, Ay = A;
	else if (fabs(vy) <= 0)
		Ax = A, Ay = 0;
	else
		Ax = A / sqrt(vx * vx + vy * vy) * fabs(vx), Ay = A / sqrt(vx * vx + vy * vy) * fabs(vy);
	//�ж��Ƿ����ײǽ���趨�ٶ�
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
	double DeltaAng = CalcAng(nx, ny);//���㵱ǰ������Ŀ����ƫ���
	double forwar = CalcForward(DeltaAng), rot;//�ٶ�����ٶȵ��趨ֵ
	double I = 0.5 * (goods == 0 ? 0.04100625 : 0.07890481) * Pi * 20, B = 50.0 / I;//����ת�������ͽǼ��ٶ�
	bool Check = (fabs(DeltaAng) < 1.56) &&
		(tan(fabs(DeltaAng)) * sqrt((nx - x) * (nx - x) + (ny - y) * (ny - y)) <= 0.4 - eps);//�жϵ�ǰ����ֱ���Ƿ��ܵ�Ŀ���
	//�趨rotate
	if (Check)
		rot = 0;
	else if (DeltaAng > 0)//�Ƿ����DeltaAng*w<0�������Ӱ����ʲô
	{
		if (w * w / B * 0.5 < DeltaAng - eps)
			rot = Pi;//��߿��Ը���w��DeltaAng�Ĵ�С��ϵ�ʵ�����rot
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
	//ײǽ�ж�
	if (fabs(vx) > eps || fabs(vy) > eps)
		MarginCheck(forwar);
	return pair<double, double>(forwar, rot);
}
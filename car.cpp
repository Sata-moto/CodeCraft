#include "Global.h"
#include "car.h"

using namespace std;
const double eps = 1e-2;
const double Pi = 3.1415926536;
pair<double, double> Car::mov(double nx, double ny)
{
	double forwar = 0, rot;
	double I = 0.5 * (goods == 0 ? 0.04100625 : 0.07890481) * Pi * 20, B = 50.0 / I;
	double DeltaAng = atan2(ny - y, nx - x) - ang;
	if (DeltaAng >= Pi)
		DeltaAng -= 2 * Pi;
	if (DeltaAng <= -Pi)
		DeltaAng += 2 * Pi;
	bool Check = (fabs(DeltaAng) >= 1.56) ||
		(tan(fabs(DeltaAng)) * sqrt((nx - x) * (nx - x) + (ny - y) * (ny - y)) > 0.4 - eps);
	if ((fabs(vx) > eps || fabs(vy) > eps) && Check)
		return pair<double, double>(0, 0);
	if (!Check)
	{
		if (fabs(w) <= eps)
			rot = 0;
		else if (w > 0)
			rot = -Pi;
		else if (w < 0)
			rot = Pi;
	}
	else if (DeltaAng > 0)
	{
		if (w * w / B * 0.5 < DeltaAng - eps)
			rot = Pi;
		else
			rot = -Pi;
	}
	else if (DeltaAng < 0)
	{
		if (w * w / B * 0.5 < fabs(DeltaAng) - eps)
			rot = -Pi;
		else
			rot = Pi;
	}
	if (fabs(w) <= 0.8 && !Check)
	{
		if (fabs(vx) <= eps && fabs(vy) <= eps)
			forwar = 6;
		else
		{
			double M = (goods == 0 ? 0.2025 : 0.2809) * Pi * 20, A = 250.0 / M, Ax, Ay;
			double Margin = 0.2 + (goods == 0 ? 0.45 : 0.53);
			if (fabs(vx) <= 0)
				Ax = 0, Ay = A;
			else if (fabs(vy) <= 0)
				Ax = A, Ay = 0;
			else
				Ax = A / sqrt(vx * vx + vy * vy) * fabs(vx), Ay = A / sqrt(vx * vx + vy * vy) * fabs(vy);
			forwar = 6;
			if (vx > 0 && vx * vx / Ax * 0.5 > 50 - x - Margin)
				forwar = 0;
			if (vx < 0 && vx * vx / Ax * 0.5 > x - Margin)
				forwar = 0;
			if (vy > 0 && vy * vy / Ay * 0.5 > 50 - y - Margin)
				forwar = 0;
			if (vy < 0 && vy * vy / Ay * 0.5 > y - Margin)
				forwar = 0;
		}
	}
	return pair<double, double>(forwar, rot);
}
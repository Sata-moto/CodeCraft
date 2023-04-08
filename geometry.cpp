#include "Global.h"
#include "namespace.h"
#include "geometry.h"


using namespace std;
static const double eps = 1e-2;


//几何/数学
pair<double, double> getVec(double NowAng) {
	return make_pair(cos(NowAng), sin(NowAng));
}
double Sign(double k) {
	if (fabs(k) < eps)return 0;
	return k > 0 ? 1 : -1;
}
double Dot(double x1, double y1, double x2, double y2) {
	return x1 * x2 + y1 * y2;
}
double Dot(pair<double, double>s, pair<double, double>t) {
	return s.first * t.first + s.second * t.second;
}
double Cross(double x1, double y1, double x2, double y2) {
	return x1 * y2 - x2 * y1;
}
double Cross(pair<double, double>s, pair<double, double>t) {
	return s.first * t.second - s.second * t.first;
}
pair<double, double> Rotate(pair<double, double> Tvec, double ang) {
	double x = Tvec.first, y = Tvec.second;
	return make_pair(x * cos(ang) - y * sin(ang), y * cos(ang) + x * sin(ang));
}
pair<double, double> multi(pair<double, double> Tvec, double k) {
	return make_pair(Tvec.first * k, Tvec.second * k);
}
pair<double, double> Add(pair<double, double>vec1, pair<double, double>vec2) {
	return make_pair(vec1.first + vec2.first, vec1.second + vec2.second);
}
pair<double, double> Sub(pair<double, double>vec1, pair<double, double>vec2) {
	return make_pair(vec1.first - vec2.first, vec1.second - vec2.second);
}
double Dist(double x1, double y1, double x2, double y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
double Dist(pair<double, double>P1, pair<double, double>P2) {
	return sqrt((P1.first - P2.first) * (P1.first - P2.first) + (P1.second - P2.second) * (P1.second - P2.second));
}
double PointToLine(pair<double, double> P, pair<double, double> S, pair<double, double> D) {
	double x = P.first, y = P.second, sx = S.first, sy = S.second, dx = D.first, dy = D.second;
	return fabs(Cross(dx, dy, x - sx, y - sy)) / CombineV(D);
}
double PointToSegment(pair<double, double> P, pair<double, double> S, pair<double, double> T) {
	double x = P.first, y = P.second, sx = S.first, sy = S.second, tx = T.first, ty = T.second;
	bool PosCheck = (Dot(x - sx, y - sy, tx - sx, ty - sy) > 0) && (Dot(x - tx, y - ty, sx - tx, sy - ty) > 0);
	if (PosCheck)return PointToLine(P, S, Add(T, multi(S, -1)));
	else return min(Dist(P, S), Dist(P, T));
}
bool SegmentCross(pair<double, double>s1, pair<double, double>t1, pair<double, double>s2, pair<double, double>t2) {
	bool Xcheck = (max(s1.first, t1.first) < min(s2.first, t2.first)) || (min(s1.first, t1.first) > max(s2.first, t2.first));
	bool Ycheck = (max(s1.second, t1.second) < min(s2.second, t2.second)) || (min(s1.second, t1.second) > max(s2.second, t2.second));
	bool RejectCheck = Xcheck || Ycheck;
	bool l1check = Cross(s2.first - s1.first, s2.second - s1.second, t1.first - s1.first, t1.second - s1.second) *
		Cross(t2.first - s1.first, t2.second - s1.second, t1.first - s1.first, t1.second - s1.second) <= 0;
	bool l2check = Cross(s1.first - s2.first, s1.second - s2.second, t2.first - s2.first, t2.second - s2.second) *
		Cross(t1.first - s2.first, t1.second - s2.second, t2.first - s2.first, t2.second - s2.second) <= 0;
	bool CrossCheck = l1check && l2check;
	return (!RejectCheck) && CrossCheck;
}
pair<double, double> CrossPoint(pair<double, double>s1, pair<double, double>t1, pair<double, double>s2, pair<double, double>t2) {
	pair<double, double>a = Sub(t1, s1), b = Sub(t2, s2), u = Sub(t1, t2);
	double T = Cross(u, b) / Cross(a, b);
	return Sub(t1, multi(a, T));
}
void AdjuAng(double& InAng) {
	if (InAng >= Pi)
		InAng -= 2 * Pi;
	if (InAng <= -Pi)
		InAng += 2 * Pi;
}
double CombineV(double p, double q) {
	return sqrt(p * p + q * q);
}
double CombineV(pair<double, double>SpeedVec) {
	return sqrt(SpeedVec.first * SpeedVec.first + SpeedVec.second * SpeedVec.second);
}
void UnitV(pair<double, double>& Vect) {
	double length = CombineV(Vect);
	Vect.first /= length;
	Vect.second /= length;
}
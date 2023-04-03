#include "Global.h"
#include "car.h"
#include "desk.h"
#include "namespace.h"

using namespace std;
const double eps = 1e-2;
const double epss = 0;
const double deltaeps = 0.02;//延长二分判定长度
Car car[5];
static int dx[4] = { -1,0,1,0 }, dy[4] = { 0,1,0,-1 };
static int dxx[8] = { -1,0,1,1,1,0,-1,-1 }, dyy[8] = { -1,-1,-1,0,1,1,1,0 };
static double deltaang1 = atan(1.0 / 3.0), deltaang2 = Pi / 2 - 2 * deltaang1;
static double angset[8] = { -Pi + deltaang1,-Pi + deltaang1 + deltaang2,-deltaang1 - deltaang2,-deltaang1,deltaang1,deltaang1 + deltaang2,Pi - deltaang1 - deltaang2,Pi - deltaang1 };
static int vis[N][N], t;
static bool obfind;
pair<double, double>des[4];



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
	if (PosCheck)return PointToLine(P, S, Add(S, multi(T, -1)));
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
void DFS(pair<int, int>meshc, pair<double, double>realc, double r) {
	vis[meshc.first][meshc.second] = t;
	if (map[meshc.first][meshc.second] == '#') {
		obfind = true;
		return;
	}
	for (int i = 0; i < 4; i++) {
		int nx = meshc.first + dx[i], ny = meshc.second + dy[i];
		if (nx < 0 || nx>101 || ny < 0 || ny>101)continue;
		if (vis[nx][ny] == t)continue;
		pair<double, double>realcent = math_n::etoz(nx, ny);
		if (Dist(make_pair(realcent.first - 0.25, realcent.second - 0.25), realc) < r ||
			Dist(make_pair(realcent.first - 0.25, realcent.second + 0.25), realc) < r ||
			Dist(make_pair(realcent.first + 0.25, realcent.second + 0.25), realc) < r ||
			Dist(make_pair(realcent.first + 0.25, realcent.second - 0.25), realc) < r)
			DFS(make_pair(nx, ny), realc, r);
		if (obfind)return;
	}
}
bool Search(double x, double y, double r) {
	t++; obfind = false;
	DFS(math_n::ztoe(x, y), make_pair(x, y), r);
	return obfind;
}


//小车
int Car::Carry(int x) {
	return x == 0 ? 0 : 1;
}
double Car::GetR(int k) {
	return k == 0 ? 0.45 : 0.53;
}
double Car::CalcAng(double nx, double ny) {
	double res = atan2(ny - y, nx - x) - ang;
	AdjuAng(res);
	return res;
}
double Car::CalcRotate(double nx, double ny, double DeltaAng) {
	int numID = -1;
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			numID = i;
			break;
		}
	}
	//计算转动惯量、角加速度和距离目标点的距离
	double I = 0.5 * pow(GetR(goods), 4) * Pi * 20, B = 50.0 / I, diss = Dist(nx, ny, x, y);
	//判断当前朝向直行是否能到目标点
	bool Check = (fabs(DeltaAng) < 1.56) && (tan(fabs(DeltaAng)) * diss <= 0.05) &&
		ObCheck(x, y, x + diss * cos(ang), y + diss * sin(ang), destination[numID], GetR(goods), 0);
	//这里需要修※※※（影响过隧道抖动&过墙角判断）
	//根据当前偏向角和角速度决定加速旋转或减速旋转
	double res = 0;
	if (Check)
		res = 0;
	else if (DeltaAng > 0 && w >= 0) {
		if (w * w / B * 0.5 < DeltaAng - eps) res = Pi;//这边可以根据w与DeltaAng的大小关系适当调整rot
		else res = 0;
	}
	else if (DeltaAng > 0 && w < 0) res = Pi;
	else if (DeltaAng < 0 && w <= 0) {
		if (w * w / B * 0.5 < fabs(DeltaAng) - eps) res = -Pi;
		else res = 0;
	}
	else if (DeltaAng < 0 && w > 0) res = -Pi;
	return res;
}
double Car::CalcForward(double nx, double ny, double DeltaAng) {
	double res = cos(DeltaAng) * (fabs(DeltaAng) > Pi / 2 ? 0 : 6);
	double Cv = CombineV(vx, vy), M = pow(GetR(goods), 2) * Pi * 20, A = 250.0 / M;
	pair<int, int>s = math_n::ztoe(x, y), t = math_n::ztoe(nx, ny);
	pair<double, double>reals = math_n::etoz(s.first, s.second), realt = math_n::etoz(t.first, t.second);
	if (fabs(DeltaAng) < Pi / 18 && Cv * Cv * 0.5 / A > Dist(reals.first, reals.second, realt.first, realt.second) - 0.3)res = 0;
	//到点减速需要修※※※※※（影响准确进入窄道&短距离前进）
	return res;
}
bool Car::ObCheck(double x1, double y1, double x2, double y2, int desk_num, double width, bool Checkbool) {
	//output << "Startnode=" << x1 << "," << y1 << endl;//
	//output << "Endnode=" << x2 << "," << y2 << endl;//
	queue<pair<double, double> >q;
	pair<double, double> p[6];
	pair<int, int>S, T;
	pair<double, double>realS, realT;
	pair<double, double>Dvec = make_pair(x2 - x1, y2 - y1);
	UnitV(Dvec);
	pair<double, double>Vertvec1 = Rotate(Dvec, Pi / 2);//逆时针90度
	pair<double, double>Vertvec2 = Rotate(Dvec, -Pi / 2);//顺时针90度
	//output << "Vertvec1 " << Vertvec1.first << " " << Vertvec1.second << endl;
	//output << "Vertvec2 " << Vertvec2.first << " " << Vertvec2.second << endl;

	//判断终点一定范围内没有障碍物※※※（是否需要配合到终点减速？可能不需要）Checkbool用于判断是否需要判断终点附近的障碍物情况
	if (Checkbool && Search(x2, y2, GetR(goods)))return false;

	q.push(make_pair(x2, y2));
	q.push(make_pair(x1, y1));
	q.push(make_pair(x2 + Vertvec1.first * width, y2 + Vertvec1.second * width));
	q.push(make_pair(x1 + Vertvec1.first * width, y1 + Vertvec1.second * width));
	q.push(make_pair(x2 + Vertvec2.first * width, y2 + Vertvec2.second * width));
	q.push(make_pair(x1 + Vertvec2.first * width, y1 + Vertvec2.second * width));
	q.push(make_pair(x2 + Vertvec1.first * width * 0.5, y2 + Vertvec1.second * width * 0.5));
	q.push(make_pair(x1 + Vertvec1.first * width * 0.5, y1 + Vertvec1.second * width * 0.5));
	q.push(make_pair(x2 + Vertvec2.first * width * 0.5, y2 + Vertvec2.second * width * 0.5));
	q.push(make_pair(x1 + Vertvec2.first * width * 0.5, y1 + Vertvec2.second * width * 0.5));
	while (!q.empty()) {
		realS = q.front(); q.pop();
		realT = q.front(); q.pop();
		if (realS.first < 0 || realS.first>50 || realS.second < 0 || realS.second>50)return false;
		if (realT.first < 0 || realT.first>50 || realT.second < 0 || realT.second>50)return false;
		//output << "realS=" << realS.first << "," << realS.second << endl;//
		//output << "realT=" << realT.first << "," << realT.second << endl;//
		S = math_n::ztoe(realS.first, realS.second);
		T = math_n::ztoe(realT.first, realT.second);
		int Crossnum = 1;
		//int t = 0;//
		while (S != T) {
			//if (++t > 100)output << "no!" << endl;//
			//if (t > 100)exit(0);//
			//output << "S=" << S.first << "," << S.second << endl;//
			//output << "T=" << T.first << "," << T.second << endl;//
			if (dis[Carry(goods)][desk_num][S.first][S.second] == 1e9)
				return false;
			p[0] = math_n::etoz(S.first, S.second);
			p[1] = make_pair(p[0].first - 0.25, p[0].second - 0.25);
			p[2] = make_pair(p[0].first - 0.25, p[0].second + 0.25);
			p[3] = make_pair(p[0].first + 0.25, p[0].second + 0.25);
			p[4] = make_pair(p[0].first + 0.25, p[0].second - 0.25);
			p[5] = p[1];
			for (int i = 1; i <= 4; i++) {
				if (Dot(dx[i - 1], dy[i - 1], realT.first - realS.first, realT.second - realS.second) <= 0)
					continue;
				//output << "i=" << i << endl;//
				//output << "p[i]=" << p[i].first << "," << p[i].second << endl;//
				//output << "p[i+1]=" << p[i + 1].first << "," << p[i + 1].second << endl;//
				if (SegmentCross(realS, realT, p[i], p[i + 1])) {
					//output << i << "pass" << endl;//
					Crossnum = i;
					break;
				}
			}
			int nx = S.first + dx[Crossnum - 1], ny = S.second + dy[Crossnum - 1];
			S = make_pair(nx, ny);
			//output << endl;//
		}
		if (dis[Carry(goods)][desk_num][S.first][S.second] == 1e9)
			return false;
	}
	return true;
}


//旧版
void Car::CarCrashCheck(double& forwar, double& rot) {
	//定义警戒范围与其他后续变量
	double AlertRange = 4, AlertTime = 1.2, CheckTime = 0.4, minDis = AlertRange;
	int numID = -1, numi = -1, cat = 0;
	double SinAng, CosAng, vecux, vecuy, ux, uy, vecv, v, d, vecv2, v2;
	double v1x, v1y, v2x, v2y, AddAng, PredAng1, PredAng2, PredAng3, newd1, newd2, newd3;
	double I = 0.5 * pow(GetR(goods), 4) * Pi * 20, B = 50.0 / I;
	pair<double, double>Vecp;
	//找寻当前小车
	for (int i = 0; i < 4; i++)
		if (car[i].x == x && car[i].y == y) {
			numID = i; break;
		}
	for (int i = 0; i < 4; i++) {
		//判定是否是同一辆车
		if (i == numID)continue;
		//判定该小车是否进入警戒范围
		if (Dist(car[i].x, car[i].y, x, y) > AlertRange)continue;
		//判断两个小车之间是否有障碍物
		if (!ObCheck(x, y, car[i].x, car[i].y, destination[numID], 0, 0))continue;
		//计算两小车速度方向单位向量、速度向量以及距离
		Vecp = getVec(ang);
		vecv = CombineV(Vecp); v = CombineV(vx, vy);
		v1x = Vecp.first; v1y = Vecp.second;
		d = fabs(Cross(car[i].x - x, car[i].y - y, v1x, v1y)) / vecv;

		Vecp = getVec(car[i].ang);
		vecv2 = CombineV(Vecp); v2 = CombineV(car[i].vx, car[i].vy);
		v2x = Vecp.first; v2y = Vecp.second;
		//计算两个小车速度方向的正弦和余弦值
		SinAng = fabs(Cross(v2x, v2y, v1x, v1y)) / (vecv2 * vecv);
		CosAng = Dot(v1x, v1y, v2x, v2y) / (vecv2 * vecv);
		//沿着当前小车速度方向分解另一小车的速度，ux为速度分量，vecux为速度方向分量
		//ux为正表示同向，为负表示反向；uy为正表示靠近，为负表示远离
		ux = v2 * CosAng;
		uy = v2 * SinAng;

		vecux = vecv2 * CosAng;
		vecuy = vecv2 * SinAng;
		//修正uy与vecuy
		if (Sign(Cross(car[i].x - x, car[i].y - y, v1x, v1y)) * Sign(Cross(v1x, v1y, v2x, v2y)) <= 0)
			uy *= -1, vecuy *= -1;
		//前方180度视角内无车
		if ((d >= GetR(goods) + GetR(car[i].goods) + 0.07 && uy < eps) ||
			(uy >= eps && max(d - GetR(goods) - GetR(car[i].goods) - 0.07, 0.0) / uy >= AlertTime))
			continue;
		//另一小车与当前小车同向行驶
		if (Dot(car[i].x - x, car[i].y - y, v1x, v1y) / (vecv * CombineV(car[i].x - x, car[i].y - y)) < 0)continue;
		if (ux < 0) {
			if (cat <= 0)cat = 1, numi = i, minDis = Dist(car[i].x, car[i].y, x, y);
			else if (Dist(car[i].x, car[i].y, x, y) < minDis) {
				minDis = Dist(car[i].x, car[i].y, x, y);
				numi = i;
			}
		}
		else if (cat <= 0) {
			if (cat == 0)cat = -1, numi = i, minDis = Dist(car[i].x, car[i].y, x, y);
			else if (Dist(car[i].x, car[i].y, x, y) < minDis) {
				minDis = Dist(car[i].x, car[i].y, x, y);
				numi = i;
			}
		}
		continue;
	}
	if (cat == 0)return;
	Vecp = getVec(ang);
	vecv = CombineV(Vecp); v = CombineV(vx, vy);
	v1x = Vecp.first; v1y = Vecp.second;
	d = fabs(Cross(car[numi].x - x, car[numi].y - y, v1x, v1y)) / vecv;

	Vecp = getVec(car[numi].ang);
	vecv2 = CombineV(Vecp); v2 = CombineV(car[numi].vx, car[numi].vy);
	v2x = Vecp.first; v2y = Vecp.second;
	//计算两个小车速度方向的正弦和余弦值
	SinAng = fabs(Cross(v2x, v2y, v1x, v1y)) / (vecv2 * vecv);
	CosAng = Dot(v1x, v1y, v2x, v2y) / (vecv2 * vecv);
	//沿着当前小车速度方向分解另一小车的速度，ux为速度分量，vecux为速度方向分量
	//ux为正表示同向，为负表示反向；uy为正表示靠近，为负表示远离
	ux = v2 * CosAng;
	uy = v2 * SinAng;

	vecux = vecv2 * CosAng;
	vecuy = vecv2 * SinAng;
	//修正uy与vecuy
	if (Sign(Cross(car[numi].x - x, car[numi].y - y, v1x, v1y)) * Sign(Cross(v1x, v1y, v2x, v2y)) <= 0)
		uy *= -1, vecuy *= -1;
	//另一小车与当前小车同向行驶
	if (vecux >= 0.15) {
		if (Dist(x, y, car[numi].x, car[numi].y) > AlertRange / 2)return;
		//补丁：两个小车均在对方前方180度范围内
		if (Sign(Dot(car[numi].x - x, car[numi].y - y, v1x, v1y)) >= 0 && Sign(Dot(x - car[numi].x, y - car[numi].y, v2x, v2y)) >= 0) {
			double vcx, vcy, T, tx, ty, RestLen1, RestLen2;
			vcx = x + v1x - car[numi].x - v2x;
			vcy = y + v1y - car[numi].y - v2y;
			if (fabs(Cross(v1x, v1y, v2x, v2y)) < eps) {
				if (Dot(v1x, v1y, car[numi].x - x, car[numi].y - y) > 0)
					tx = car[numi].x, ty = car[numi].y;
				else tx = x, ty = y;
			}
			else {
				T = Cross(vcx, vcy, v2x, v2y) / Cross(v1x, v1y, v2x, v2y);
				tx = x + v1x - T * v1x;
				ty = y + v1y - T * v1y;
			}
			RestLen1 = Dist(car[numi].x, car[numi].y, tx, ty);
			RestLen2 = Dist(tx, ty, x, y);
			if (fabs(RestLen1 - RestLen2) < 1) {
				if (goods > car[numi].goods || (goods == car[numi].goods && numID > numi))forwar = 6;
				else forwar = max(ux - 3, 0.0);
			}
			else if (RestLen1 > RestLen2)forwar = 6;
			else forwar = max(ux - 3, 0.0);
		}
		//普通的追及问题
		else if (Sign(Dot(car[numi].x - x, car[numi].y - y, v1x, v1y)) > 0 && Sign(Dot(x - car[numi].x, y - car[numi].y, v2x, v2y)) < 0) {
			forwar = max(ux - 3, 0.0);//追及问题可以调参（速度较小影响运送时间，速度较大影响运送碰撞）
		}
	}
	//另一小车与当前小车速度反向（速度分量）
	else {
		if (d > GetR(goods) + GetR(car[numi].goods) /* && ((uy > eps && (d - GetR(goods) - GetR(car[numi].goods)) / uy >= CheckTime) || uy <= eps)*/)forwar = 6;//为什么删掉后半截就不会正向回避损失速度
		else forwar = 6 * cos((1 - max(Dist(x, y, car[numi].x, car[numi].y) - GetR(goods) - GetR(car[numi].goods), 0.0) / (AlertRange - GetR(goods) - GetR(car[numi].goods))) * (Pi / 2));//forwar是否需要乘进速度夹角参数
		double Ang1 = Cross(v1x, v1y, car[numi].x - x, car[numi].y - y) / (vecv * CombineV(car[numi].x - x, car[numi].y - y));
		//重要者优先
		if (goods > car[numi].goods || (goods == car[numi].goods && numID > numi))
			return;
		//旋转方向调整
		if (uy > 0.2) {//加入uy与当前距离比较参数
			if (fabs(w) > eps) {
				AddAng = w * w / B * 0.5;
				PredAng1 = ang + Sign(w) * (AddAng + Pi / 15);
				PredAng2 = ang + Sign(w) * AddAng;
				PredAng3 = ang + Sign(w) * (AddAng - Pi / 15);
				AdjuAng(PredAng1); AdjuAng(PredAng2); AdjuAng(PredAng3);
				Vecp = getVec(PredAng1);
				newd1 = fabs(Cross(car[numi].x - x, car[numi].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
				Vecp = getVec(PredAng2);
				newd2 = fabs(Cross(car[numi].x - x, car[numi].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
				Vecp = getVec(PredAng3);
				newd3 = fabs(Cross(car[numi].x - x, car[numi].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
				if (newd3 > GetR(goods) + GetR(car[numi].goods))rot = 0;
				else if (newd1 <= GetR(goods) + GetR(car[numi].goods) && newd2 > GetR(goods) + GetR(car[numi].goods))rot = 0;
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
				newd1 = fabs(Cross(car[numi].x - x, car[numi].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
				Vecp = getVec(PredAng2);
				newd2 = fabs(Cross(car[numi].x - x, car[numi].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
				Vecp = getVec(PredAng3);
				newd3 = fabs(Cross(car[numi].x - x, car[numi].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
				if (newd3 > GetR(goods) + GetR(car[numi].goods))rot = 0;
				else if (newd1 <= GetR(goods) + GetR(car[numi].goods) && newd2 > GetR(goods) + GetR(car[numi].goods))rot = 0;
				else rot = Sign(w) * Pi;
			}
			else if (Ang1 >= 0)
				rot = -Pi;
			else
				rot = Pi;
		}
	}
}
void Car::MarginCheck(double& forwar) {
	int numID = -1;
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			numID = i;
		}
	}
	//计算质量和加速度
	double M = pow(GetR(goods), 2) * Pi * 20, A = 250.0 / M;
	double vv = CombineV(vx, vy);
	//判断是否可能发生边界碰撞并修改速度设定值
	double l = 0, r = 50, mid, res = 0;
	while (r - l >= 0.25) {
		mid = (l + r) / 2;
		double tx = x + (mid + deltaeps) * cos(ang), ty = y + (mid + deltaeps) * sin(ang);
		if (tx < 0 || tx>50 || ty < 0 || ty>50)r = mid;
		else {
			if (ObCheck(x, y, x + (mid + deltaeps) * cos(ang), y + (mid + deltaeps) * sin(ang), destination[numID], GetR(goods) + epss, 1))
				res = mid, l = mid;//注意这里和别处ObCheck的相性
			else r = mid;
		}
	}

	l = 0; r = 6;
	double resv = 0;
	while (r - l >= 0.01) {
		mid = (l + r) / 2;
		if (mid * mid / A * 0.5 > res)r = mid;
		else {
			if (vv / A + (res - vv * vv / A * 0.5) / vv >= 0)resv = mid, l = mid;
			else r = mid;
		}
	}
	forwar = min(forwar, resv);
}
pair<double, double> Car::mov(double nx, double ny) {

	int numID = -1;
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			numID = i;
			break;
		}
	}

	//计算当前朝向与目标点的偏向角
	double DeltaAng = CalcAng(nx, ny);
	//计算角速度与速度的设定值
	double rot = CalcRotate(nx, ny, DeltaAng), forwar = CalcForward(nx, ny, DeltaAng);
	//小车碰撞判定

	output << "original set---------------" << endl;
	output << "ID=" << numID << endl;
	output << "forward=" << forwar << endl;
	output << "rotate=" << rot << endl;
	output << endl;

	CarCrashCheck(forwar, rot);
	//边界碰撞判定

	//粗糙补丁（前往工作台的小车不需要减速）

	output << "CarCrashCheck---------------" << endl;
	output << "ID=" << numID << endl;
	output << "forward=" << forwar << endl;
	output << "rotate=" << rot << endl;
	output << endl;

	double checkforwar = forwar;

	if (desk[destination[numID]].x != nx || desk[destination[numID]].y != ny)
		MarginCheck(forwar);

	if (fabs(checkforwar) > eps && fabs(forwar) < eps && fabs(w) < eps && fabs(rot) < eps)
		forwar = checkforwar;

	output << "Margin---------------" << endl;
	output << "ID=" << numID << endl;
	output << "forward=" << forwar << endl;
	output << "rotate=" << rot << endl;
	output << endl;

	return pair<double, double>(forwar, rot);
}


//新版
bool Car::ChooseAvoider(int Cnum) {
	int numID = -1;
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			numID = i;
			break;
		}
	}
	if (goods > car[Cnum].goods || (goods == car[Cnum].goods && numID > Cnum))
		return true;//需要加入对方向上是否有可避让空间的判断※※※
	return false;
}
bool Car::judge(int desk_num, double Ang, double d) {
	double tx = x + (d + deltaeps) * cos(Ang), ty = y + (d + deltaeps) * sin(Ang);
	if (tx < 0 || tx>50 || ty < 0 || ty>50) return false;//判断是否越界
	pair<int, int>s = math_n::ztoe(x, y), t = math_n::ztoe(tx, ty);
	pair<double, double>sreal = math_n::etoz(s.first, s.second), treal = math_n::etoz(t.first, t.second);
	if ((dis[Carry(goods)][desk_num][s.first][s.second] - dis[Carry(goods)][desk_num][t.first][t.second]) / Dist(sreal.first, sreal.second, treal.first, treal.second) > 1.5)return false;
	//连续性的判断方法？（注意参数1.2与二分上界关联）※※※
	return ObCheck(x, y, tx, ty, desk_num, GetR(goods) + epss, 1);
}
pair<double, double> Car::Static_Avoidance(int desk_num, int mode) {

	double startang = Pi * 10, endang = Pi * 10;
	int obid = -1, lim = 0;
	pair<int, int>now = math_n::ztoe(x, y);
	for (int i = 0; i < 8; i++) {
		if (dis[Carry(goods)][desk_num][now.first][now.second] == 1e9) {
			obid = i; break;
		}
	}
	if (obid == -1) {
		for (int i = 0; i < 8; i++) {
			if (dis[Carry(goods)][desk_num][now.first + dxx[i]][now.second + dyy[i]] >
				dis[Carry(goods)][desk_num][now.first][now.second]) {
				obid = i; break;
			}
		}
	}
	lim = obid;
	obid = (obid + 1) % 8;
	while (obid != lim) {
		if (dis[Carry(goods)][desk_num][now.first + dxx[obid]][now.second + dyy[obid]] == 1e9) {
			if (startang != Pi * 10) {
				endang = angset[(obid + 1) % 8];
				if (endang < startang)endang += 2 * Pi;
				break;
			}
		}
		else {
			if (dis[Carry(goods)][desk_num][now.first + dxx[obid]][now.second + dyy[obid]] <
				dis[Carry(goods)][desk_num][now.first][now.second]) {
				if (startang == Pi * 10)
					startang = angset[obid];
			}
			else {
				if (startang != Pi * 10) {
					endang = angset[(obid + 1) % 8];
					if (endang < startang)endang += 2 * Pi;
					break;
				}
			}
		}
		obid = (obid + 1) % 8;
	}
	if (startang == Pi * 10) {
		startang = -Pi;
		endang = Pi;
	}
	else {
		if (endang == Pi * 10) {
			endang = angset[(lim + 1) % 8];
			if (endang < startang)endang += 2 * Pi;
		}
		if (startang == angset[0] || startang == angset[2] || startang == angset[4] || startang == angset[6])
			startang -= deltaang2;
		else startang -= 2 * deltaang1;
	}


	int numID = -1;
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			numID = i;
			break;
		}
	}

	double Delt = (endang-startang) / 24, l, r, mid, res, ansang = -1, ansdis = -1, maxdisperreal = -1e9, disperreal;
	
	double realtx, realty;
	while (startang < endang) {
		res = -1; l = 0; r = 5;
		while (r - l >= 0.25) {
			mid = (l + r) / 2;
			if (judge(desk_num, startang, mid))res = mid, l = mid;
			else r = mid;
		}
		if (res == -1) {
			startang += Delt;
			continue;
		}
		realtx = x + res * cos(startang);
		realty = y + res * sin(startang);
		pair<int, int>ss = math_n::ztoe(x, y), tt = math_n::ztoe(realtx, realty);
		pair<double, double>ssreal = math_n::etoz(ss.first, ss.second), ttreal = math_n::etoz(tt.first, tt.second);
		disperreal = (dis[Carry(goods)][desk_num][ss.first][ss.second] - dis[Carry(goods)][desk_num][tt.first][tt.second]) / Dist(ssreal.first, ssreal.second, ttreal.first, ttreal.second);
		if (disperreal > maxdisperreal || (fabs(disperreal - maxdisperreal) < eps && res > ansdis)) {//可以考虑差在一定范围内就选长的
			ansang = startang;
			ansdis = res;
			maxdisperreal = disperreal;
		}
		startang += Delt;
	}


	
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			output << i << "-------------------" << endl;
			pair<int, int>k;
			output << "Static" << endl << endl;
			pair<int, int>deskxy = math_n::ztoe(desk[desk_num].x, desk[desk_num].y);

			//output<<"Target desk_xy="<<
			output << "nowpos" << endl;
			k = math_n::ztoe(x, y);
			output << k.first << " " << k.second << endl;
			output << map[k.first][k.second] << endl;
			k = math_n::ztoe(x + ansdis * cos(ansang), y + ansdis * sin(ansang));
			output << "target" << endl;
			output << k.first << " " << k.second << endl;
			output << map[k.first][k.second] << endl;
			output << "ansdis=" << ansdis << endl;
			output << "ansang=" << ansang << endl;
			output << endl;
		}
	}
	
	
	

	if (mode == 0)	return mov(x + ansdis * cos(ansang), y + ansdis * sin(ansang));
	else return make_pair(x + ansdis * cos(ansang), y + ansdis * sin(ansang));
}
pair<double, double> Car::Dynamic_Avoidance(int Cnum) {
	return Static_Avoidance(destination[Cnum], 0);
}
pair<double, double> Car::mov(int desk_num)
{
	double AlertRange = 4;
	int numID = -1;
	double vecv, v, d, vecv2, v2;
	double v1x, v1y, v2x, v2y, SinAng, CosAng;
	double ux, uy, vecux, vecuy;
	pair<double, double>Vecp;
	for (int i = 0; i < 4; i++)
		if (car[i].x == x && car[i].y == y) {
			numID = i; break;
		}
	//步骤一：判断当前小车是否需要进入动态回避模式
	for (int i = 0; i < 4; i++) {
		if (i == numID)continue;
		if (!ObCheck(x, y, car[i].x, car[i].y, desk_num, 0, 0))continue;
		Vecp = getVec(ang);
		vecv = CombineV(Vecp); v = CombineV(vx, vy);
		v1x = Vecp.first; v1y = Vecp.second;
		d = fabs(Cross(car[i].x - x, car[i].y - y, v1x, v1y)) / vecv;

		Vecp = getVec(car[i].ang);
		vecv2 = CombineV(Vecp); v2 = CombineV(car[i].vx, car[i].vy);
		v2x = Vecp.first; v2y = Vecp.second;

		SinAng = fabs(Cross(v2x, v2y, v1x, v1y)) / (vecv2 * vecv);
		CosAng = Dot(v1x, v1y, v2x, v2y) / (vecv2 * vecv);

		ux = v2 * CosAng;
		uy = v2 * SinAng;

		vecux = vecv2 * CosAng;
		vecuy = vecv2 * SinAng;

		if (Sign(Cross(car[i].x - x, car[i].y - y, v1x, v1y)) * Sign(Cross(v1x, v1y, v2x, v2y)) <= 0)
			uy *= -1, vecuy *= -1;

		if (Dot(Sub(des[numID], make_pair(x, y)), Sub(des[i], make_pair(car[i].x, car[i].y))) < 0 && Dist(x, y, car[i].x, car[i].y) < AlertRange / 2) {
			//加强动态避障判断（Dot(Sub(des[numID], make_pair(x, y)), Sub(des[i], make_pair(car[i].x, car[i].y)))还不够）※※※
			if (ChooseAvoider(i))continue;
			bool Check1 = !Search(x, y, GetR(goods) + 2.0 * GetR(car[i].goods) + 0.04), Check2 = !Search(car[i].x, car[i].y, 2.0 * GetR(goods) + GetR(car[i].goods) + 0.04);
			if (Check1 && Check2 && d >= GetR(goods) + GetR(car[i].goods))continue;
			if ((!Check1 || !Check2) && d >= GetR(goods) + GetR(car[i].goods) + 5)continue;//注意这里+1的参数调整
			if (Check1) {
				if (Check1 && Check2)continue;
				//else return make_pair(0, 0);//(23012830)
			}
			else return Dynamic_Avoidance(i);
		}
	}
	//步骤二：判断连接当前小车与目标工作台的线段是否经过障碍物
	if (!ObCheck(x, y, desk[desk_num].x, desk[desk_num].y, desk_num, GetR(goods) + epss, 0))
		return mov(des[numID].first, des[numID].second);

	
	/*
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			output << i << "-------------------" << endl;
			pair<int, int>k;
			output << "desk" << endl << endl;
			output << "nowpos" << endl;
			k = math_n::ztoe(x, y);
			output << k.first << " " << k.second << endl;
			output << map[k.first][k.second] << endl;
			k = math_n::ztoe(desk[desk_num].x, desk[desk_num].y);
			output << "target" << endl;
			output << k.first << " " << k.second << endl;
			output << map[k.first][k.second] << endl;
			output << endl;
		}
	}
	*/
	
	

	return mov(desk[desk_num].x, desk[desk_num].y);
}
void calc() {
	des[0] = car[0].Static_Avoidance(destination[0], 1);
	des[1] = car[1].Static_Avoidance(destination[1], 1);
	des[2] = car[2].Static_Avoidance(destination[2], 1);
	des[3] = car[3].Static_Avoidance(destination[3], 1);
}
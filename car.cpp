#include "Global.h"
#include "car.h"
#include "desk.h"
#include "namespace.h"

using namespace std;
const double eps = 1e-2;
const double epss = 0;
Car car[5];
static int dx[4] = { -1,0,1,0 }, dy[4] = { 0,1,0,-1 };
static int dxx[8] = { -1,0,1,1,1,0,-1,-1 }, dyy[8] = { -1,-1,-1,0,1,1,1,0 };
static int vis[N][N], t;
static int st[5];
static bool obfind;
pair<double, double>des[4] = { make_pair(-1,-1),make_pair(-1,-1),make_pair(-1,-1),make_pair(-1,-1) };



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
double Car::CalcRotate(double nx, double ny, int desk_num, double DeltaAng) {
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
	bool Check = (fabs(DeltaAng) < 1.56) && (tan(fabs(DeltaAng)) * diss <= 0.01) &&
		ObCheck(x, y, x + diss * cos(ang), y + diss * sin(ang), desk_num, GetR(goods), 0);
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
	double diss = Dist(nx, ny, x, y);
	pair<int, int>s = math_n::ztoe(x, y), t = math_n::ztoe(nx, ny);
	pair<double, double>reals = math_n::etoz(s.first, s.second), realt = math_n::etoz(t.first, t.second);

	double l = 0, r = 6, mid, resv = 0;
	while (r - l >= 0.01) {
		mid = (l + r) / 2;
		if (mid * mid / A * 0.5 > diss)r = mid;
		else {
			//这边可以加入mid与当前CombineV(vx,vy)大小的判断
			if (mid / A + (diss - mid * mid / A * 0.5) / mid >= 0.02)resv = mid, l = mid;//注意这里的参数调整
			else r = mid;
		}
	}

	return min(res, resv);
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
	if (Checkbool && Search(x2, y2, desk_num, GetR(goods)))return false;

	q.push(make_pair(x2, y2));
	q.push(make_pair(x1, y1));
	q.push(make_pair(x2 + Vertvec1.first * width * 0.99, y2 + Vertvec1.second * width * 0.99));
	q.push(make_pair(x1 + Vertvec1.first * width * 0.99, y1 + Vertvec1.second * width * 0.99));
	q.push(make_pair(x2 + Vertvec2.first * width * 0.99, y2 + Vertvec2.second * width * 0.99));
	q.push(make_pair(x1 + Vertvec2.first * width * 0.99, y1 + Vertvec2.second * width * 0.99));//这里需不需要乘0.99
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
			if (dis[Carry(goods)][desk_num][S.first][S.second] == 1e9 || map[S.first][S.second] == '#')
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
		if (dis[Carry(goods)][desk_num][S.first][S.second] == 1e9 || map[S.first][S.second] == '#')
			return false;
	}
	return true;
}
void Car::DFS(pair<int, int>meshc, pair<double, double>realc, int desk_num, double r) {
	vis[meshc.first][meshc.second] = t;
	if (map[meshc.first][meshc.second] == '#' || fabs(dis[Carry(goods)][desk_num][meshc.first][meshc.second] - 1e9) < eps) {
		obfind = true;
		return;
	}
	pair<double, double>p[6];
	for (int i = 0; i < 4; i++) {
		int nx = meshc.first + dx[i], ny = meshc.second + dy[i];
		if (nx < 0 || nx>101 || ny < 0 || ny>101)continue;
		if (vis[nx][ny] == t)continue;
		p[0] = math_n::etoz(nx, ny);
		p[1] = make_pair(p[0].first - 0.25, p[0].second - 0.25);
		p[2] = make_pair(p[0].first - 0.25, p[0].second + 0.25);
		p[3] = make_pair(p[0].first + 0.25, p[0].second + 0.25);
		p[4] = make_pair(p[0].first + 0.25, p[0].second - 0.25);
		p[5] = p[1];
		if (PointToSegment(realc, p[1], p[2]) < r || PointToSegment(realc, p[2], p[3]) < r ||
			PointToSegment(realc, p[3], p[4]) < r || PointToSegment(realc, p[4], p[5]) < r)
			DFS(make_pair(nx, ny), realc, desk_num, r);
		if (obfind)return;
	}
}
bool Car::Search(double x, double y, int desk_num, double r) {
	t++; obfind = false;
	DFS(math_n::ztoe(x, y), make_pair(x, y), desk_num, r);
	return obfind;
}


//旧版
void Car::CarCrashCheck(double& forwar, double& rot, int desk_num) {
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
		if (!ObCheck(x, y, car[i].x, car[i].y, desk_num, 0, 0))continue;

		int numm = i;
		bool Checkw = false;
		while (car[numm].FindAvoid) {
			Checkw |= (car[numm].Avoidnum == numID);
			numm = car[numm].Avoidnum;
		}

		if (car[i].FindAvoid == 3 && Checkw)continue;

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
		if ((d >= GetR(goods) + GetR(car[i].goods) + 0.5 && uy < eps) ||
			(uy >= eps && max(d - GetR(goods) - GetR(car[i].goods) - 0.5, 0.0) / uy >= AlertTime))
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
				if (FindAvoid || ((!car[numi].FindAvoid) && (goods > car[numi].goods || (goods == car[numi].goods && numID > numi))))forwar = 6;
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
		if (FindAvoid || ((!car[numi].FindAvoid) && (goods > car[numi].goods || (goods == car[numi].goods && numID > numi))))
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
void Car::MarginCheck(double& forwar, int desk_num) {
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
	while (r - l >= 0.05) {
		mid = (l + r) / 2;
		double tx = x + mid * cos(ang), ty = y + mid * sin(ang);
		if (tx < 0 || tx>50 || ty < 0 || ty>50)r = mid;
		else {
			if (ObCheck(x, y, x + (mid + 0.02) * cos(ang), y + (mid + 0.02) * sin(ang), desk_num, GetR(goods) + epss, 1))
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
			//这边可以加入mid与当前CombineV(vx,vy)大小的判断
			if (mid / A + (res - mid * mid / A * 0.5) / mid >= 0.02)resv = mid, l = mid;
			else r = mid;
		}
	}
	forwar = min(forwar, resv);
}
pair<double, double> Car::mov(double nx, double ny, int desk_num) {

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
	double rot = CalcRotate(nx, ny, desk_num, DeltaAng), forwar = CalcForward(nx, ny, DeltaAng);
	//小车碰撞判定

	/*
	output << "original set---------------" << endl;
	output << "ID=" << numID << endl;
	output << "forward=" << forwar << endl;
	output << "rotate=" << rot << endl;
	output << endl;
	*/
	if(!FindAvoid)
		CarCrashCheck(forwar, rot, desk_num);
	//边界碰撞判定

	//粗糙补丁（前往工作台的小车不需要减速）

	/*
	output << "CarCrashCheck---------------" << endl;
	output << "ID=" << numID << endl;
	output << "forward=" << forwar << endl;
	output << "rotate=" << rot << endl;
	output << endl;
	*/

	double checkforwar = forwar;

	MarginCheck(forwar, desk_num);

	if (fabs(checkforwar) > eps && fabs(forwar) < eps && fabs(w) < eps && fabs(rot) < eps)
		forwar = checkforwar;

	/*
	output << "Margin---------------" << endl;
	output << "ID=" << numID << endl;
	output << "forward=" << forwar << endl;
	output << "rotate=" << rot << endl;
	output << endl;
	*/

	return pair<double, double>(forwar, rot);
}


//新版
bool Car::ChooseAvoider(int Cnum) {
	if (FindAvoid)return true;
	int numID = -1;
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			numID = i;
			break;
		}
	}

	if (car[Cnum].FindAvoid) {
		int numm = Cnum;
		bool Chw = false;
		while (car[numm].FindAvoid) {
			Chw |= (car[numm].Avoidnum == numID);
			numm = car[numm].Avoidnum;
		}
		if (Chw)return true;
	}

	if (!car[Cnum].FindAvoid && (goods > car[Cnum].goods || (goods == car[Cnum].goods && numID > Cnum)))
		return true;
	return false;
}
bool Car::accessjudge(int desk_num, double Ang, double d, double deltaeps) {

	double tx = x + (d + deltaeps) * cos(Ang), ty = y + (d + deltaeps) * sin(Ang);
	double ttx = x + d * cos(Ang), tty = y + d * sin(Ang);
	if (tx < 0 || tx>50 || ty < 0 || ty>50) return false;//判断是否越界
	pair<int, int>s = math_n::ztoe(x, y), t = math_n::ztoe(tx, ty), tt = math_n::ztoe(ttx, tty);
	pair<double, double>sreal = math_n::etoz(s.first, s.second), treal = math_n::etoz(t.first, t.second), ttreal = math_n::etoz(tt.first, tt.second);

	//这样是否可以完全避免不选当前点
	if (s.first == tt.first && s.second == tt.second)
		return false;

	if ((dis[Carry(goods)][desk_num][s.first][s.second] - dis[Carry(goods)][desk_num][tt.first][tt.second]) / Dist(sreal.first, sreal.second, ttreal.first, ttreal.second) > 1.5)
		return false;
	//连续性的判断方法？（注意参数1.2与二分上界关联）※※※
	return ObCheck(x, y, tx, ty, desk_num, GetR(goods) + epss, 1);

}
pair<double, double> Car::Static_Avoidance(int desk_num, int mode) {

	double startang = Pi * 10, endang = Pi * 10;
	startang = -Pi;
	endang = Pi;



	int numID = -1;
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			numID = i;
			break;
		}
	}

	/*
	startang = -Pi;
	endang = Pi;
	*/
	
	
	//output << "numID=" << numID << endl;
	//output << endl;

	/*
	output << "startang=" << startang << endl;
	output << "endang=" << endang << endl;
	output << endl;
	*/
	

	double ansang = -1, ansdis = -1, maxdisdown = -1e9, disdown;
	double Delt = (endang - startang) / 90, l, r, mid, maxlen, res;
	//Delt划分过细会导致掉帧，划分过粗糙会导致有些角度无法达到从而使小车卡在较窄的隧道入口※※※※※※※

	double realtx, realty;
	while (startang < endang) {
		maxlen = -1; l = 0; r = 5;
		while (r - l >= 0.125) {
			mid = (l + r) / 2;
			if (accessjudge(desk_num, startang, mid, 0.02))maxlen = mid, l = mid;
			else r = mid;
		}

		if (maxlen == -1) {
			startang += Delt;
			continue;
		}

		res = -1;
		pair<double, double> s, t;
		pair<int, int>S, T;
		pair<double, double>p[6];
		int Crossnum = 1;

		s = make_pair(x, y);
		t = make_pair(x + maxlen * cos(startang), y + maxlen * sin(startang));
		S = math_n::ztoe(x, y);
		T = math_n::ztoe(x + maxlen * cos(startang), y + maxlen * sin(startang));
		while (S != T) {
			p[0] = math_n::etoz(S.first, S.second);
			p[1] = make_pair(p[0].first - 0.25, p[0].second - 0.25);
			p[2] = make_pair(p[0].first - 0.25, p[0].second + 0.25);
			p[3] = make_pair(p[0].first + 0.25, p[0].second + 0.25);
			p[4] = make_pair(p[0].first + 0.25, p[0].second - 0.25);
			p[5] = p[1];
			for (int i = 1; i <= 4; i++) {
				if (Dot(dx[i - 1], dy[i - 1], t.first - s.first, t.second - s.second) <= 0)
					continue;
				if (SegmentCross(s, t, p[i], p[i + 1])) {
					Crossnum = i;
					break;
				}
			}
			int nx = S.first + dx[Crossnum - 1], ny = S.second + dy[Crossnum - 1];
			if (dis[Carry(goods)][desk_num][S.first][S.second] <= dis[Carry(goods)][desk_num][nx][ny]) {
				res = Dist(CrossPoint(s, t, p[Crossnum], p[Crossnum + 1]), s) - 0.03;
				//这里有无更精准的移动方式？（增大参数或许可以在一定程度上避免小车过度行驶问题）※※※※※※※※
				break;
			}
			S = make_pair(nx, ny);
		}

		if (res == -1 || res > maxlen)
			res = maxlen;
		realtx = x + res * cos(startang);
		realty = y + res * sin(startang);

		/*
		output << "res=" << res << endl;

		
		output << "nowang=" << startang << endl;
		output << "maxlen=" << maxlen << endl;
		

		output << "x=" << x << " " << "y=" << y << endl;
		output << "realtx=" << realtx << " " << "realty=" << realty << endl;
		*/

		pair<int, int>ss = math_n::ztoe(x, y), tt = math_n::ztoe(realtx, realty);

		/*
		output << "ss=" << ss.first << " " << ss.second << endl;
		output << "tt=" << tt.first << " " << tt.second << endl;
		*/

		pair<double, double>ssreal = math_n::etoz(ss.first, ss.second), ttreal = math_n::etoz(tt.first, tt.second);

		/*
		output << "reals=" << ssreal.first << " " << ssreal.second << endl;
		output << "realt=" << ttreal.first << " " << ttreal.second << endl;
		*/


		disdown = dis[Carry(goods)][desk_num][ss.first][ss.second] - dis[Carry(goods)][desk_num][tt.first][tt.second];

		/*
		output << "dis1=" << dis[Carry(goods)][desk_num][ss.first][ss.second] << " " << "dis2=" << dis[Carry(goods)][desk_num][tt.first][tt.second] << endl;
		output << "disdown=" << disdown << endl;
		output << endl;
		*/

		if (disdown > maxdisdown || (fabs(disdown - maxdisdown) < eps && res < ansdis)) {
			ansang = startang;
			ansdis = res;
			maxdisdown = disdown;
		}
		startang += Delt;
	}

	/*
	output << "numID=" << numID << endl;
	output << "ansang=" << ansang << endl;
	output << "ansdis=" << ansdis << endl;
	output << "maxdisdown=" << maxdisdown << endl;
	output << endl;
	*/

	pair<double, double>dest;
	pair<int, int>temp = math_n::ztoe(x, y);
	pair<double, double>obnum, gettonum, dvec, nownum = make_pair(temp.first, temp.second);
	double mindis = dis[Carry(goods)][desk_num][temp.first][temp.second];

	if (maxdisdown < eps) {
		if (!goods) {
			for (int i = 0; i < 8; i += 2) {
				int nx = temp.first + dxx[i], ny = temp.second + dyy[i];
				if (nx < 0 || nx>101 || ny < 0 || ny>101)
					continue;
				if (map[nx][ny] == '#') {
					obnum = make_pair(nx, ny);
					break;
				}
			}
			for (int i = 1; i < 8; i += 2) {
				int nx = temp.first + dxx[i], ny = temp.second + dyy[i];
				if (nx < 0 || nx>101 || ny < 0 || ny>101)
					continue;
				if (dis[Carry(goods)][desk_num][nx][ny] < mindis) {
					mindis = dis[Carry(goods)][desk_num][nx][ny];
					gettonum = make_pair(nx, ny);
				}
			}
			dvec = Sub(gettonum, obnum);
			UnitV(dvec);
			dest = Add(multi(dvec, 0.5), make_pair(x, y));
		}
		else {
			for (int i = 0; i < 8; i += 2) {
				int nx = temp.first + dxx[i], ny = temp.second + dyy[i];
				if (nx < 0 || nx>101 || ny < 0 || ny>101)
					continue;
				if (map[nx][ny] == '#') {
					obnum = make_pair(nx, ny);
					break;
				}
			}
			if (fabs(obnum.first) > eps || fabs(obnum.second) > eps) {
				for (int i = 1; i < 8; i += 2) {
					int nx = temp.first + dxx[i], ny = temp.second + dyy[i];
					if (nx < 0 || nx>101 || ny < 0 || ny>101)
						continue;
					if (dis[Carry(goods)][desk_num][nx][ny] < mindis) {
						mindis = dis[Carry(goods)][desk_num][nx][ny];
						gettonum = make_pair(nx, ny);
					}
				}
			}
			else {
				for (int i = 1; i < 8; i += 2) {
					int nx = temp.first + dxx[i] + dxx[i], ny = temp.second + dyy[i] + dyy[i];
					if (nx < 0 || nx>101 || ny < 0 || ny>101)
						continue;
					if (dis[Carry(goods)][desk_num][nx][ny] < mindis) {
						mindis = dis[Carry(goods)][desk_num][nx][ny];
						gettonum = make_pair(nx, ny);
						int nxx1 = nx + dxx[(i + 2) % 8], nyy1 = ny + dyy[(i + 2) % 8];
						int nxx2 = nx + dxx[(i + 2) % 8] + dxx[(i + 2) % 8], nyy2 = ny + dyy[(i + 2) % 8] + ny + dyy[(i + 2) % 8];
						if ((nxx1 >= 0 && nxx1 <= 101 && nyy1 >= 0 && nyy1 <= 101 && map[nxx1][nyy1] == '#') ||
							(nxx2 >= 0 && nxx2 <= 101 && nyy2 >= 0 && nyy2 <= 101 && map[nxx2][nyy2] == '#')) {
							dvec = Sub(gettonum, make_pair(nxx1, nyy1));
						}
						else dvec = Sub(gettonum, make_pair(nx + dxx[(i + 6) % 8], ny + dyy[(i + 6) % 8]));
					}
				}
			}
			dvec = Sub(gettonum, obnum);
		}
	}
	if (fabs(dvec.first) > eps || fabs(dvec.second) > eps) {
		UnitV(dvec);
		dest = Add(multi(dvec, 0.5), make_pair(x, y));
	}
	else dest = make_pair(x + ansdis * cos(ansang), y + ansdis * sin(ansang));
	
	/*
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			pair<int, int>k;
			output << "Static" << endl << endl;
			pair<int, int>deskxy = math_n::ztoe(desk[desk_num].x, desk[desk_num].y);

			output<<"Target desk_xy="<<
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
	*/
	
	

	if (mode == 0)	return mov(dest.first, dest.second, desk_num);
	else return dest;
}
void Car::DFSAccess(pair<int, int>now, pair<double, double>realc, int avoid1, int avoid2, double r) {
	vis[now.first][now.second] = t;
	if (map[now.first][now.second] == '#') {
		obfind = true;
		return;
	}
	pair<double, double>p[6];
	for (int i = 0; i < 4; i++) {
		if (i == avoid1 || i == avoid2)continue;
		int nx = now.first + dx[i], ny = now.second + dy[i];
		if (nx < 0 || nx>101 || ny < 0 || ny>101)continue;
		if (vis[nx][ny] == t)continue;
		p[0] = math_n::etoz(nx, ny);
		p[1] = make_pair(p[0].first - 0.25, p[0].second - 0.25);
		p[2] = make_pair(p[0].first - 0.25, p[0].second + 0.25);
		p[3] = make_pair(p[0].first + 0.25, p[0].second + 0.25);
		p[4] = make_pair(p[0].first + 0.25, p[0].second - 0.25);
		p[5] = p[1];
		if (PointToSegment(realc, p[1], p[2]) < r || PointToSegment(realc, p[2], p[3]) < r ||
			PointToSegment(realc, p[3], p[4]) < r || PointToSegment(realc, p[4], p[5]) < r)
			DFSAccess(make_pair(nx, ny), realc, avoid1, avoid2, r);
		if (obfind)return;
	}
}
bool Car::SearchAccess(pair<int, int>now, pair<double, double>realc, int avoid1, int avoid2, double r) {
	t++; obfind = false;
	DFSAccess(now, realc, avoid1, avoid2, r);
	return obfind;
}
bool Car::AvoidCheck(double sx, double sy, double& len) {
	pair<int, int>xid = math_n::ztoe(sx, sy);
	pair<int, int>myid = math_n::ztoe(x, y);
	int tot = 0, p1 = -1, p2 = -1;
	for (int i = 0; i < 4; i++) {
		int nx1 = xid.first + dx[i], ny1 = xid.second + dy[i];
		int nx2 = xid.first + dx[i] + dx[i], ny2 = xid.second + dy[i] + dy[i];
		if (nx1 >= 0 && nx1 <= 101 && ny1 >= 0 && ny1 <= 101 && map[nx1][ny1] == '#') {
			pair<double, double>k1 = math_n::etoz(nx1, ny1);
			if (PointToSegment(make_pair(sx, sy), make_pair(k1.first - 0.25, k1.second - 0.25), make_pair(k1.first - 0.25, k1.second + 0.25)) < GetR(goods) + 0.02 ||
				PointToSegment(make_pair(sx, sy), make_pair(k1.first - 0.25, k1.second + 0.25), make_pair(k1.first + 0.25, k1.second + 0.25)) < GetR(goods) + 0.02 ||
				PointToSegment(make_pair(sx, sy), make_pair(k1.first + 0.25, k1.second + 0.25), make_pair(k1.first + 0.25, k1.second - 0.25)) < GetR(goods) + 0.02 ||
				PointToSegment(make_pair(sx, sy), make_pair(k1.first + 0.25, k1.second - 0.25), make_pair(k1.first - 0.25, k1.second - 0.25)) < GetR(goods) + 0.02) {
				tot++;
				if (p1 == -1)p1 = i;
				else p2 = i;
			}
		}
		else if (nx2 >= 0 && nx2 <= 101 && ny2 >= 0 && ny2 <= 101 && map[nx2][ny2] == '#') {
			pair<double, double>k2 = math_n::etoz(nx2, ny2);
			if (PointToSegment(make_pair(sx, sy), make_pair(k2.first - 0.25, k2.second - 0.25), make_pair(k2.first - 0.25, k2.second + 0.25)) < GetR(goods) + 0.02 ||
				PointToSegment(make_pair(sx, sy), make_pair(k2.first - 0.25, k2.second + 0.25), make_pair(k2.first + 0.25, k2.second + 0.25)) < GetR(goods) + 0.02 ||
				PointToSegment(make_pair(sx, sy), make_pair(k2.first + 0.25, k2.second + 0.25), make_pair(k2.first + 0.25, k2.second - 0.25)) < GetR(goods) + 0.02 ||
				PointToSegment(make_pair(sx, sy), make_pair(k2.first + 0.25, k2.second - 0.25), make_pair(k2.first - 0.25, k2.second - 0.25)) < GetR(goods) + 0.02) {
				tot++;
				if (p1 == -1)p1 = i;
				else p2 = i;
			}
		}
	}

	/*
	output << "tot=" << tot << endl;
	output << "p1=" << p1 << " " << "p2=" << p2 << endl;
	output << endl;
	*/

	if (!tot) return false;
	return !SearchAccess(xid, make_pair(sx, sy), p1, p2, GetR(goods) + 2 * GetR(car[Avoidnum].goods));
}
pair<double, double> Car::Dynamic_Avoidance(int mode) {
	pair<double, double>Vecp;
	double startang = -Pi, endang = Pi, deltaang = Pi / 4;
	pair<int, int>xzb = math_n::ztoe(x, y);
	pair<double, double>szb = math_n::etoz(xzb.first, xzb.second);
	double sx = szb.first, sy = szb.second;
	double maxlen, l, r, mid;
	bool dircheck;
	pair<double, double>lsta2 = lassta2, lsta3 = lassta3;

	int numID = -1;
	for (int i = 0; i < 4; i++) {
		if (x == car[i].x && y == car[i].y) {
			numID = i;
			break;
		}
	}

	int numm = numID;
	while (car[numm].FindAvoid)numm = car[numm].Avoidnum;
	pair<double, double>StaPo = Static_Avoidance(destination[numm], 1);
	pair<int, int>newnum = math_n::ztoe(StaPo.first, StaPo.second);
	pair<int, int>s1 = math_n::ztoe(lassta.first, lassta.second);
	pair<int, int>s2 = math_n::ztoe(lassta2.first, lassta2.second);
	pair<int, int>s3 = math_n::ztoe(lassta3.first, lassta3.second);
	if (fabs(newnum.first - s3.first) > eps || fabs(newnum.second - s3.second) > eps) {
		lassta3 = StaPo;
	}
	lassta2 = make_pair(x, y);
	pair<int, int>news2 = math_n::ztoe(lassta2.first, lassta2.second);
	if (fabs(s2.first - news2.first) > eps || fabs(s2.second - news2.second) > eps) {
		lassta = lsta2;
	}

	pair<int, int>xx = math_n::ztoe(x, y);
	pair<double, double>VecA = Sub(xx, math_n::ztoe(lassta.first, lassta.second));
	pair<double, double>VecA2 = Sub(xx, math_n::ztoe(lassta2.first, lassta2.second));
	pair<double, double>VecA3 = Sub(math_n::ztoe(lassta3.first, lassta3.second), xx);

	/*
	output << "x=" << x << " y=" << y << endl;
	output << "lassta=" << math_n::ztoe(lassta.first, lassta.second).first << " " << math_n::ztoe(lassta.first, lassta.second).second << endl;
	output << "map=" << map[math_n::ztoe(lassta.first, lassta.second).first][math_n::ztoe(lassta.first, lassta.second).second] << endl;
	output << "lassta2=" << math_n::ztoe(lassta2.first, lassta2.second).first << " " << math_n::ztoe(lassta2.first, lassta2.second).second << endl;
	output << "map=" << map[math_n::ztoe(lassta2.first, lassta2.second).first][math_n::ztoe(lassta2.first, lassta2.second).second] << endl;
	output << "lassta3=" << math_n::ztoe(lassta3.first, lassta3.second).first << " " << math_n::ztoe(lassta3.first, lassta3.second).second << endl;
	output << "map=" << map[math_n::ztoe(lassta3.first, lassta3.second).first][math_n::ztoe(lassta3.first, lassta3.second).second] << endl;
	output << endl;


	output << "VecA=" << VecA.first << " " << VecA.second << endl;
	output << "VecA2=" << VecA2.first << " " << VecA2.second << endl;
	output << "VecA3=" << VecA3.first << " " << VecA3.second << endl;
	output << endl;
	*/

	if (fabs(VecA3.first) < eps && fabs(VecA3.second) < eps)
		VecA3 = VecA;

	pair<double, double>dir = getVec(ang);
	pair<double, double>cardir = make_pair(x - car[Avoidnum].x, y - car[Avoidnum].y);
	if (!ObCheck(x, y, car[Avoidnum].x, car[Avoidnum].y, 51, 0, 0))cardir = make_pair(0.0, 0.0);

	if (Dot(VecA, VecA3) <= 0 || Dot(VecA, dir) <= 0 || Dot(VecA3, dir) <= 0) {
		if (mode == 0)
			return mov(StaPo.first, StaPo.second, destination[numm]);
		else return make_pair(StaPo.first, StaPo.second);
	}

	while (startang < endang) {
		Vecp = getVec(startang);
		if ((Dot(Vecp, VecA) < 0 && fabs(Dot(Vecp, VecA)) > eps) || (Dot(Vecp, VecA2) < 0 && fabs(Dot(Vecp, VecA2)) > eps) ||
			(Dot(Vecp, VecA3) < 0 && fabs(Dot(Vecp, VecA3)) > eps) || (Dot(Vecp, cardir) < 0 && fabs(Dot(Vecp, cardir) > eps))) {
			startang += deltaang;
			continue;
		}

		l = 0; r = 4; maxlen = -1;
		while (r - l >= 0.01) {
			mid = (l + r) / 2;
			double tx = sx + mid * cos(startang), ty = sy + mid * sin(startang);
			pair<int, int>tt = math_n::ztoe(tx, ty);
			if (tx < 0 || tx>50 || ty < 0 || ty>50) {
				r = mid;
				continue;
			}
			if (fabs(xzb.first - tt.first) < eps && fabs(xzb.second - tt.second) < eps) {
				l = mid;
				continue;
			}
			//可能需要加入连续性判断
			if (ObCheck(sx, sy, tx, ty, destination[numm], GetR(goods), 1))maxlen = mid, l = mid;
			else r = mid;
		}

		if (maxlen == -1) {
			startang += deltaang;
			continue;
		}

		/*
		output << endl;
		output << "nowang=" << startang << endl;
		output << "maxlen=" << maxlen << endl;
		output << "realS=" << x << " " << y << endl;
		output << "realT=" << sx + maxlen * cos(startang) << " " << sy + maxlen * sin(startang) << endl;
		output << endl;
		*/

		//不能选择工作台做避让点，0.4为容忍参数
		if (Dist(sx + maxlen * cos(startang), sy + maxlen * sin(startang), desk[destination[numm]].x, desk[destination[numm]].y) < 0.4) {
			startang += deltaang;
			continue;
		}

		if (AvoidCheck(sx + maxlen * cos(startang), sy + maxlen * sin(startang), maxlen)) {
			setto = make_pair(sx + maxlen * cos(startang), sy + maxlen * sin(startang));
			FindAvoid = 2;
			break;
		}
		startang += deltaang;
	}

	
	startang = -Pi;
	deltaang = Pi / 2;

	while (startang < endang) {
		Vecp = getVec(startang);
		//output << "Vecp=" << Vecp.first << " " << Vecp.second << endl;
		if ((Dot(Vecp, VecA) < 0 && fabs(Dot(Vecp, VecA)) > eps) || (Dot(Vecp, VecA2) < 0 && fabs(Dot(Vecp, VecA2)) > eps) ||
			(Dot(Vecp, VecA3) < 0 && fabs(Dot(Vecp, VecA3)) > eps) || (Dot(Vecp, cardir) < 0 && fabs(Dot(Vecp, cardir) > eps))) {
				startang += deltaang;
				continue;
		}
		l = 0; r = 50; maxlen = -1;
		while (r - l >= 0.01) {
			mid = (l + r) / 2;
			double tx = sx + mid * cos(startang), ty = sy + mid * sin(startang);
			pair<int, int>tt = math_n::ztoe(tx, ty);
			if (tx < 0 || tx>50 || ty < 0 || ty>50) {
				r = mid;
				continue;
			}
			if (fabs(xzb.first - tt.first) < eps && fabs(xzb.second - tt.second) < eps) {
				l = mid;
				continue;
			}
			if (ObCheck(sx, sy, tx, ty, destination[numm], GetR(goods), 1))maxlen = mid, l = mid;
			else r = mid;
		}

		if (maxlen == -1) {
			startang += deltaang;
			continue;
		}
		
		pair<double, double>realS = make_pair(x, y), realT = make_pair(sx + maxlen * cos(startang), sy + maxlen * sin(startang));
		pair<int, int>S = math_n::ztoe(realS.first, realS.second), T = math_n::ztoe(realT.first, realT.second);
		pair<double, double>p[6], goodPoint = make_pair(-1.0, -1.0);
		int Crossnum = 1;

		while (S != T) {
			p[0] = math_n::etoz(S.first, S.second);
			p[1] = make_pair(p[0].first - 0.25, p[0].second - 0.25);
			p[2] = make_pair(p[0].first - 0.25, p[0].second + 0.25);
			p[3] = make_pair(p[0].first + 0.25, p[0].second + 0.25);
			p[4] = make_pair(p[0].first + 0.25, p[0].second - 0.25);
			p[5] = p[1];
			for (int i = 1; i <= 4; i++) {
				if (Dot(dx[i - 1], dy[i - 1], realT.first - realS.first, realT.second - realS.second) <= 0)
					continue;
				if (SegmentCross(realS, realT, p[i], p[i + 1])) {
					Crossnum = i;
					break;
				}
			}
			int nx = S.first + dx[Crossnum - 1], ny = S.second + dy[Crossnum - 1];
			if (dis[Carry(goods)][destination[numm]][S.first][S.second] <= dis[Carry(goods)][destination[numm]][nx][ny]) {
				goodPoint = CrossPoint(realS, realT, p[Crossnum], p[Crossnum + 1]);
				break;
			}
			S = make_pair(nx, ny);
		}

		/*
		output << "nowang=" << startang << endl;
		output << "maxlen=" << maxlen << endl;
		output << "realS=" << realS.first << " " << realS.second << endl;
		output << "realT=" << realT.first << " " << realT.second << endl;
		output << "goodPoint=" << goodPoint.first << " " << goodPoint.second << endl;
		output << endl;
		*/

		if (fabs(goodPoint.first + 1) > eps && Dist(goodPoint, realT) > 2.0 * GetR(car[Avoidnum].goods) + 0.03) {
			setto = make_pair(x + (Dist(realS, goodPoint) + 2.0 * GetR(car[Avoidnum].goods) + 0.03) * cos(startang),
				y + (Dist(realS, goodPoint) + 2.0 * GetR(car[Avoidnum].goods) + 0.03) * sin(startang));
			FindAvoid = 2;
			break;
		}
		startang += deltaang;
	}


	if (mode == 0) {
		if (FindAvoid == 1)return mov(StaPo.first, StaPo.second, destination[numm]);
		else {
			if ((fabs(x - setto.first) > 0.01 && fabs(y - setto.second) > 0.01)) {
				pair<double, double>vecx = make_pair(setto.first - x, 0), vecy = make_pair(0, setto.second - y);
				UnitV(vecx); UnitV(vecy); multi(vecx, 0.02); multi(vecy, 0.02);
				pair<double, double>po1 = Add(make_pair(x, y), vecx), po2 = Add(make_pair(x, y), vecy);
				bool check1 = !Search(po1.first, po1.second, 51, GetR(goods)), check2 = !Search(po2.first, po2.second, 51, GetR(goods));
				if (check1 && check2)return mov(setto.first, setto.second, destination[numm]);
				if (check1)return mov(setto.first, y, destination[numm]);
				if (check2)return mov(x, setto.second, destination[numm]);
			}
			else return mov(setto.first, setto.second, destination[numm]);
		}
	}
	else {
		if (FindAvoid == 1)return make_pair(StaPo.first, StaPo.second);
		else return make_pair(setto.first, setto.second);
	}
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

	/*
	pair<int, int>df = math_n::ztoe(x, y);
	output << "numID=" << numID << endl;
	output << "dis=" << dis[Carry(goods)][desk_num][df.first][df.second] << endl;
	output << endl;
	*/

	
	output << numID << "--------------------------------------------" << endl;
	output << "FindAvoid=" << FindAvoid << endl;
	output << "Avoidnum=" << Avoidnum << endl;
	output << "setto=" << setto.first << " " << setto.second << endl;
	output << "Reach=" << Reach << endl;
	output << endl;
	



	//步骤一：判断当前小车是否处于回避模式/是否需要进入动态回避模式
	for (int i = 0; i < 4; i++) {
		if (numID == i)continue;
		if (!car[i].FindAvoid)continue;
		if (car[i].Avoidnum != numID)continue;
		int numm = numID;
		while (car[numm].FindAvoid)numm = car[numm].Avoidnum;
		if ((!car[i].Reach) && Dist(x, y, car[i].x, car[i].y) <= 4 && ObCheck(x, y, car[i].x, car[i].y, 51, 0, 0) &&
			Dot(car[i].x - x, car[i].y - y, des[numm].first, des[numm].second) > 0)
			return make_pair(0.0, 0.0);
	}

	int firstnum = numID;
	while (car[firstnum].FindAvoid)firstnum = car[firstnum].Avoidnum;

	if (FindAvoid > 1) {
		if (Reach)
			return make_pair(0.0, 0.0);
		else {
			if ((fabs(x - setto.first) > 0.01 && fabs(y - setto.second) > 0.01)) {
				pair<double, double>vecx = make_pair(setto.first - x, 0), vecy = make_pair(0, setto.second - y);
				UnitV(vecx); UnitV(vecy); multi(vecx, 0.02); multi(vecy, 0.02);
				pair<double, double>po1 = Add(make_pair(x, y), vecx), po2 = Add(make_pair(x, y), vecy);
				bool check1 = !Search(po1.first, po1.second, 51, GetR(goods)), check2 = !Search(po2.first, po2.second, 51, GetR(goods));
				if (check1 && check2)return mov(setto.first, setto.second, destination[firstnum]);
				if (check1)return mov(setto.first, y, destination[firstnum]);
				if (check2)return mov(x, setto.second, destination[firstnum]);
			}
			else return mov(setto.first, setto.second, destination[firstnum]);
		}
	}
	else if (FindAvoid == 1)
		return Static_Avoidance(destination[firstnum], 0);

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
		

		int numm = i;
		while (car[numm].FindAvoid)numm = car[numm].Avoidnum;
		pair<double, double>tr = car[i].Static_Avoidance(destination[numm], 1);
		pair<double, double>vec1 = Sub(des[numID], make_pair(x, y)), vec2 = Sub(tr, make_pair(car[i].x, car[i].y));

		if (Dot(vec1, Sub(make_pair(car[i].x, car[i].y), make_pair(x, y))) > 0 && Dot(vec2, Sub(make_pair(x, y), make_pair(car[i].x, car[i].y))) > 0 &&
			PointToLine(make_pair(x, y), make_pair(car[i].x, car[i].y), vec2) < GetR(goods) + GetR(car[i].goods) &&
			PointToLine(make_pair(car[i].x, car[i].y), make_pair(x, y), vec1) < GetR(goods) + GetR(car[i].goods) &&
			/*Dot(v1x, v1y, car[i].x - x, car[i].y - y) > 0 && Dot(v2x, v2y, x - car[i].x, y - car[i].y) > 0 &&*/
			/*Dot(v1x, v1y, v2x, v2y) < 0 && fabs(v) > eps && fabs(v2) > eps &&*/
			Dist(x, y, car[i].x, car[i].y) < 3 && (Search(x, y, desk_num, GetR(goods) + 2.0 * GetR(car[i].goods) + 0.03) ||
				Search(car[i].x, car[i].y, desk_num, 2.0 * GetR(goods) + GetR(car[i].goods) + 0.03))) {
			if (!ChooseAvoider(i)) {
				FindAvoid = 1;
				Reach = false;
				lassta3 = lassta2 = lassta = make_pair(car[i].x, car[i].y);
				Avoidnum = i;
				goodsrec = car[Avoidnum].goods;
			}
		}
	}

	if (FindAvoid == 1)
		return Dynamic_Avoidance(0);
	if (FindAvoid && Dist(x, y, setto.first, setto.second) < 0.4)
		Reach = true;
	if (FindAvoid == 2 && Dist(x, y, car[Avoidnum].x, car[Avoidnum].y) <= 4)
		FindAvoid = 3;
	if (FindAvoid == 3 && (Dist(x, y, car[Avoidnum].x, car[Avoidnum].y) > 4 || goodsrec != car[Avoidnum].goods))
		FindAvoid = 0;
	if (FindAvoid) {
		if (Reach)
			return make_pair(0.0, 0.0);
		else {
			if ((fabs(x - setto.first) > 0.01 && fabs(y - setto.second) > 0.01)) {
				pair<double, double>vecx = make_pair(setto.first - x, 0), vecy = make_pair(0, setto.second - y);
				UnitV(vecx); UnitV(vecy); multi(vecx, 0.02); multi(vecy, 0.02);
				pair<double, double>po1 = Add(make_pair(x, y), vecx), po2 = Add(make_pair(x, y), vecy);
				bool check1 = !Search(po1.first, po1.second, 51, GetR(goods)), check2 = !Search(po2.first, po2.second, 51, GetR(goods));
				if (check1 && check2)return mov(setto.first, setto.second, destination[firstnum]);
				if (check1)return mov(setto.first, y, destination[firstnum]);
				if (check2)return mov(x, setto.second, destination[firstnum]);
			}
			else return mov(setto.first, setto.second, destination[firstnum]);
		}
	}


	//步骤二：判断连接当前小车与目标工作台的线段是否经过障碍物
	if (!ObCheck(x, y, desk[desk_num].x, desk[desk_num].y, desk_num, GetR(goods) + epss, 0))
		return mov(des[numID].first, des[numID].second, desk_num);
	return mov(desk[desk_num].x, desk[desk_num].y, desk_num);
}
void calc() {

	//复杂度优化：当某个点在动态回避时这里也许不需要计算静态回避※※※※※※※※※※※
	des[0] = car[0].Static_Avoidance(destination[0], 1);
	des[1] = car[1].Static_Avoidance(destination[1], 1);
	des[2] = car[2].Static_Avoidance(destination[2], 1);
	des[3] = car[3].Static_Avoidance(destination[3], 1);

	//提前更新状态
	for (int i = 0; i < 4; i++) {
		if (car[i].FindAvoid && Dist(car[i].x, car[i].y, car[i].setto.first, car[i].setto.second) < 0.4)
			car[i].Reach = true;
		if (car[i].FindAvoid == 2 && Dist(car[i].x, car[i].y, car[car[i].Avoidnum].x, car[car[i].Avoidnum].y) <= 4)
			car[i].FindAvoid = 3;
		if (car[i].FindAvoid == 3 && (Dist(car[i].x, car[i].y, car[car[i].Avoidnum].x, car[car[i].Avoidnum].y) > 4 || car[i].goodsrec != car[car[i].Avoidnum].goods))
			car[i].FindAvoid = 0;
	}
	for (int i = 0; i < 4; i++) {
		if (car[i].FindAvoid == 1)
			car[i].Dynamic_Avoidance(0);
	}

	//死胡同情况特判
	for (int i = 0; i < 4; i++) {
		if (car[i].FindAvoid == 1) {
			int tt = car[i].Avoidnum;
			while (car[tt].FindAvoid)tt = car[tt].Avoidnum;
			if (car[i].workbench != destination[tt])
				continue;
			int numm = i, len = 0;
			st[len = 1] = numm;
			while (car[numm].FindAvoid) {
				numm = car[numm].Avoidnum;
				st[++len] = numm;
			}
			car[i].FindAvoid = 0;
			for (int j = 2; j <= len; j++) {
				car[st[j]].FindAvoid = 1;
				car[st[j]].Avoidnum = st[j - 1];
				car[st[j]].goodsrec = car[st[j - 1]].goods;
				car[st[j]].Reach = false;
				car[st[j]].lassta3 = car[st[j]].lassta2 = car[st[j]].lassta = make_pair(car[st[j - 1]].x, car[st[j - 1]].y);
			}
		}
	}


	t = 0;

	/*
	output << "frame_number is " << frame_number << endl;
	output << des[3].first << ' ' << des[3].second << endl;
	output << endl;
	*/
}
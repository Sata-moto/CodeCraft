#include "Global.h"
#include "namespace.h"
#include "desk.h"
#include "geometry.h"
#include "car.h"


using namespace std;
static const int Carnum = 4;
static const double eps = 1e-2;
static const double epss = 0;

Car car[5];
static const int dx[4] = { -1,0,1,0 }, dy[4] = { 0,1,0,-1 };
static const int dxx[8] = { -1,0,1,1,1,0,-1,-1 }, dyy[8] = { -1,-1,-1,0,1,1,1,0 };
static pair<double, double>des[Carnum + 1] = { make_pair(-1,-1),make_pair(-1,-1),make_pair(-1,-1),make_pair(-1,-1) };
static int vis[N][N], t;
static int st[Carnum + 1];
static bool obfind;
static int set0[Carnum + 1];
static int revAvoid[Carnum + 1][Carnum + 1];

//动态避障1变量组
static const double AlertRange = 4;
static const double AlertTime = 1.2;
static double I, B;
static double SinAng, CosAng, vecux, vecuy, ux, uy, vecv, v, d, vecv2, v2;
static double v1x, v1y, v2x, v2y;


//状态更新及不经过障碍物（可能经过小车）的路径目标点计算
bool CheckAvoidTree(int now, int id) {
	for (int i = 1; i <= revAvoid[now][0]; i++) {
		int numm = revAvoid[now][i];
		//判定距离大于2但小于4时等待回避自己的小车找到回避点；若距离小于2则尝试回避正在回避自己的小车
		bool Check = (!car[numm].Reach) && Dist(car[id].x, car[id].y, car[numm].x, car[numm].y) <= 4 && Dist(car[id].x, car[id].y, car[numm].x, car[numm].y) >= 2 &&
			car[id].ObCheck(car[id].x, car[id].y, car[numm].x, car[numm].y, 51, 0, 0) &&
			Dot(car[numm].x - car[id].x, car[numm].y - car[id].y, des[id].first, des[id].second) > 0;
		if (Check)return true;
		Check |= CheckAvoidTree(numm, id);
		if (Check)return true;
	}
	return false;
}
void CheckRunningCrash() {

	//等待的小车或许也可以清？但是可能会干扰原来的逻辑
	//更准确的判断死机方式？
	for (int i = 0; i < 4; i++) {
		bool Check = false;
		for (int j = 0; j < 4; j++) {
			if (i == j)continue;
			if (Dist(car[i].x, car[i].y, car[j].x, car[j].y) < car[i].GetR(car[i].goods) + car[j].GetR(car[j].goods) + 0.03) {
				Check = true;
				break;
			}
		}
		if ((fabs(car[i].lasx - car[i].x) < 0.1 && fabs(car[i].lasy - car[i].y) < 0.1) || (fabs(car[i].vx) < 0.3 && fabs(car[i].vy) < 0.3) || Check) {
			if (!set0[i])set0[i] = frame_number;
			if (((car[i].FindAvoid && !car[i].Reach) || (!car[i].FindAvoid)) && (frame_number - set0[i] >= 250) && set0[i]) {
				set0[i] = 0;
				car[i].FindAvoid = 0;
				car[i].Reach = false;
				car[i].lassta3 = car[i].lassta2 = car[i].lassta = make_pair(0.0, 0.0);
				car[i].Avoidnum = -1;
				car[i].goodsrec = -1;
			}
			if ((car[i].FindAvoid && car[i].Reach) && (frame_number - set0[i] >= 400) && set0[i]) {
				set0[i] = 0;
				car[i].FindAvoid = 0;
				car[i].Reach = false;
				car[i].lassta3 = car[i].lassta2 = car[i].lassta = make_pair(0.0, 0.0);
				car[i].Avoidnum = -1;
				car[i].goodsrec = -1;
			}
		}
		else set0[i] = 0;
		car[i].lasx = car[i].x;
		car[i].lasy = car[i].y;
	}
}
void DynamicStatusUpdate() {

	//一般情况下的动态避让状态转移
	for (int i = 0; i < 4; i++) {
		if (car[i].FindAvoid >= 2 && Dist(car[i].x, car[i].y, car[i].setto.first, car[i].setto.second) < 0.4)
			car[i].Reach = true;
		if (car[i].FindAvoid == 2 && Dist(car[i].x, car[i].y, car[car[i].Avoidnum].x, car[car[i].Avoidnum].y) <= 4)
			car[i].FindAvoid = 3;
		if (car[i].FindAvoid == 3 && (Dist(car[i].x, car[i].y, car[car[i].Avoidnum].x, car[car[i].Avoidnum].y) > 4 || car[i].goodsrec != car[car[i].Avoidnum].goods)) {
			car[i].FindAvoid = 0;
			car[i].Reach = false;
			car[i].lassta3 = car[i].lassta2 = car[i].lassta = make_pair(0.0, 0.0);
			car[i].Avoidnum = -1;
			car[i].goodsrec = -1;
		}
	}


	//死胡同情况下的动态避让状态转移
	//这里构成一定的问题，问题在于若之前小车回避构成树形结构则倒置后会因父节点的儿子节点不唯一而出现倒置缺失
	for (int i = 0; i < 4; i++) {
		if (car[i].FindAvoid == 1) {
			int tt = car[i].Avoidnum;
			while (car[tt].FindAvoid)tt = car[tt].Avoidnum;
			if (car[i].workbench != destination[tt])continue;
			if ((car[i].goods || car[i].lasgoods) && car[i].goods == car[i].lasgoods)continue;
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

	for (int i = 0; i < 4; i++) {
		car[i].lasgoods = car[i].goods;
	}

}
void CalculateTrueDestination() {

	//计算所有点的真正目的地
	//注意：该目的地为不经过任何障碍物的目的地，故可以直接调用最初版本的mov
	for (int i = 0; i < 4; i++) {
		if (!car[i].FindAvoid) {
			if (!car[i].ObCheck(car[i].x, car[i].y, desk[destination[i]].x, desk[destination[i]].y, destination[i], car[i].GetR(car[i].goods) + epss, 0))
				des[i] = car[i].Static_Avoidance(destination[i], 1);
			else des[i] = make_pair(desk[destination[i]].x, desk[destination[i]].y);
		}
		else {
			if (car[i].FindAvoid == 1)
				des[i] = car[i].Dynamic_Avoidance();
			else {
				if (car[i].Reach)
					des[i] = make_pair(car[i].x, car[i].y);
				else {
					if ((fabs(car[i].x - car[i].setto.first) > 0.01 && fabs(car[i].y - car[i].setto.second) > 0.01)) {
						pair<double, double>vecx = make_pair(car[i].setto.first - car[i].x, 0), vecy = make_pair(0, car[i].setto.second - car[i].y);
						UnitV(vecx); UnitV(vecy); multi(vecx, 0.02); multi(vecy, 0.02);
						pair<double, double>po1 = Add(make_pair(car[i].x, car[i].y), vecx), po2 = Add(make_pair(car[i].x, car[i].y), vecy);
						bool check1 = !car[i].Search(po1.first, po1.second, 51, car[i].GetR(car[i].goods));
						bool check2 = !car[i].Search(po2.first, po2.second, 51, car[i].GetR(car[i].goods));
						if (check1 && check2)des[i] = car[i].setto;
						else if (check1)des[i] = make_pair(car[i].setto.first, car[i].y);
						else if (check2)des[i] = make_pair(car[i].x, car[i].setto.second);
					}
					else des[i] = car[i].setto;
				}
			}
		}

		/*
		output << "Calculate True Destination " << i << " Done" << endl << endl;
		output << "*********************************" << endl << endl;
		*/
	}

	
	//加入被避让小车的停止考虑
	for (int i = 0; i < 4; i++)
		revAvoid[i][0] = 0;
	for (int i = 0; i < 4; i++)
		if (car[i].FindAvoid)
			revAvoid[car[i].Avoidnum][++revAvoid[car[i].Avoidnum][0]] = i;

	for (int i = 0; i < 4; i++)
		if (CheckAvoidTree(i, i))
			des[i] = make_pair(car[i].x, car[i].y);
	
}
void IntoDynamicCheckAndUpdate() {

	double AlertRange = 4;
	double vecv, v, d, vecv2, v2;
	double v1x, v1y, v2x, v2y, SinAng, CosAng;
	double ux, uy, vecux, vecuy;
	pair<double, double>Vecp;

	for (int i = 0; i < 4; i++) {

		if (car[i].FindAvoid)continue;

		Vecp = getVec(car[i].ang);
		vecv = CombineV(Vecp); v = CombineV(car[i].vx, car[i].vy);
		v1x = Vecp.first; v1y = Vecp.second;

		for (int j = 0; j < 4; j++) {

			if (j == i)continue;
			if (!car[i].ObCheck(car[i].x, car[i].y, car[j].x, car[j].y, 51, 0, 0))continue;

			d = fabs(Cross(car[j].x - car[i].x, car[j].y - car[i].y, v1x, v1y)) / vecv;

			Vecp = getVec(car[j].ang);
			vecv2 = CombineV(Vecp); v2 = CombineV(car[j].vx, car[j].vy);
			v2x = Vecp.first; v2y = Vecp.second;

			SinAng = fabs(Cross(v2x, v2y, v1x, v1y)) / (vecv2 * vecv);
			CosAng = Dot(v1x, v1y, v2x, v2y) / (vecv2 * vecv);

			ux = v2 * CosAng;
			uy = v2 * SinAng;
			vecux = vecv2 * CosAng;
			vecuy = vecv2 * SinAng;

			if (Sign(Cross(car[j].x - car[i].x, car[j].y - car[i].y, v1x, v1y)) * Sign(Cross(v1x, v1y, v2x, v2y)) <= 0)
				uy *= -1, vecuy *= -1;

			pair<double, double> vec1 = Sub(des[i], make_pair(car[i].x, car[i].y));
			pair<double, double> vec2 = Sub(des[j], make_pair(car[j].x, car[j].y));


			//遇到不停在避让点且正对着的小车且没有避让空间时考虑动态避让
			bool check1 = (!car[j].FindAvoid || (car[j].FindAvoid && !car[j].Reach))&&
				Dot(vec1, Sub(make_pair(car[j].x, car[j].y), make_pair(car[i].x, car[i].y))) > 0 &&
				Dot(vec2, Sub(make_pair(car[i].x, car[i].y), make_pair(car[j].x, car[j].y))) > 0 &&
				(PointToLine(make_pair(car[i].x, car[i].y), make_pair(car[j].x, car[j].y), vec2) < car[i].GetR(car[i].goods) + car[j].GetR(car[j].goods) ||
				PointToLine(make_pair(car[j].x, car[j].y), make_pair(car[i].x, car[i].y), vec1) < car[i].GetR(car[i].goods) + car[j].GetR(car[j].goods)) &&
				/*Dot(v1x, v1y, car[j].x - car[i].x, car[j].y - car[i].y) > 0 && Dot(v2x, v2y, car[i].x - car[j].x, car[i].y - car[j].y) > 0 &&*/
				/*Dot(v1x, v1y, v2x, v2y) < 0 && fabs(v) > eps && fabs(v2) > eps &&*/
				Dist(car[i].x, car[i].y, car[j].x, car[j].y) < 3 &&
				(car[i].Search(car[i].x, car[i].y, 51, car[i].GetR(car[i].goods) + 2.0 * car[j].GetR(car[j].goods) + 0.03) || car[j].Search(car[j].x, car[j].y, 51, 2.0 * car[i].GetR(car[i].goods) + car[j].GetR(car[j].goods) + 0.03));

			//遇到停在避让点的小车且没有避让空间时考虑动态避让
			bool check2 = (car[j].FindAvoid && car[j].Reach) &&
				Dot(vec1, Sub(make_pair(car[j].x, car[j].y), make_pair(car[i].x, car[i].y))) > 0 &&
				PointToLine(make_pair(car[j].x, car[j].y), make_pair(car[i].x, car[i].y), vec1) < car[i].GetR(car[i].goods) + car[j].GetR(car[j].goods) &&
				Dist(car[i].x, car[i].y, car[j].x, car[j].y) < 3 &&
				(car[i].Search(car[i].x, car[i].y, 51, car[i].GetR(car[i].goods) + 2.0 * car[j].GetR(car[j].goods) + 0.03) || car[j].Search(car[j].x, car[j].y, 51, 2.0 * car[i].GetR(car[i].goods) + car[j].GetR(car[j].goods) + 0.03));

			//进入动态避障的判定还需要修改
			if (check1 || check2) {
				if (!car[i].ChooseAvoider(j)) {
					car[i].FindAvoid = 1;
					car[i].Reach = false;
					car[i].lassta3 = car[i].lassta2 = car[i].lassta = make_pair(car[j].x, car[j].y);
					car[i].Avoidnum = j;
					car[i].goodsrec = car[j].goods;
				}
			}
		}

		if (car[i].FindAvoid == 1)
			des[i] = car[i].Dynamic_Avoidance();
	}
}


//小车
int Car::Carry(int x) {
	return x == 0 ? 0 : 1;
}
double Car::GetR(int k) {
	return k == 0 ? 0.45 : 0.53;
}
int Car::FindnumID() {
	for (int i = 0; i < 4; i++)
		if (car[i].x == x && car[i].y == y)
			return i;
}
int Car::Findfirstnum(int numID) {
	int firstnum = numID;
	while (car[firstnum].FindAvoid)firstnum = car[firstnum].Avoidnum;
	return firstnum;
}
double Car::CalcAng(double nx, double ny) {
	double res = atan2(ny - y, nx - x) - ang;
	AdjuAng(res);
	return res;
}
double Car::CalcRotate(double nx, double ny, int desk_num, double DeltaAng) {
	
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
double Car::CalcForward(double nx, double ny, int desk_num, double DeltaAng) {

	
	if (goods < 4 && (fabs(desk[desk_num].x - nx) > eps || fabs(desk[desk_num].y - ny) > eps))
		return cos(DeltaAng) * (fabs(DeltaAng) > Pi / 2 ? 0 : 6);
	

	double res = cos(DeltaAng) * (fabs(DeltaAng) > Pi / 5 ? 0 : 6);

	double Cv = CombineV(vx, vy), M = pow(GetR(goods), 2) * Pi * 20, A = 250.0 / M;
	double diss = Dist(nx, ny, x, y);
	double l = 0, r = 6, mid, resv = 0;
	while (r - l >= 0.01) {
		mid = (l + r) / 2;
		if (mid * mid / A * 0.5 > diss)r = mid;
		else {
			//这边可以加入mid与当前CombineV(vx,vy)大小的判断
			if (((diss - mid * mid / A * 0.5) > 0) && (mid / A + (diss - mid * mid / A * 0.5) / mid >= 0.02))
				resv = mid, l = mid;//注意这里的参数调整
			else r = mid;
		}
	}

	return min(res, resv);
}


//静态避障（障碍物避障）
void Car::MarginCheck(double& forwar, int desk_num) {

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
bool Car::ObCheck(double x1, double y1, double x2, double y2, int desk_num, double width, bool Checkbool) {

	queue<pair<double, double> >q;
	pair<double, double> p[6];
	pair<int, int>S, T;
	pair<double, double>realS, realT;
	pair<double, double>Dvec = make_pair(x2 - x1, y2 - y1);
	UnitV(Dvec);
	pair<double, double>Vertvec1 = Rotate(Dvec, Pi / 2);//逆时针90度
	pair<double, double>Vertvec2 = Rotate(Dvec, -Pi / 2);//顺时针90度


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
		
		S = math_n::ztoe(realS.first, realS.second);
		T = math_n::ztoe(realT.first, realT.second);

		int Crossnum = 1;
		
		while (S != T) {
			
			if (dis[Carry(goods)][desk_num][S.first][S.second] == 1e9 || map[S.first][S.second] == '#')
				return false;

			p[0] = math_n::etoz(S.first, S.second);
			p[1] = make_pair(p[0].first - 0.25, p[0].second - 0.25);
			p[2] = make_pair(p[0].first - 0.25, p[0].second + 0.25);
			p[3] = make_pair(p[0].first + 0.25, p[0].second + 0.25);
			p[4] = make_pair(p[0].first + 0.25, p[0].second - 0.25);
			p[5] = p[1];

			for (int i = 1; i <= 4; i++) {
				if (Dot(dx[i - 1], dy[i - 1], realT.first - realS.first, realT.second - realS.second) <= 0)continue;
				if (SegmentCross(realS, realT, p[i], p[i + 1])) { Crossnum = i; break; }
			}

			int nx = S.first + dx[Crossnum - 1], ny = S.second + dy[Crossnum - 1];
			S = make_pair(nx, ny);
		}

		if (dis[Carry(goods)][desk_num][S.first][S.second] == 1e9 || map[S.first][S.second] == '#')
			return false;
	}
	return true;
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

	double startang = -Pi, endang = Pi, Delt = (endang - startang) / 90;
	double ansang = -1, ansdis = -1, maxdisdown = -1e9, disdown;
	double l, r, mid, maxlen, res;
	int Crossnum;
	pair<double, double> s, t, p[6];
	pair<int, int>S, T;

	//寻找所有角度上距离下降最多的方向
	while (startang < endang) {

		//寻找最长长度当前方向上的最长长度
		maxlen = -1; l = 0; r = 5;
		while (r - l >= 0.125) {
			mid = (l + r) / 2;
			if (accessjudge(desk_num, startang, mid, 0.02))maxlen = mid, l = mid;
			else r = mid;
		}
		if (maxlen == -1) {startang += Delt;continue;}


		s = make_pair(x, y); t = make_pair(x + maxlen * cos(startang), y + maxlen * sin(startang));
		S = math_n::ztoe(x, y); T = math_n::ztoe(x + maxlen * cos(startang), y + maxlen * sin(startang));
		Crossnum = -1; res = -1;

		while (S != T) {

			p[0] = math_n::etoz(S.first, S.second);
			p[1] = make_pair(p[0].first - 0.25, p[0].second - 0.25);
			p[2] = make_pair(p[0].first - 0.25, p[0].second + 0.25);
			p[3] = make_pair(p[0].first + 0.25, p[0].second + 0.25);
			p[4] = make_pair(p[0].first + 0.25, p[0].second - 0.25);
			p[5] = p[1];

			for (int i = 1; i <= 4; i++) {
				if (Dot(dx[i - 1], dy[i - 1], t.first - s.first, t.second - s.second) <= 0)continue;
				if (SegmentCross(s, t, p[i], p[i + 1])) { Crossnum = i; break; }
			}

			int nx = S.first + dx[Crossnum - 1], ny = S.second + dy[Crossnum - 1];

			if (dis[Carry(goods)][desk_num][S.first][S.second] <= dis[Carry(goods)][desk_num][nx][ny]) {
				res = Dist(CrossPoint(s, t, p[Crossnum], p[Crossnum + 1]), s) - 0.03;
				//这里有无更精准的移动方式？（增大参数或许可以在一定程度上避免小车过度行驶问题，一定不能设置为加上该参数！）※※※※※※※※
				break;
			}

			S = make_pair(nx, ny);
		}

		if (res == -1 || res > maxlen)res = maxlen;

		double realtx = x + res * cos(startang);
		double realty = y + res * sin(startang);

		pair<int, int>ss = math_n::ztoe(x, y), tt = math_n::ztoe(realtx, realty);
		pair<double, double>ssreal = math_n::etoz(ss.first, ss.second), ttreal = math_n::etoz(tt.first, tt.second);

		disdown = dis[Carry(goods)][desk_num][ss.first][ss.second] - dis[Carry(goods)][desk_num][tt.first][tt.second];

		if (disdown > maxdisdown || (fabs(disdown - maxdisdown) < eps && res < ansdis)) {
			ansang = startang;
			ansdis = res;
			maxdisdown = disdown;
		}

		startang += Delt;
	}

	pair<double, double>dest;
	pair<int, int>temp = math_n::ztoe(x, y);
	pair<double, double>nownum = make_pair(temp.first, temp.second);
	pair<double, double>obnum, gettonum, dvec;
	double mindis = dis[Carry(goods)][desk_num][temp.first][temp.second];

	//墙角情况特判（0.45/0.53）
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

	//防bug措施：防止既无墙角也无下降方向的可能性（选择上升方向，然后再尝试找下降方向）
	if (fabs(dvec.first) > eps || fabs(dvec.second) > eps) {
		UnitV(dvec);
		dest = Add(multi(dvec, 0.5), make_pair(x, y));
	}
	else dest = make_pair(x + ansdis * cos(ansang), y + ansdis * sin(ansang));

	if (mode == 0)	return mov(dest.first, dest.second, desk_num);
	else return dest;
}


//动态避障1（有足够避让空间的小车避障）
void Car::PreCalc(int Choosenum) {

	pair<double, double>Vecp;

	I = 0.5 * pow(GetR(goods), 4) * Pi * 20, B = 50.0 / I;

	Vecp = getVec(ang);
	vecv = CombineV(Vecp); v = CombineV(vx, vy);
	v1x = Vecp.first; v1y = Vecp.second;
	d = fabs(Cross(car[Choosenum].x - x, car[Choosenum].y - y, v1x, v1y)) / vecv;

	Vecp = getVec(car[Choosenum].ang);
	vecv2 = CombineV(Vecp); v2 = CombineV(car[Choosenum].vx, car[Choosenum].vy);
	v2x = Vecp.first; v2y = Vecp.second;


	SinAng = fabs(Cross(v2x, v2y, v1x, v1y)) / (vecv2 * vecv);
	CosAng = Dot(v1x, v1y, v2x, v2y) / (vecv2 * vecv);


	//ux为正表示同向，为负表示反向；uy为正表示靠近，为负表示远离
	ux = v2 * CosAng;
	uy = v2 * SinAng;
	vecux = vecv2 * CosAng;
	vecuy = vecv2 * SinAng;
	if (Sign(Cross(car[Choosenum].x - x, car[Choosenum].y - y, v1x, v1y)) * Sign(Cross(v1x, v1y, v2x, v2y)) <= 0)
		uy *= -1, vecuy *= -1;
}
int Car::ChooseCrashCar(int numID, int desk_num) {

	int Choosenum = -1, cat = 0;
	double minDis = AlertRange;

	for (int i = 0; i < 4; i++) {

		if (i == numID)continue;
		if (Dist(car[i].x, car[i].y, x, y) > AlertRange)continue;
		if (!ObCheck(x, y, car[i].x, car[i].y, desk_num, 0, 0))continue;

		PreCalc(i);

		//正向180度内是否有需要回避的小车（注意小车的半径）
		pair<double, double>decv = multi(make_pair(v1x, v1y), -GetR(goods));
		pair<double, double>newpos = Add(make_pair(x, y), decv);

		/*
		if (numID == 2 && i == 1) {
			output << "v1=" << v1x << " " << v1y << endl;
			output << "decv=" << decv.first << " " << decv.second << endl;
			output << "newpos=" << newpos.first << " " << newpos.second << endl;
			output << "vec=" << car[i].x - newpos.first << " " << car[i].y - newpos.second << endl;
			output << endl;
		}
		*/

		if (Dot(car[i].x - newpos.first, car[i].y - newpos.second, v1x, v1y) / (vecv * CombineV(car[i].x - newpos.first, car[i].y - newpos.second)) < 0)
			continue;

		//判定当前小车行驶方向直线是否经过对面小车，或对面小车在警戒时间内到达这条直线
		if ((d >= GetR(goods) + GetR(car[i].goods) + 0.5 && uy < eps) ||
			(uy >= eps && max(d - GetR(goods) - GetR(car[i].goods) - 0.5, 0.0) / uy > AlertTime))
			continue;

		//在同样的优先级中优先选取满足前几行条件且最近的小车
		//优先级1：选择停在避让点或正向对碰的小车
		if (ux < 0 || (car[i].FindAvoid && car[i].Reach)) {
			if (cat <= 0)cat = 1, Choosenum = i, minDis = Dist(car[i].x, car[i].y, x, y);
			else if (Dist(car[i].x, car[i].y, x, y) < minDis) {
				minDis = Dist(car[i].x, car[i].y, x, y);
				Choosenum = i;
			}
		}
		//优先级2：选择追及的小车
		else if (cat <= 0) {
			if (cat == 0)cat = -1, Choosenum = i, minDis = Dist(car[i].x, car[i].y, x, y);
			else if (Dist(car[i].x, car[i].y, x, y) < minDis) {
				minDis = Dist(car[i].x, car[i].y, x, y);
				Choosenum = i;
			}
		}
		continue;
	}
	if (cat == 0)return -1;
	else return Choosenum;
}
void Car::SetFforChase(int numID, int Choosenum, double& forwar, double& rot) {

	//小车追及相对速度较小，不需要原来那么大的警戒范围
	if (Dist(x, y, car[Choosenum].x, car[Choosenum].y) > AlertRange / 2)return;

	//补丁：两个小车均在彼此前方180度范围内
	if (Sign(Dot(car[Choosenum].x - x, car[Choosenum].y - y, v1x, v1y)) >= 0 && Sign(Dot(x - car[Choosenum].x, y - car[Choosenum].y, v2x, v2y)) >= 0) {
		double vcx, vcy, T, tx, ty, RestLen1, RestLen2;
		vcx = x + v1x - car[Choosenum].x - v2x;
		vcy = y + v1y - car[Choosenum].y - v2y;
		if (fabs(Cross(v1x, v1y, v2x, v2y)) < eps) {
			if (Dot(v1x, v1y, car[Choosenum].x - x, car[Choosenum].y - y) > 0)
				tx = car[Choosenum].x, ty = car[Choosenum].y;
			else tx = x, ty = y;
		}
		else {
			T = Cross(vcx, vcy, v2x, v2y) / Cross(v1x, v1y, v2x, v2y);
			tx = x + v1x - T * v1x;
			ty = y + v1y - T * v1y;
		}
		RestLen1 = Dist(car[Choosenum].x, car[Choosenum].y, tx, ty);
		RestLen2 = Dist(tx, ty, x, y);
		if (fabs(RestLen1 - RestLen2) < 1) {
			if (goods > car[Choosenum].goods || (goods == car[Choosenum].goods && numID > Choosenum))forwar = 6;
			else forwar = max(ux - 3, 0.0);
		}
		else if (RestLen1 > RestLen2)forwar = 6;
		else forwar = max(ux - 3, 0.0);
	}
	//一般情况下的追及问题
	else if (Sign(Dot(car[Choosenum].x - x, car[Choosenum].y - y, v1x, v1y)) > 0 && Sign(Dot(x - car[Choosenum].x, y - car[Choosenum].y, v2x, v2y)) < 0)
		forwar = max(ux - 3, 0.0);//追及问题可以调参（速度较小影响运送时间，速度较大影响运送碰撞）

}
void Car::SetRFforInactive(int numID, int Choosenum, double& forwar, double& rot) {

	if (d > GetR(goods) + GetR(car[Choosenum].goods) - 0.05 /* && ((uy > eps && (d - GetR(goods) - GetR(car[Choosenum].goods)) / uy >= CheckTime) || uy <= eps)*/)
		forwar = 6;
	else forwar = 6 * cos((1 - max(Dist(x, y, car[Choosenum].x, car[Choosenum].y) - GetR(goods) - GetR(car[Choosenum].goods + 0.5), 0.0) / (AlertRange - GetR(goods) - GetR(car[Choosenum].goods))) * (Pi / 2));

	//先判断当前小车直行至目标点是否会撞到完成避让的小车
	double d = PointToSegment(make_pair(car[Choosenum].x, car[Choosenum].y), make_pair(x, y), des[numID]);
	if (d > GetR(goods) + GetR(car[Choosenum].goods) + 0.03 || Dist(x, y, car[Choosenum].x, car[Choosenum].y) > GetR(goods) + GetR(car[Choosenum].goods) + 0.15)return;

	//定位完成避让小车两个两端可以通过的点
	pair<double, double>Vecp = make_pair(x - car[Choosenum].x, y - car[Choosenum].y);
	UnitV(Vecp);
	pair<double, double>vec1 = Rotate(Vecp, Pi / 2), vec2 = Rotate(Vecp, -Pi / 2);
	vec1 = multi(vec1, GetR(goods) + GetR(car[Choosenum].goods));
	vec2 = multi(vec2, GetR(goods) + GetR(car[Choosenum].goods));
	pair<double, double>newpo1 = Add(make_pair(car[Choosenum].x, car[Choosenum].y), vec1);
	pair<double, double>newpo2 = Add(make_pair(car[Choosenum].x, car[Choosenum].y), vec2);

	//checknewpo1表示当前小车需要前往右侧（rot=-Pi），checknewpo2表示当前小车需要前往左侧（rot=Pi）
	bool checknewpo1 = false, checknewpo2 = false;
	if (!car[Choosenum].Search(newpo1.first, newpo1.second, 51, GetR(goods)))
		checknewpo1 = true;
	if (!car[Choosenum].Search(newpo2.first, newpo2.second, 51, GetR(goods)))
		checknewpo2 = true;

	if (checknewpo1 && checknewpo2) {
		pair<int, int>xpo1 = math_n::ztoe(newpo1.first, newpo1.second);
		pair<int, int>xpo2 = math_n::ztoe(newpo2.first, newpo2.second);
		int firstnum = Findfirstnum(numID);
		if (dis[Carry(goods)][destination[firstnum]][xpo1.first][xpo1.second] < dis[Carry(goods)][destination[firstnum]][xpo2.first][xpo2.second])
			checknewpo2 = false;
		else checknewpo1 = false;
	}

	/*
	output << "newpo1=" << newpo1.first << " " << newpo1.second << " check=" << checknewpo1 << endl;
	output << "newpo2=" << newpo2.first << " " << newpo2.second << " check=" << checknewpo2 << endl;
	*/

	//前往右侧
	if (checknewpo1) {
		UnitV(vec1);
		newpo1 = Add(make_pair(x, y), vec1);
		rot = CalcRotate(newpo1.first, newpo1.second, 51, CalcAng(newpo1.first, newpo1.second));
	}
	//前往左侧
	else {
		UnitV(vec2);
		newpo2 = Add(make_pair(x, y), vec2);
		rot = CalcRotate(newpo2.first, newpo2.second, 51, CalcAng(newpo2.first, newpo2.second));
	}
}
void Car::SetRFforActive(int numID, int Choosenum, double& forwar, double& rot) {

	//为什么删掉后半截就不会正向回避损失速度?※※※※
	//减去的0.05是当小车挨在一起时很难做到d大于两车半径相加
	if (d > GetR(goods) + GetR(car[Choosenum].goods) - 0.05 /* && ((uy > eps && (d - GetR(goods) - GetR(car[Choosenum].goods)) / uy >= CheckTime) || uy <= eps)*/)
		forwar = 6;
	//forwar是否需要乘进速度夹角参数?※※※※
	else forwar = 6 * cos((1 - max(Dist(x, y, car[Choosenum].x, car[Choosenum].y) - GetR(goods) - GetR(car[Choosenum].goods), 0.0) / (AlertRange - GetR(goods) - GetR(car[Choosenum].goods))) * (Pi / 2));

	/*
	output << numID << " is in SetRforActive" << endl;
	output << endl;
	*/

	if ((FindAvoid && !car[Choosenum].FindAvoid) ||
		(!(FindAvoid ^ car[Choosenum].FindAvoid) && (goods > car[Choosenum].goods || (goods == car[Choosenum].goods && numID > Choosenum))))
		return;

	/*
	output << numID << " is avoiding " << Choosenum << endl;
	output << endl;
	*/

	pair<double, double>Vecp;
	double AddAng, PredAng1, PredAng2, PredAng3, newd1, newd2, newd3;
	double Ang1 = Cross(v1x, v1y, car[Choosenum].x - x, car[Choosenum].y - y) / (vecv * CombineV(car[Choosenum].x - x, car[Choosenum].y - y));

	//旋转方向调整
	if (uy > 0.2) {//加入uy与当前距离比较参数
		if (fabs(w) > eps) {
			AddAng = w * w / B * 0.5;
			PredAng1 = ang + Sign(w) * (AddAng + Pi / 15);
			PredAng2 = ang + Sign(w) * AddAng;
			PredAng3 = ang + Sign(w) * (AddAng - Pi / 15);
			AdjuAng(PredAng1); AdjuAng(PredAng2); AdjuAng(PredAng3);
			Vecp = getVec(PredAng1);
			newd1 = fabs(Cross(car[Choosenum].x - x, car[Choosenum].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
			Vecp = getVec(PredAng2);
			newd2 = fabs(Cross(car[Choosenum].x - x, car[Choosenum].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
			Vecp = getVec(PredAng3);
			newd3 = fabs(Cross(car[Choosenum].x - x, car[Choosenum].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
			if (newd3 > GetR(goods) + GetR(car[Choosenum].goods))rot = 0;
			else if (newd1 <= GetR(goods) + GetR(car[Choosenum].goods) && newd2 > GetR(goods) + GetR(car[Choosenum].goods))rot = 0;
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
			newd1 = fabs(Cross(car[Choosenum].x - x, car[Choosenum].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
			Vecp = getVec(PredAng2);
			newd2 = fabs(Cross(car[Choosenum].x - x, car[Choosenum].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
			Vecp = getVec(PredAng3);
			newd3 = fabs(Cross(car[Choosenum].x - x, car[Choosenum].y - y, Vecp.first, Vecp.second)) / CombineV(Vecp);
			if (newd3 > GetR(goods) + GetR(car[Choosenum].goods))rot = 0;
			else if (newd1 <= GetR(goods) + GetR(car[Choosenum].goods) && newd2 > GetR(goods) + GetR(car[Choosenum].goods))rot = 0;
			else rot = Sign(w) * Pi;
		}
		else if (Ang1 >= 0)
			rot = -Pi;
		else
			rot = Pi;
	}
}
void Car::SetRFforCrash(int numID, int Choosenum, double& forwar, double& rot) {

	if (car[Choosenum].FindAvoid && car[Choosenum].Reach)
		SetRFforInactive(numID, Choosenum, forwar, rot);
	else {
		PreCalc(Choosenum);
		SetRFforActive(numID, Choosenum, forwar, rot);
	}

}
void Car::CarCrashCheck(double& forwar, double& rot, int desk_num) {

	int numID = FindnumID(), Choosenum = ChooseCrashCar(numID, desk_num);
	if (Choosenum == -1)return;

	//另一小车与当前小车同向行驶且不为停在避让点的小车
	if (vecux >= 0.15 && !(car[Choosenum].FindAvoid && car[Choosenum].Reach)) {
		PreCalc(Choosenum);
		SetFforChase(numID, Choosenum, forwar, rot);
	}

	//另一小车与当前小车对向行驶或为停在避让点的小车
	else SetRFforCrash(numID, Choosenum, forwar, rot);
}


//动态避障2（无足够避让空间的小车避障）
bool Car::ChooseAvoider(int Cnum) {

	if (FindAvoid)return true;
	int numID = FindnumID();

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
bool Car::AvoidSituation1Check(double sx, double sy) {
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

	if (!tot) return false;
	return !SearchAccess(xid, make_pair(sx, sy), p1, p2, GetR(goods) + 2 * GetR(car[Avoidnum].goods));
}
void Car::Updatelas(pair<double, double> StaPo) {

	pair<double, double>lsta2 = lassta2, lsta3 = lassta3;

	pair<int, int>newnum = math_n::ztoe(StaPo.first, StaPo.second);
	pair<int, int>s2 = math_n::ztoe(lassta2.first, lassta2.second);
	pair<int, int>s3 = math_n::ztoe(lassta3.first, lassta3.second);

	if (fabs(newnum.first - s3.first) > eps || fabs(newnum.second - s3.second) > eps)
		lassta3 = StaPo;

	lassta2 = make_pair(x, y);
	pair<int, int>news2 = math_n::ztoe(lassta2.first, lassta2.second);

	if (fabs(s2.first - news2.first) > eps || fabs(s2.second - news2.second) > eps)
		lassta = lsta2;
}
pair<double, double> Car::Dynamic_Avoidance() {

	pair<int, int>xzb = math_n::ztoe(x, y);
	pair<double, double>szb = math_n::etoz(xzb.first, xzb.second);
	double sx = szb.first, sy = szb.second;

	int numID = FindnumID();
	int firstnum = Findfirstnum(numID);

	/*
	output << "numID=" << numID << endl;
	output << "firstnum=" << firstnum << endl;
	output << endl;
	*/

	pair<double, double>StaPo = Static_Avoidance(destination[firstnum], 1);

	Updatelas(StaPo);

	//判定路径描绘与小车朝向是否一致
	pair<double, double>VecA = Sub(xzb, math_n::ztoe(lassta.first, lassta.second));
	pair<double, double>VecA2 = Sub(xzb, math_n::ztoe(lassta2.first, lassta2.second));
	pair<double, double>VecA3 = Sub(math_n::ztoe(lassta3.first, lassta3.second), xzb);
	if (fabs(VecA3.first) < eps && fabs(VecA3.second) < eps)VecA3 = VecA;//到达工作台的路径描绘（修改！）※※※※※※※※※※※※※
	pair<double, double>dir = getVec(ang);
	if (Dot(VecA, VecA3) <= 0 || Dot(VecA, dir) <= 0 || Dot(VecA3, dir) <= 0)
		return make_pair(StaPo.first, StaPo.second);


	//与被回避小车的相对方向
	pair<double, double>cardir = make_pair(x - car[Avoidnum].x, y - car[Avoidnum].y);
	if (!ObCheck(x, y, car[Avoidnum].x, car[Avoidnum].y, 51, 0, 0))cardir = make_pair(0.0, 0.0);

	//output << "cardir=" << cardir.first << " " << cardir.second << endl << endl;


	//情形1：墙角或靠边（共八个方向）
	double startang = -Pi, endang = Pi, deltaang = Pi / 4;
	double maxlen, l, r, mid;
	pair<double, double>Vecp;

	while (startang < endang) {

		Vecp = getVec(startang);

		if ((Dot(Vecp, VecA) < 0 && fabs(Dot(Vecp, VecA)) > eps) || (Dot(Vecp, VecA2) < 0 && fabs(Dot(Vecp, VecA2)) > eps) ||
			(Dot(Vecp, VecA3) < 0 && fabs(Dot(Vecp, VecA3)) > eps) || (Dot(Vecp, cardir) < 0 && fabs(Dot(Vecp, cardir)) > eps)) {
			startang += deltaang;
			continue;
		}

		l = 0; r = 4; maxlen = -1;

		//二分寻找最长长度
		while (r - l >= 0.01) {
			mid = (l + r) / 2;
			double tx = sx + mid * cos(startang), ty = sy + mid * sin(startang);
			pair<int, int>tt = math_n::ztoe(tx, ty);
			if (tx < 0 || tx>50 || ty < 0 || ty>50) {r = mid;continue;}
			if (fabs(xzb.first - tt.first) < eps && fabs(xzb.second - tt.second) < eps) {l = mid;continue;}
			if (ObCheck(sx, sy, tx, ty, destination[firstnum], GetR(goods), 1))maxlen = mid, l = mid;
			else r = mid;
		}
		if (maxlen == -1) { startang += deltaang; continue; }


		//不能选择工作台做避让点，0.4为容忍参数
		if (Dist(sx + maxlen * cos(startang), sy + maxlen * sin(startang), desk[destination[firstnum]].x, desk[destination[firstnum]].y) < 0.4) {
			startang += deltaang;
			continue;
		}

		//判定该点是否可以作为避让点
		if (AvoidSituation1Check(sx + maxlen * cos(startang), sy + maxlen * sin(startang))) {
			setto = make_pair(sx + maxlen * cos(startang), sy + maxlen * sin(startang));
			FindAvoid = 2;
			break;
		}

		startang += deltaang;
	}

	/*
	output << "Calculate " << numID << " Dynamic Avoidance Situation 1 Done" << endl << endl;
	output << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl << endl;
	*/

	
	//情形2：长直通道（共四个方向）
	startang = -Pi; deltaang = Pi / 2;

	while (startang < endang) {

		Vecp = getVec(startang);

		if ((Dot(Vecp, VecA) < 0 && fabs(Dot(Vecp, VecA)) > eps) || (Dot(Vecp, VecA2) < 0 && fabs(Dot(Vecp, VecA2)) > eps) ||
			(Dot(Vecp, VecA3) < 0 && fabs(Dot(Vecp, VecA3)) > eps) || (Dot(Vecp, cardir) < 0 && fabs(Dot(Vecp, cardir)) > eps)) {
				startang += deltaang;
				continue;
		}

		l = 0; r = 50; maxlen = -1;

		//二分寻找最长长度
		while (r - l >= 0.01) {
			mid = (l + r) / 2;
			double tx = sx + mid * cos(startang), ty = sy + mid * sin(startang);
			pair<int, int>tt = math_n::ztoe(tx, ty);
			if (tx < 0 || tx>50 || ty < 0 || ty>50) {r = mid;continue;}
			if (fabs(xzb.first - tt.first) < eps && fabs(xzb.second - tt.second) < eps) {l = mid;continue;}
			if (ObCheck(sx, sy, tx, ty, destination[firstnum], GetR(goods), 1))maxlen = mid, l = mid;
			else r = mid;
		}
		if (maxlen == -1) { startang += deltaang; continue; }

		//output << "maxlen=" << maxlen << endl;

		//不能选择工作台做避让点，0.4为容忍参数
		if (Dist(sx + maxlen * cos(startang), sy + maxlen * sin(startang), desk[destination[firstnum]].x, desk[destination[firstnum]].y) < 0.4) {
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
			if (dis[Carry(car[firstnum].goods)][destination[firstnum]][S.first][S.second] <= dis[Carry(car[firstnum].goods)][destination[firstnum]][nx][ny]) {
				goodPoint = CrossPoint(realS, realT, p[Crossnum], p[Crossnum + 1]);
				break;
			}
			S = make_pair(nx, ny);
		}

		/*
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

	/*
	output << "Calculate " << numID << " Dynamic Avoidance Situation 2 Done" << endl << endl;
	output << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl << endl;
	*/


	//判定小车回避状态并返回对应目标点（已处理过，是可以不经过障碍物而到达的点）
	if (FindAvoid == 1)return make_pair(StaPo.first, StaPo.second);
	else {
		if ((fabs(x - setto.first) > 0.01 && fabs(y - setto.second) > 0.01)) {
			pair<double, double>vecx = make_pair(setto.first - x, 0), vecy = make_pair(0, setto.second - y);
			UnitV(vecx); UnitV(vecy); multi(vecx, 0.02); multi(vecy, 0.02);
			pair<double, double>po1 = Add(make_pair(x, y), vecx), po2 = Add(make_pair(x, y), vecy);
			bool check1 = !Search(po1.first, po1.second, 51, GetR(goods)), check2 = !Search(po2.first, po2.second, 51, GetR(goods));
			if (check1 && check2)return make_pair(setto.first, setto.second);
			if (check1)return make_pair(setto.first, y);
			if (check2)return make_pair(x, setto.second);
		}
		else return make_pair(setto.first, setto.second);
	}

	/*
	output << "Calculate " << numID << " Dynamic Avoidance Done" << endl << endl;
	output << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl << endl;
	*/

}


//移动决策输出
pair<double, double> Car::mov(double nx, double ny, int desk_num) {

	/*
	output << "numID=" << FindnumID() << endl;
	output << endl;
	*/

	//计算当前朝向与目标点的偏向角
	double DeltaAng = CalcAng(nx, ny);

	//计算角速度与速度的设定值
	double rot = CalcRotate(nx, ny, desk_num, DeltaAng), forwar = CalcForward(nx, ny, desk_num, DeltaAng);

	/*
	output << "Calcforwar=" << forwar << endl;
	output << "Calcrot=" << rot << endl;
	output << endl;
	*/

	//小车碰撞判定
	CarCrashCheck(forwar, rot, desk_num);

	/*
	output << "CarCrashforwar=" << forwar << endl;
	output << "CarCrashrot=" << rot << endl;
	output << endl;
	*/

	//边界碰撞判定
	double checkforwar = forwar;

	//可以考虑删除资本家系统观察表现是否更优（CalcForward内也有）⭐⭐⭐⭐⭐
	if (goods >= 4 || (fabs(desk[desk_num].x - nx) < eps && fabs(desk[desk_num].y - ny) < eps))
		MarginCheck(forwar, desk_num);

	if (fabs(checkforwar) > eps && fabs(forwar) < eps && fabs(w) < eps && fabs(rot) < eps)
		forwar = checkforwar;

	/*
	output << "Marginforwar=" << forwar << endl;
	output << "Marginrot=" << rot << endl;
	output << endl;
	*/

	/*
	output << FindnumID() << " Done" << endl;
	output << "-----------------------------------------" << endl;
	output << endl;
	*/

	return pair<double, double>(forwar, rot);
}
pair<double, double> Car::mov(int desk_num) {

	int numID = FindnumID();
	int firstnum = Findfirstnum(numID);

	if (des[numID].first == x && des[numID].second == y)
		return make_pair(0.0, 0.0);
	else return mov(des[numID].first, des[numID].second, destination[firstnum]);
}
void calc() {

	t = 0;

	CheckRunningCrash();
	//检查是否死机

	/*
	output << "CheckRunningCrash Done" << endl << endl;
	output << "------------------------------------------" << endl << endl;
	*/

	DynamicStatusUpdate();
	//所有小车动态回避状态更新
	//状态更新包括：1.正常流程下的状态更新；2.到达终点情况下的状态倒置
	//注意：各小车状态更新的顺序

	/*
	output << "DynamicStatusUpdate Done" << endl << endl;
	output << "-----------------------------------------" << endl << endl;
	*/

	CalculateTrueDestination();
	//在前面两个工作的基础上计算当前小车的目标点
	//目标点包括：1.静态避障下的目标点；2.动态避障时未找到避让点的前进方向（最先被避让的小车在静态避障下的目标点）；3.动态避障找到避让点时的前进方向

	/*
	output << "CalculateTrueDestination Done" << endl << endl;
	output << "-----------------------------------------" << endl << endl;
	*/

	IntoDynamicCheckAndUpdate();
	//检查未进入动态避障的小车是否可以进入动态避障状态，若可以进入则进入并更新信息

	/*
	output << "IntoDynamicCheckAndUpdate Done" << endl << endl;
	output << "-----------------------------------------" << endl << endl;
	*/

	/*
	for (int i = 0; i < 4; i++) {
		output << "numID=" << i << endl;
		output << "pos=" << car[i].x << " " << car[i].y << endl;
		output << "des=" << des[i].first << " " << des[i].second << endl;
		output << "FindAvoid=" << car[i].FindAvoid << endl;
		output << "Avoidnum=" << car[i].Avoidnum << endl;
		output << "setto=" << car[i].setto.first << " " << car[i].setto.second << endl;
		output << "Reach=" << car[i].Reach << endl;
		output << endl;
	}

	output << "------------------------------------" << endl;
	output << endl;
	*/

}
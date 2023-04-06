#pragma once


//几何/数学
pair<double, double> getVec(double);                                                                                   // 得到指向对应角度的单位向量
double Sign(double);                                                                                                   // 提取输入参数的符号
double Dot(double, double, double, double);                                                                            // 点积（点对形式）
double Dot(pair<double, double>, pair<double, double>);                                                                // 点积（向量形式）
double Cross(double, double, double, double);                                                                          // 叉积（点对形式）
double Cross(pair<double, double>, pair<double, double>);                                                              // 叉积（向量形式）
pair<double, double> Rotate(pair<double, double>, double);                                                             // 旋转向量
pair<double, double> multi(pair<double, double>, double);                                                              // 向量乘标量
pair<double, double> Add(pair<double, double>, pair<double, double>);                                                  // 向量加向量
pair<double, double> Sub(pair<double, double>, pair<double, double>);                                                  // 向量减向量
double Dist(double, double, double, double);                                                                           // 计算两个点之间的距离（double点对形式）
double Dist(pair<double, double>, pair<double, double>);                                                               // 计算两个点之间的距离（pair形式）
double PointToLine(pair<double, double>, pair<double, double>, pair<double, double>);                                  // 计算点到直线距离
double PointToSegment(pair<double, double>, pair<double, double>, pair<double, double>);                               // 计算点到线段距离
bool SegmentCross(pair<double, double>, pair<double, double>, pair<double, double>, pair<double, double>);             // 判断线段是否相交
pair<double,double> CrossPoint(pair<double, double>, pair<double, double>, pair<double, double>, pair<double, double>);// 直线求交点
void AdjuAng(double&);                                                                                                 // 调整角度范围使其落在-Pi到Pi之间
double CombineV(double, double);                                                                                       // 计算合速度（点对形式，非负）
double CombineV(pair<double, double>);                                                                                 // 计算合速度（向量形式，非负）
void UnitV(pair<double, double>&);                                                                                     // 单位化向量

struct Car
{
	int workbench, goods;                                           // 所处工作台编号，携带物品编号
	double timerate, hitrate;                                       // 时间价值系数，碰撞价值系数
	double vx, vy, w;                                               // 二维线速度向量，角速度
	double ang, x, y;                                               // 朝向角度，坐标
	int FindAvoid, Avoidnum;                                        // 是否处于正在寻找回避点的状态（1表示正在回避，2表示已经回避好，3表示被回避小车进入范围，4表示被回避小车离开范围），正在回避的小车编号
	bool Reach;                                                     // 小车是否抵达目标点
	int goodsrec;                                                   // 记录被回避小车的商品情况
	pair<double, double> setto;                                     // 小车在动态回避二阶段应去的点
	pair<double, double>lassta, lassta2, lassta3;                   // 在动态避障中上一次要去的点

	//小车
	int Carry(int);                                                           // 判断当前小车是否持有物品
	double GetR(int);                                                         // 根据是否持有物品返回当前半径
	double CalcAng(double, double);                                           // 计算当前朝向与目标点的偏向角
	double CalcRotate(double, double, int, double);                           // 根据偏向角计算角速度
	double CalcForward(double, double, double);                               // 根据偏向角计算前进速度
	bool ObCheck(double, double, double, double, int, double, bool);          // 碰撞检测（包含最小容忍宽度）
	void DFS(pair<int, int>, pair<double, double>, int, double);              // 搜索一定范围的格子并判断是否是障碍物
	bool Search(double, double, int, double);                                 // 判断一定范围内是否有障碍物（半径不宜过大）

	//避障
	void MarginCheck(double&, int);                                           // 地图边界避障
	void CarCrashCheck(double&, double&, int);                                // 仅小车避障

	bool ChooseAvoider(int);                                                  // 判断当前情况下是否应该由另一小车避让
	bool accessjudge(int, double, double, double);                            // 判断当前角度一定距离是否可以通行
	pair<double, double> Static_Avoidance(int, int);                          // 静态避障（仅考虑障碍物避障）
	void DFSAccess(pair<int, int>, pair<double, double>, int, int, double);   // 判断以某一点为圆心r为半径的1/2圆或1/4圆中有无障碍物
	bool SearchAccess(pair<int, int>, pair<double, double>, int, int, double);// 判断以某一点为圆心r为半径的1/2圆或1/4圆中有无障碍物
	bool AvoidCheck(double, double, double&);                                 // 判断走到某位置是否能实现对另一小车的避让
	pair<double, double> Dynamic_Avoidance(int);                              // 动态避障（小车+障碍物避障）

	//移动决策输出
	pair<double, double> mov(double, double, int);                            // 将小车向目标点移动（坐标形式，仅有小车避障）
	pair<double, double> mov(int);                                            // 将小车向目标点移动（目标点编号形式，加入障碍物避障）
};
extern Car car[5];

extern void calc();
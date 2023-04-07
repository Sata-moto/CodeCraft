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


//状态更新及不经过障碍物的路径目标点计算
bool CheckAvoidTree(int, int);                   // 检查小车避让树
void CheckRunningCrash();                        // 死机判断
void DynamicStatusUpdate();                      // 动态避让更新：处于动态避让中小车的状态转移
void CalculateTrueDestination();                 // 计算所有小车不经过障碍物的前进目标点
void IntoDynamicCheckAndUpdate();                // 动态避让更新：未处于动态避障中的小车尝试进入动态避障


struct Car
{
	int workbench, goods;                                           // 所处工作台编号，携带物品编号
	double timerate, hitrate;                                       // 时间价值系数，碰撞价值系数
	double vx, vy, w;                                               // 二维线速度向量，角速度
	double ang, x, y;                                               // 朝向角度，坐标
	double lasx, lasy;                                              // 记录前一帧的坐标以用于死机判断
	int FindAvoid, Avoidnum;                                        // 是否处于正在寻找回避点的状态（1表示正在回避，2表示已经回避好，3表示被回避小车进入范围，4表示被回避小车离开范围），正在回避的小车编号
	bool Reach;                                                     // 小车是否抵达目标点
	int goodsrec;                                                   // 记录被回避小车的商品情况
	pair<double, double> setto;                                     // 小车在动态回避二阶段应去的点
	pair<double, double> lassta, lassta2, lassta3;                  // 在动态避障中上一次要去的点

	//小车
	int Carry(int);                                                           // 判断当前小车是否持有物品
	double GetR(int);                                                         // 根据是否持有物品返回当前半径
	int FindnumID();                                                          // 返回当前小车的编号
	int Findfirstnum(int);                                                    // 返回当前小车所处回避树的根节点
	double CalcAng(double, double);                                           // 计算当前朝向与目标点的偏向角
	double CalcRotate(double, double, int, double);                           // 根据偏向角计算角速度
	double CalcForward(double, double, int, double);                          // 根据偏向角计算前进速度

	//静态避障（障碍物避障）
	void MarginCheck(double&, int);                                           // 地图边界减速
	void DFS(pair<int, int>, pair<double, double>, int, double);              // 搜索一定范围的格子并判断是否是障碍物
	bool Search(double, double, int, double);                                 // 判断一定范围内是否有障碍物（半径不宜过大）
	bool ObCheck(double, double, double, double, int, double, bool);          // 碰撞检测（包含最小容忍宽度参数）
	bool accessjudge(int, double, double, double);                            // 判断当前角度一定距离是否可以通行
	pair<double, double> Static_Avoidance(int, int);                          // 静态避障

	//动态避障（小车避障）
	bool ChooseAvoider(int);                                                  // 判断当前情况下是否应该由另一小车避让
	void DFSAccess(pair<int, int>, pair<double, double>, int, int, double);   // 搜索以某一点为圆心r为半径的1/2圆或1/4圆中有无障碍物
	bool SearchAccess(pair<int, int>, pair<double, double>, int, int, double);// 判断以某一点为圆心r为半径的1/2圆或1/4圆中有无障碍物
	bool AvoidSituation1Check(double, double);                                // 判断动态避障情形1下寻找的避让点是否合法
	void Updatelas(pair<double, double>);                                     // 更新三个lassta数据成员（用小车经过的点以及静态避障或未找到避障点的动态避障下将要去的点尝试描绘小车路径）
	void CarCrashCheck(double&, double&, int);                                // 动态避障：有足够避让空间
	pair<double, double> Dynamic_Avoidance();                                 // 动态避障：物足够避让空间

	//移动决策输出
	pair<double, double> mov(double, double, int);                            // 将小车向目标点移动（坐标形式，仅有小车避障）
	pair<double, double> mov(int);                                            // 将小车向目标点移动（目标点编号形式，加入障碍物避障）
};
extern Car car[5];

extern void calc();
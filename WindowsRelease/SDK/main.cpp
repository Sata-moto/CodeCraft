#include "Global.h"
#include "car.h"
#include "desk.h"

char map[N][N];						// 地图
Car car[5];
Desk desk[52];
int cnt_desk;						// 一共有多少工作台
int occupied[52][10];				// 工作台是否被占用
int occupied_goods[10];				// 某个物品是否在被生产

int money;							// 金钱数
int frame_number;					// 帧序号

int destination[5];                 // 小车在当前时间的目的地
queue <int > total_destination[5];  // 小车经过上次决策后产生的目的地组
int buy[5];                         // 1 为 buy,0 为 sel
queue <int > total_buy[5];          // 小车经过上次决策后产生的 buy 组
int check[5];						// 小车当前是否 check 一下是否有商品
queue <int > total_check[5];		// 小车总的 check 组

// Sel 是让小车去卖东西，Buy 是买
void Sel(int car_num, int desk_num, int Check = 0)
{
	total_destination[car_num].push(desk_num), total_buy[car_num].push(0), total_check[car_num].push(Check);
}
void Buy(int car_num, int desk_num, int Check = 0)
{
	total_destination[car_num].push(desk_num), total_buy[car_num].push(1), total_check[car_num].push(Check);
}

vector <int > available_desk[10];//各种工作台当前有哪些空闲的

double dis(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
}

double dddis(int desk1, int desk2)
{
	return dis(desk[desk1].x, desk[desk1].y, desk[desk2].x, desk[desk2].y);
}

double cddis(int car1, int desk1)
{
	return dis(car[car1].x, car[car1].y, desk[desk1].x, desk[desk1].y);
}

//决策参数列表
namespace parameter
{
	//fun1 - 根据当前某种物品的剩余量计算生产它的权重衰减
	double fun1(int remain)
	{
		return 1.0/(remain+1);
	}
	double fun2(bool output_is_ready, int output_is_doing)
	{
		if (output_is_ready) return 1.2;
		else if (output_is_doing > 500) return 1;
		else return 0.8 + output_is_doing / 1250.0;
	}
	double fun3(int current_frame)
	{
		return 1;
		//还没想好
	}
	double fun4(int current_frame, double distance)
	{
		if (current_frame > 8500)
			return 1.0 / distance;
		else return 1;
	}
	double fun5(bool is_7, bool is_empty, bool is_done, double is_doing)
	{
		if (!is_7) return 0.8;
		else if (!is_empty) return 0;
		else if (is_done) return 1.2;
		else if (is_doing > 1000) return 1;
		else if (is_doing) return 0.8 + is_doing / 2500.0;
		else return 1;
	}
	double fun6(int number_of_exists)
	{
		return 1 + number_of_exists / 10.0;
	}
}

int son[10][2];
double Earning[10] = { 0,3000,3200,3400,7100,7800,8300,29000 };

// 决策生产 4/5/6 中的谁
void make_decision(int car_num)
{
	// 贪心决策
	// 思路：1，2，3 种物品的生产视为不需要决策的，每个机器人独立决策当前
	// 生产 4/5/6，生产哪一个根据 生产利润/生产它需要的距离
	// * fun1(该种物品的场上剩余数目) * fun2（该种物品的目的工作
	// 台的产品格上是否有物品了，或者正在做）* fun3（时间选择系数）决定，
	// 选择权重大的那个，如果目的地工作台上已经有了物品，则将其卖出。
	// 卖出的地点是 7/9，权重是 出售它需要的距离  * fun4（当前时间）
	// * fun5（是否是 7 并且该格子空着并且有没有输出） * fun6（是 
	// 7 的话工作台上已经有了几种物品） 
	// 注意：送第二次原料时保证第一次生产完毕并拿走。

	for (int k = 1; k <= 9; k++)
		available_desk[k].clear();
	for (int k = 0; k < cnt_desk; k++)
		if (!occupied[k][0] || desk[k].type <= 3)
			available_desk[desk[k].type].push_back(k);
	//初始化工作台

	double max_earning = 0;
	int max_earning_desk_num = -1;
	int son_desk1 = -1, son_desk2 = -1;

	for (register int k = 4; k <= 6; k++)
	{
		for (int i = 0; i < (int)available_desk[k].size(); i++)
		{
			int now = available_desk[k][i];
			int son_Desk1 = -1, son_Desk2 = -1;
			double weight = 0, min_distance = 999999999;

			for (int p = 0; p < (int)available_desk[son[k][0]].size(); p++)
				for (int q = 0; q < (int)available_desk[son[k][1]].size(); q++)
				{
					int son_DESK1 = available_desk[son[k][0]][p];
					int son_DESK2 = available_desk[son[k][1]][q];
					double DIS = min(cddis(car_num, son_DESK1) + dddis(son_DESK1, now) + dddis(now, son_DESK2) * 2,
						cddis(car_num, son_DESK2) + dddis(son_DESK2, now) + dddis(now, son_DESK1) * 2);
					if (DIS < min_distance)
					{
						min_distance = DIS;
						son_Desk1 = son_DESK1;
						son_Desk2 = son_DESK2;
						if (cddis(car_num, son_Desk1) + dddis(son_Desk1, now) + dddis(now, son_Desk2) * 2 != DIS)
							swap(son_Desk1, son_Desk2);
					}
				}
			//计算最小距离

			if (son_Desk1 == -1) continue;

			int exist_count = occupied_goods[k];
			for (int j = 0; j < (int)available_desk[7].size(); j++)
				exist_count += desk[available_desk[7][j]].input_status[k];
			//计算当前物品场上存在的数量
			//occupied_good 直到物品被卖掉后才会减少，所以加上 7 上的就是场上的总量

			weight = (Earning[k] + Earning[son[k][0]] + Earning[son[k][1]]) / min_distance
				* parameter::fun1(exist_count) * parameter::fun2(desk[now].output_status, 500 - desk[now].remain_time)
				* parameter::fun3(frame_number);

			if (weight > max_earning)
			{
				max_earning = weight;
				max_earning_desk_num = now;
				son_desk1 = son_Desk1;
				son_desk2 = son_Desk2;
			}
		}
	}

	if (max_earning_desk_num != -1)
	{
		occupied[max_earning_desk_num][0] = 1;
		occupied_goods[desk[max_earning_desk_num].type]++;
		Buy(car_num, son_desk1);
		Sel(car_num, max_earning_desk_num);
		Buy(car_num, son_desk2);
		Sel(car_num, max_earning_desk_num, 1);
	}
}

// 决策 4/5/6 卖到哪去
void make_decision_to_7(int car_num, int goods)
{
	for (int k = 1; k <= 9; k++)
		available_desk[k].clear();
	for (int k = 0; k < cnt_desk; k++)
		if (!occupied[k][goods])
			available_desk[desk[k].type].push_back(k);
	//初始化工作台

	double max_earning = 0;
	int max_earning_desk_num = -1;
	for (register int k = 7; k <= 9; k++)
	{
		if (k == 8) continue;
		for (int i = 0; i < (int)available_desk[k].size(); i++)
		{
			int now = available_desk[k][i];
			double weight = 0;

			weight = Earning[goods] / cddis(car_num, now) * parameter::fun4(frame_number, cddis(car_num, now))
				* parameter::fun5(k == 7 ? 1 : 0, !desk[now].input_status[goods], desk[now].output_status, 500 - desk[now].remain_time)
				* parameter::fun6(desk[now].input_status[4] + desk[now].input_status[5] + desk[now].input_status[6]);

			if (weight > max_earning)
			{
				max_earning = weight;
				max_earning_desk_num = now;
			}
		}
	}

	if (max_earning_desk_num != -1)
	{
		Sel(car_num, max_earning_desk_num, 2);
		occupied[max_earning_desk_num][goods] = 1;
	}
}

// 决策把 7 号物品卖到哪去，直接就近就可以了
void make_decision_to_8(int car_num)
{
	for (int k = 1; k <= 9; k++)
		available_desk[k].clear();
	for (int k = 0; k < cnt_desk; k++)
		if (desk[k].remain_time <= 0)
			available_desk[desk[k].type].push_back(k);
	//初始化工作台

	double cloest_distance = 9999999999;
	int cloest_desk = -1;

	for (register int k = 8; k <= 9; k++)
	{
		for (int i = 0; i < (int)available_desk[k].size(); i++)
		{
			int now = available_desk[k][i];
			if (cddis(car_num, now) < cloest_distance)
			{
				cloest_desk = now;
				cloest_distance = cddis(car_num, now);
			}
		}
	}

	if (cloest_desk != -1)
		Sel(car_num, cloest_desk);
}

void init()
{
	son[4][0] = 1, son[4][1] = 2;
	son[5][0] = 1, son[5][1] = 3;
	son[6][0] = 2, son[6][1] = 3;
}

bool judge(int dest, int goods)
{
	if (goods == 4 && desk[dest].input_status[5] && desk[dest].input_status[6])
		return true;
	if (goods == 5 && desk[dest].input_status[4] && desk[dest].input_status[6])
		return true;
	if (goods == 6 && desk[dest].input_status[4] && desk[dest].input_status[5])
		return true;
	return false;
}

bool md[4] = { 0,0,0,0 };
bool md_7[4] = { 0,0,0,0 };
bool md_9[4] = { 0,0,0,0 };
bool wait[4] = { 0,0,0,0 };

int main()
{
	for (int k = 1; k <= 101; k++)
		scanf("%s", &map[k][1]);
	//地图没有用，101行是因为最后一行 OK

	init();

	printf("OK\n");
	fflush(stdout);
	//预处理

	while (scanf("%d %d", &frame_number, &money))
	{
		printf("%d\n", frame_number);
		scanf("%d", &cnt_desk);
		for (register int k = 0; k < cnt_desk; k++)
		{
			scanf("%d %lf %lf %d", &desk[k].type, &desk[k].x, &desk[k].y, &desk[k].remain_time);
			int input, input_cnt = 0, output;
			scanf("%d %d", &input, &output);
			for (int i = 0; i < 9; i++) desk[k].input_status[i] = 0;
			while (input)
			{
				if (input % 2 == 1)
					desk[k].input_status[input_cnt] = 1;
				input >>= 1, input_cnt++;
			}
			desk[k].output_status = output;
		} // 初始化工作台
		for (register int k = 0; k < 4; k++)
		{
			scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf",
				&car[k].workbench, &car[k].goods, &car[k].timerate, &car[k].hitrate,
				&car[k].w, &car[k].vx, &car[k].vy, &car[k].ang, &car[k].x, &car[k].y);
		} // 初始化小车
		char is_OK[10];
		scanf("%s", is_OK);
		// 初始化完毕

		if (frame_number == 1)
			for (int k = 0; k < 4; k++)
			{
				make_decision(k);
				if (!total_destination[k].empty())
				{
					destination[k] = total_destination[k].front();
					total_destination[k].pop();
					buy[k] = total_buy[k].front();
					total_buy[k].pop();
					check[k] = total_check[k].front();
					total_check[k].pop();
				}
			}
		//第一帧初始化决策

		for (int k = 0; k < 4; k++)
		{
			if (md_9[k])
			{
				make_decision_to_8(k);
				if (!total_destination[k].empty())
				{
					destination[k] = total_destination[k].front();
					total_destination[k].pop();
					buy[k] = total_buy[k].front();
					total_buy[k].pop();
					check[k] = total_check[k].front();
					total_check[k].pop();
				}
				md_9[k] = false;
			}
			if (md_7[k])
			{
				make_decision_to_7(k, car[k].goods);
				if (!total_destination[k].empty())
				{
					destination[k] = total_destination[k].front();
					total_destination[k].pop();
					buy[k] = total_buy[k].front();
					total_buy[k].pop();
					check[k] = total_check[k].front();
					total_check[k].pop();
				}
				md_7[k] = false;
			}
			if (md[k])
			{
				make_decision(k);
				if (!total_destination[k].empty())
				{
					destination[k] = total_destination[k].front();
					total_destination[k].pop();
					buy[k] = total_buy[k].front();
					total_buy[k].pop();
					check[k] = total_check[k].front();
					total_check[k].pop();
				}
				md[k] = false;
			}

			if (car[k].workbench == destination[k])
			{
				//对每个小车，如果已经到了目的地，并且可以做 buy/sell 指令，输出 buy,sell
				if (buy[k] && !wait[k])
					printf("buy %d\n", k);
				else if (!buy[k] && !wait[k])
				{
					printf("sell %d\n", k);
					if (car[k].goods >= 4 && car[k].goods <= 6)
						occupied_goods[car[k].goods]--;
				}
				if (check[k] == 1) //这组送往 4/5/6 的决策完成
				{
					//如果有输出了，就拿走
					if (desk[destination[k]].output_status)
					{
						occupied[destination[k]][0] = 0;
						printf("buy %d\n", k);
						md_7[k] = 1;
						wait[k] = 0;
					}
					//如果正在做并且输入填满了，要等待输出
					else if (desk[destination[k]].remain_time != -1 && desk[destination[k]].input_status != 0)
						wait[k] = 1;
					//否则就是刚填完一组，那么接触占用自己去做决策。
					else occupied[destination[k]][0] = 0;
				}
				else if (check[k] == 2)
				{
					occupied[destination[k]][car[k].goods] = 0;

					//如果有输出了，就拿走
					if (desk[destination[k]].output_status)
					{
						printf("buy %d\n", k);
						md_9[k] = 1;
						wait[k] = 0;
					}
					//如果正在做并且输入填满了，要等待输出
					else if (desk[destination[k]].remain_time != -1 && judge(destination[k], car[k].goods))
						wait[k] = 1;
					//否则就是刚填完一组，那么接触占用自己去做决策。
				}
				//如果当前买了之后有一个 check 请求，就会 check 当前工作台有没有 output
				//如果有 output，那就决策把这个送到哪去
				//如果因为挤占导致指令无法执行，就跳过决策并且等待。

				//从指令组中导入新的指令,如果指令组为空就新构指令
				if (!wait[k] && !md_7[k] && !md_9[k] && total_destination[k].empty())
					md[k] = true;
				if (!total_destination[k].empty())
				{
					destination[k] = total_destination[k].front();
					total_destination[k].pop();
					buy[k] = total_buy[k].front();
					total_buy[k].pop();
					check[k] = total_check[k].front();
					total_check[k].pop();
				}
			}

			pair<double, double> temp;
			temp = car[k].mov(desk[destination[k]].x, desk[destination[k]].y);
			printf("forward %d %lf\n", k, temp.first);
			printf("rotate %d %lf\n", k, temp.second);
			//每个小车朝当前的目的地前进
		}
		printf("OK\n");
		fflush(stdout);
	}

	return 0;
}
#include "Global.h"
#include "car.h"
#include "desk.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>

ofstream fdebug;

namespace seed_n
{
	int seed = 0;
	int seeds[5] = { 0,352354535,350895017,351758063,350804994 };
	int seed_MOD = 998244353;
}

namespace map_n
{
	int num_desk_7;
	int num_desk_9;
	int cnt_desk;						// 一共有多少工作台
	char map[N][N];						// 地图
}

namespace desk_n
{
	Desk desk[52];
	vector <int > available_desk[10]; //各种工作台当前有哪些空闲的
	void init_desk()
	{
		for (int k = 1; k <= 9; k++)
			available_desk[k].clear();
		for (int k = 0; k < map_n::cnt_desk; k++)
			available_desk[desk[k].type].push_back(k);
	}
}

namespace car_n
{
	bool available_car[4] = { 0,0,0,0 }; // stop_frame 前转到 stop_frame 后时，小车空闲标记。
}

namespace constant_n
{
	double Earning[10] = { 0,3000,3200,3400,7100,7800,8300,29000 };
	int money;							
	int frame_number;					
	int son[10][2];
	vector <int > father[10];			// son 和 father 描述了工作台之间的需求信息，在 init 中初始化
}

namespace wait_n
{
	bool is_waiting_for_7[10];							//某种物品有小车堵塞在了送到七号的过程种
	bool wait[4] = { 0,0,0,0 };							//当前小车送到了最后一个原料，但是当前产品还没有生产出来
	bool wait_until_spare_3[4] = { 0,0,0,0 };			//当前小车拿原料 1-3 时，发现 1-3 还没有生产好
	bool wait_until_spare_7[4] = { 0,0,0,0 };			//当前小车在第一层决策，准备拿取物品时发现没地方送导致等待
	bool wait_until_spare_sell[4] = { 0,0,0,0 };		//当前小车想要卖掉物品，但是没办法卖，导致等待（由优化引起）
	bool wait_stop_frame[4] = { 0,0,0,0 };				//stop_frame 后面的等待
}

namespace math_n
{
	double dis(double x1, double y1, double x2, double y2)
	{
		return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
	}
	double dddis(int desk1, int desk2)
	{
		return dis(desk_n::desk[desk1].x, desk_n::desk[desk1].y, desk_n::desk[desk2].x, desk_n::desk[desk2].y);
	}
	double cddis(int car1, int desk1)
	{
		return dis(car[car1].x, car[car1].y, desk_n::desk[desk1].x, desk_n::desk[desk1].y);
	}
}

namespace occupied_n
{
	int occupied[52][10];					// 工作台是否被占用
	int sol_occupied[52][10];				// 是否解决工作台的占用
	int occupied_goods[10];					// 场上某种物品的数量
	int occupied_stop_frame[52][10];		// stop_frame 后的 occupied
	int sol_occupied_stop_frame[52][10];    // stop_frame 后的 sol_occupied
	int ignore_occupied[52][10];			// ignore occupied 从而允许连续的运送
	bool sol_ignore_occupied[52][10];		// 清楚 ignore occupied
	int current_occupied[52][10];			// 单纯是 check_spare_7 用到的临时参数，用来标识是否占用了一个 7

	void reload_occupied()					// 每帧会 reload occupied，ocuupied 进行占用时可以本帧占用，但是解除时必须下一帧解除
	{
		for (int k = 0; k < map_n::cnt_desk; k++)
			for (int i = 0; i <= 9; i++)
				if (sol_occupied[k][i])
				{
					occupied[k][i] -= sol_occupied[k][i];
					occupied[k][i] = max(0, occupied[k][i]);
					sol_occupied[k][i] = 0;
				}
		for (int k = 0; k < map_n::cnt_desk; k++)
			for (int i = 0; i <= 9; i++)
				if (sol_occupied_stop_frame[k][i])
				{
					occupied_stop_frame[k][i] = 0;
					sol_occupied_stop_frame[k][i] = 0;
				}
		for (int k = 0; k < map_n::cnt_desk; k++)
			for (int i = 0; i <= 9; i++)
				if (sol_ignore_occupied[k][i])
				{
					ignore_occupied[k][i] = 0;
					sol_ignore_occupied[k][i] = 0;
				}
		memset(current_occupied, 0, sizeof(current_occupied));
	}
}

namespace assist_n
{
	bool full_6(int desk_num, int goods) //当前物品送到后，判断 4-6 号工作台是不是已经满了
	{
		if (desk_n::desk[desk_num].input_status[1] || desk_n::desk[desk_num].input_status[2] || desk_n::desk[desk_num].input_status[3])
			return true;
		return false;
	}

	bool full_7(int dest, int goods)	 //当前物品送到后，判断 7 号工作台是不是已经满了
	{
		if (goods == 4 && desk_n::desk[dest].input_status[5] && desk_n::desk[dest].input_status[6])
			return true;
		if (goods == 5 && desk_n::desk[dest].input_status[4] && desk_n::desk[dest].input_status[6])
			return true;
		if (goods == 6 && desk_n::desk[dest].input_status[4] && desk_n::desk[dest].input_status[5])
			return true;
		return false;
	}

	bool check_spare_7(int type)		//当前是否有空闲的 7 号工作台
	{
		//return true;//取消先等待再拿的决策
		if (map_n::num_desk_9 != 0) return true;
		desk_n::init_desk();

		for (int k = 0; k < (int)desk_n::available_desk[7].size(); k++)
		{
			int now = desk_n::available_desk[7][k];
			if (!occupied_n::occupied[now][type] && !desk_n::desk[now].input_status[type] && !occupied_n::current_occupied[now][type])
			{
				occupied_n::current_occupied[now][type] = 1;
				return true;
			}
		}
		return false;
	}
}

namespace command_n
{
	bool init_dc;
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

	void clear_decision(int k)
	{
		if (!total_destination[k].empty())
		{
			total_destination[k].pop();
			total_buy[k].pop();
			total_check[k].pop();
		}
	}

	bool md[4] = { 0,0,0,0 };
	bool md_7[4] = { 0,0,0,0 };
	bool md_9[4] = { 0,0,0,0 };
	bool md_stop_frame[4] = { 0,0,0,0 };
}

using namespace seed_n;				//种子包
using namespace map_n;				//地图包
using namespace constant_n;			//常量包
using namespace wait_n;				//等待包
using namespace occupied_n;			//占用包
using namespace command_n;			//指令包
using namespace desk_n;				//工作包	
using namespace math_n;				//数学包
using namespace car_n;				//小车包
using namespace assist_n;			//协助包

namespace parameter					//参数包
{
	int Stop_frame = 8700;
	double Time_Upscale = 1.2;
	double Earning_Upscale = 1.2;
	double End_frame = 8950;
	double fun1_desk_exist_num_downscale = 1;//1 - 未减权   0 - 所有工作台上的 4/5/6 产品不考虑
	double dis_pow_downscale = 5;

	void adjust_fun()
	{
		if (num_desk_7 != 0)
		{
			Earning[1] = Earning[3] = Earning[2];
			Earning[4] = Earning[6] = Earning[5];
		}

		if (seed == seeds[1])
		{

		}
		else if (seed == seeds[2])
		{

		}
		else if (seed == seeds[3])
		{

		}
		else if (seed == seeds[4])
		{

		}
	}

	//fun1 - 根据当前某种物品的剩余量计算生产它的权重衰减
	double fun1(double remain, int type = 0)
	{
		if (num_desk_7 == 0) return 1;
		if (is_waiting_for_7[type]) return 0;
		//if (seed == seeds[2]) return 1.0 / (remain + 1);
		else return pow(2.718, -remain);
	}
	double fun2(bool output_is_ready, int output_is_doing, bool is_begin_now)
	{
		if (seed == seeds[1])
		{
			if (output_is_ready) return 1.5;
			else if (output_is_doing > 500) return 0.8;
			else return 1.1 + output_is_doing / 1250.0;
		}
		if (output_is_ready) return 1.2;
		else if (output_is_doing > 500) return 1;
		else if (is_begin_now) return max(1.0, 0.8 + output_is_doing / 1250.0);
		else return 0.8 + output_is_doing / 1250.0;
	}
	double fun3(bool is_begin)
	{
		if (is_begin) return 1;
		else return 1.3;
	}
	double fun4(int current_frame, double distance, bool is_7, bool is_done)
	{
		return 1;// fun4 不起作用
		if (current_frame > 8500)
			return 1.0 / distance;
		else return 1;
	}
	double fun5(bool is_7, bool is_empty, bool is_done, double is_doing, int desk_num, int goods)
	{
		if (!is_7) return 0.5;
		else if (!is_empty) return -0.01;
		else if (is_done) return 1.5;
		else if (is_doing > 1000) return 1.3;
		else if (is_doing && !full_7(desk_num, goods)) return max(1.0, 0.8 + is_doing / 1250.0);
		else if (is_doing) return 0.8 + is_doing / 1250.0;
		else return 1;
	}
	double fun6(int desk_num, int number_of_exists)
	{
		if (desk[desk_num].type != 7) return 1;
		return 1 + (number_of_exists + occupied[desk_num][4] + occupied[desk_num][5] + occupied[desk_num][6]) / 5.0;
	}
}		

void init()
{
	son[4][0] = 1, son[4][1] = 2;
	son[5][0] = 1, son[5][1] = 3;
	son[6][0] = 2, son[6][1] = 3;

	for (int k = 1; k <= 100; k++)
		for (int i = 1; i <= 100; i++)
			seed += (k * 100 + i) * map[k][i], seed %= seed_MOD;

	for (int k = 1; k <= 7; k++) father[k].push_back(9);
	for (int k = 4; k <= 6; k++) father[k].push_back(7);
	father[1].push_back(4), father[1].push_back(5);
	father[2].push_back(4), father[2].push_back(6);
	father[3].push_back(5), father[3].push_back(6);
	father[7].push_back(8);

	parameter::adjust_fun();
}

void make_decision(int car_num)
{
	init_desk();

	double max_earning = 0;
	int max_earning_desk_num = -1;
	int son_desk;
	bool is_begin = false;

	for (register int k = 4; k <= 6; k++)
	{
		for (int i = 0; i < (int)available_desk[k].size(); i++)
		{
			int now = available_desk[k][i];
			int son_Desk = -1;
			bool is_begin_now = 0;
			double weight = 0, min_distance = 999999999;

			bool cal_son1 = false, cal_son2 = false;
			if (occupied[now][0]) continue;
			if (!desk[now].input_status[son[k][0]] && !occupied[now][son[k][0]]
				&& !desk[now].input_status[son[k][1]] && !occupied[now][son[k][1]])
			{
				is_begin_now = 1;
				cal_son1 = cal_son2 = 1;
			}
			else if (!desk[now].input_status[son[k][0]] && !occupied[now][son[k][0]])
				cal_son1 = 1;
			else if (!desk[now].input_status[son[k][1]] && !occupied[now][son[k][1]])
				cal_son2 = 1;
			else if (!desk[now].input_status[son[k][0]] && ignore_occupied[now][son[k][0]] == -1)
				is_begin_now = 1, cal_son1 = 1;
			else if (!desk[now].input_status[son[k][1]] && ignore_occupied[now][son[k][1]] == -1)
				is_begin_now = 1, cal_son2 = 1;
			else continue;

			if (cal_son1)
				for (int p = 0; p < (int)available_desk[son[k][0]].size(); p++)
				{
					int son_DESK = available_desk[son[k][0]][p];
					double DIS = cddis(car_num, son_DESK) + dddis(son_DESK, now);
					if (DIS < min_distance)
						min_distance = DIS, son_Desk = son_DESK;
				}
			if (cal_son2)
				for (int q = 0; q < (int)available_desk[son[k][1]].size(); q++)
				{
					int son_DESK = available_desk[son[k][1]][q];
					double DIS = cddis(car_num, son_DESK) + dddis(son_DESK, now);
					if (DIS < min_distance)
						min_distance = DIS, son_Desk = son_DESK;
				}
			//计算最小距离

			if (son_Desk == -1) continue;

			double exist_count = occupied_goods[k];
			for (int j = 0; j < (int)available_desk[7].size(); j++)
				exist_count += desk[available_desk[7][j]].input_status[k];
			for (int j = 0; j < (int)available_desk[k].size(); j++)
				exist_count -= desk[available_desk[k][j]].output_status * (1 - parameter::fun1_desk_exist_num_downscale);
			if (!is_begin_now) exist_count = 0;

			//计算当前物品场上存在的数量
			//occupied_good 直到物品被卖掉后才会减少，所以加上 7 上的就是场上的总量

			weight = Earning[k] / min_distance
				* parameter::fun1(exist_count, k) * parameter::fun2(desk[now].output_status, 500 - desk[now].remain_time, is_begin_now)
				* parameter::fun3(is_begin_now);

			if (weight > max_earning)
			{
				max_earning = weight;
				max_earning_desk_num = now;
				son_desk = son_Desk;
				is_begin = is_begin_now;
			}
		}
	}

	if (max_earning_desk_num != -1)
	{
		occupied[max_earning_desk_num][desk[son_desk].type]++;
		if (is_begin)
		{
			occupied_goods[desk[max_earning_desk_num].type]++;
			if (ignore_occupied[max_earning_desk_num][desk[son_desk].type] == -1)
				ignore_occupied[max_earning_desk_num][desk[son_desk].type] = 1;
		}
		else ignore_occupied[max_earning_desk_num][desk[son_desk].type] = -1;
		Buy(car_num, son_desk);
		Sel(car_num, max_earning_desk_num, 1);
	}
}

void make_decision_to_7(int car_num, int goods)
{
	init_desk();

	double max_earning = 0.000000000001;
	int max_earning_desk_num = -1;
	for (register int k = 7; k <= 9; k++)
	{
		if (k == 8) continue;
		for (int i = 0; i < (int)available_desk[k].size(); i++)
		{
			int now = available_desk[k][i];
			double weight = 0;

			weight = Earning[goods] / pow(cddis(car_num, now), 1 / parameter::dis_pow_downscale) * parameter::fun4(frame_number, cddis(car_num, now), k == 7 ? 1 : 0, desk[now].output_status)
				* parameter::fun5(k == 7 ? 1 : 0, !desk[now].input_status[goods], desk[now].output_status, 500 - desk[now].remain_time, now, goods)
				* parameter::fun6(now, desk[now].input_status[4] + desk[now].input_status[5] + desk[now].input_status[6]);

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
		if (desk[max_earning_desk_num].type == 7)
			occupied[max_earning_desk_num][goods] = 1;
	}
}

void make_decision_to_8(int car_num)
{
	init_desk();

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

void decision_before_stop_frame(int k)
{
	if (frame_number >= parameter::Stop_frame)
	{
		if (wait_until_spare_3[k] || wait[k] || wait_until_spare_7[k])
		{
			available_car[k] = 1;
			clear_decision(k);
			return;
		}
	}

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
			md_7[k] = false;
			wait[k] = 0;
		}
		else check[k] = -1, wait[k] = 1;
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
		else wait[k] = 1;
		md[k] = false;
	}

	if (car[k].workbench == destination[k])
	{
		//对每个小车，如果已经到了目的地，并且可以做 buy/sell 指令，输出 buy,sell
		if (buy[k] && !wait[k] && !wait_until_spare_7[k])
		{
			printf("buy %d\n", k);
			if (wait_until_spare_3[k] && desk[destination[k]].output_status)
				wait_until_spare_3[k] = 0;
		}
		if (!buy[k] && !wait[k] && !wait_until_spare_7[k])
		{
			if (desk[destination[k]].input_status[car[k].goods])
				wait_until_spare_sell[k] = 1;
			else
			{
				printf("sell %d\n", k);
				if (car[k].goods >= 4 && car[k].goods <= 6)
					occupied_goods[car[k].goods]--;
				sol_ignore_occupied[destination[k]][car[k].goods] = 1;
				if (wait_until_spare_sell[k])
				{
					md[k] = 1;
					wait_until_spare_sell[k] = 0;
					return;
				}

				if (frame_number >= parameter::Stop_frame)
				{
					available_car[k] = 1;
					clear_decision(k);
					return;
				}
			}

			//if (occupied_goods[car[k].goods] < 0)
			//	exit(-1);
			//错误跳出点 1
		}
		if (desk[destination[k]].type <= 3 && !desk[destination[k]].output_status)
			wait_until_spare_3[k] = 1;
		if (check[k] == 1 && !wait_until_spare_sell[k]) //这组送往 4/5/6 的任务完成
		{
			if (car[k].goods != 0 && !wait[k] && !wait_until_spare_7[k])
				sol_occupied[destination[k]][car[k].goods]++;
			//如果有输出了，就拿走
			if (desk[destination[k]].output_status && frame_number <= parameter::Stop_frame)
			{
				if (check_spare_7(desk[destination[k]].type))
				{
					printf("buy %d\n", k);
					md_7[k] = 1;
					wait[k] = 0;
					wait_until_spare_7[k] = 0;
					is_waiting_for_7[desk[destination[k]].type] = 0;
					sol_occupied[destination[k]][0] = 1;
				}
				else if (full_6(destination[k], car[k].goods))
				{
					if (!wait_until_spare_7[k])
					{
						wait_until_spare_7[k] = 1;
						occupied[destination[k]][0] = 1;
						is_waiting_for_7[desk[destination[k]].type] = 0;
					}
				}
			}
			//如果正在做并且输入填满了，要等待输出
			else if (desk[destination[k]].remain_time != -1 && full_6(destination[k], car[k].goods))
				wait[k] = 1;
			//否则就是刚填完一组，那么接触占用自己去做决策。
		}
		else if (check[k] == 2 && !wait_until_spare_sell[k])
		{
			sol_occupied[destination[k]][car[k].goods] = 1;
			//如果有输出了，就拿走
			if (desk[destination[k]].output_status)
			{
				printf("buy %d\n", k);
				md_9[k] = 1;
				wait[k] = 0;
			}
			//如果正在做并且输入填满了，要等待输出
			else if (desk[destination[k]].remain_time != -1 && full_7(destination[k], car[k].goods))
				wait[k] = 1;
			//否则就是刚填完一组，那么接触占用自己去做决策。
		}
		//如果当前买了之后有一个 check 请求，就会 check 当前工作台有没有 output
		//如果有 output，那就决策把这个送到哪去
		//如果因为挤占导致指令无法执行，就跳过决策并且等待。

		//从指令组中导入新的指令,如果指令组为空就新构指令
		if (!wait_until_spare_sell[k] && !wait_until_spare_3[k] && !wait_until_spare_7[k] && !wait[k] && !md_7[k] && !md_9[k] && total_destination[k].empty())
			md[k] = true;
		if (!wait_until_spare_sell[k] && !wait_until_spare_3[k] && !total_destination[k].empty())
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

void make_decision_stop_frame(int car_num)
{
	init_desk();

	double max_weight = -1;
	int buy_desk = -1, sell_desk = -1;

	for (int k = 1; k <= 7; k++)
	{
		for (int i = 0; i < (int)available_desk[k].size(); i++)
		{
			int now = available_desk[k][i];
			if (!desk[now].output_status || occupied_stop_frame[now][0])
				continue;
			for (int j = 0; j < (int)father[k].size(); j++)
			{
				for (int t = 0; t < (int)available_desk[father[k][j]].size(); t++)
				{
					int to = available_desk[father[k][j]][t];
					if (desk[to].input_status[k] || occupied[to][k] || occupied[to][0])
						continue;
					if (father[k][j] <= 7 && occupied_stop_frame[to][k])
						continue;

					double dis = cddis(car_num, now) + dddis(now, to);
					if (dis / 6 * parameter::Time_Upscale * 50 + frame_number > parameter::End_frame)
						continue;
					double weight = pow(Earning[k], parameter::Earning_Upscale) / dis;
					if (weight > max_weight)
					{
						max_weight = weight;
						buy_desk = now;
						sell_desk = to;
					}
				}
			}
		}
	}

	if (buy_desk != -1)
	{
		Buy(car_num, buy_desk);
		Sel(car_num, sell_desk);
		occupied_stop_frame[buy_desk][0] = 1;
		occupied_stop_frame[sell_desk][desk[buy_desk].type] = 1;
	}
}

void decision_after_stop_frame(int k)
{
	if (md_stop_frame[k])
	{
		make_decision_stop_frame(k);
		if (!total_destination[k].empty())
		{
			destination[k] = total_destination[k].front();
			total_destination[k].pop();
			buy[k] = total_buy[k].front();
			total_buy[k].pop();
			check[k] = total_check[k].front();
			total_check[k].pop();
		}
		else
			destination[k] = cnt_desk;
		md_stop_frame[k] = false;
	}

	if (car[k].workbench == destination[k])
	{
		//对每个小车，如果已经到了目的地，并且可以做 buy/sell 指令，输出 buy,sell
		if (buy[k])
		{
			if (desk[destination[k]].output_status)
			{
				printf("buy %d\n", k);
				sol_occupied_stop_frame[destination[k]][0] = 1;
				wait_stop_frame[k] = 0;
			}
			else wait_stop_frame[k] = 1;
		}
		else if (car[k].goods)
		{
			printf("sell %d\n", k);
			sol_occupied_stop_frame[destination[k]][car[k].goods] = 1;
		}

		if (!wait_stop_frame[k] && total_destination[k].empty())
			md_stop_frame[k] = true;
		if (!wait_stop_frame[k] && !total_destination[k].empty())
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

void make_decision_without_7(int car_num)
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
					double DIS1 = (cddis(car_num, son_DESK1) + dddis(son_DESK1, now) + dddis(now, son_DESK2) * 2);
					double DIS2 = (cddis(car_num, son_DESK2) + dddis(son_DESK2, now) + dddis(now, son_DESK1) * 2);
					double DIS = min(DIS1, DIS2);
					if (DIS < min_distance)
					{
						min_distance = DIS;
						son_Desk1 = son_DESK1;
						son_Desk2 = son_DESK2;
						if (DIS != DIS1)
							swap(son_Desk1, son_Desk2);
					}
				}
			//计算最小距离

			if (son_Desk1 == -1) continue;

			double exist_count = occupied_goods[k];
			for (int j = 0; j < (int)available_desk[7].size(); j++)
				exist_count += desk[available_desk[7][j]].input_status[k];
			for (int j = 0; j < (int)available_desk[k].size(); j++)
				exist_count -= desk[available_desk[k][j]].output_status * (1 - parameter::fun1_desk_exist_num_downscale);

			//计算当前物品场上存在的数量
			//occupied_good 直到物品被卖掉后才会减少，所以加上 7 上的就是场上的总量

			weight = (Earning[k] + Earning[son[k][0]] + Earning[son[k][1]]) / min_distance
				* parameter::fun1(exist_count) * parameter::fun2(desk[now].output_status, 500 - desk[now].remain_time, 0);

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

void decision_before_stop_frame_without_7(int k)
{
	if (frame_number >= parameter::Stop_frame)
	{
		if (wait_until_spare_3[k] || wait[k] || wait_until_spare_7[k])
		{
			available_car[k] = 1;
			clear_decision(k);
			return;
		}
	}

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
			md_7[k] = false;
			wait[k] = 0;
		}
		else check[k] = -1, wait[k] = 1;
	}
	if (md[k])
	{
		make_decision_without_7(k);
		if (!total_destination[k].empty())
		{
			destination[k] = total_destination[k].front();
			total_destination[k].pop();
			buy[k] = total_buy[k].front();
			total_buy[k].pop();
			check[k] = total_check[k].front();
			total_check[k].pop();
		}
		else wait[k] = 1;
		md[k] = false;
	}

	if (car[k].workbench == destination[k])
	{
		//对每个小车，如果已经到了目的地，并且可以做 buy/sell 指令，输出 buy,sell
		if (buy[k] && !wait[k] && !wait_until_spare_7[k])
		{
			printf("buy %d\n", k);
			if (wait_until_spare_3[k] && desk[destination[k]].output_status)
				wait_until_spare_3[k] = 0;
		}
		if (!buy[k] && !wait[k] && !wait_until_spare_7[k])
		{
			printf("sell %d\n", k);
			if (car[k].goods >= 4 && car[k].goods <= 6)
				occupied_goods[car[k].goods]--;
			if (occupied_goods[car[k].goods] < 0)
				exit(-1);
			//错误跳出点 1
		}
		if (desk[destination[k]].type <= 3 && !desk[destination[k]].output_status)
			wait_until_spare_3[k] = 1;
		if (check[k] == 1) //这组送往 4/5/6 的决策完成
		{
			//如果有输出了，就拿走
			if (desk[destination[k]].output_status && frame_number <= parameter::Stop_frame)
			{
				if (check_spare_7(desk[destination[k]].type))
				{
					sol_occupied[destination[k]][0] = 1;
					printf("buy %d\n", k);
					md_7[k] = 1;
					wait[k] = 0;
					wait_until_spare_7[k] = 0;
				}
				else wait_until_spare_7[k] = 1;
			}
			//如果正在做并且输入填满了，要等待输出
			else if (desk[destination[k]].remain_time != -1 && desk[destination[k]].input_status != 0)
				wait[k] = 1;
			//否则就是刚填完一组，那么接触占用自己去做决策。
			else sol_occupied[destination[k]][0] = 1;
		}
		else if (check[k] == 2)
		{
			sol_occupied[destination[k]][car[k].goods] = 1;
			//如果有输出了，就拿走
			if (desk[destination[k]].output_status)
			{
				printf("buy %d\n", k);
				md_9[k] = 1;
				wait[k] = 0;
			}
			//如果正在做并且输入填满了，要等待输出
			else if (desk[destination[k]].remain_time != -1 && full_7(destination[k], car[k].goods))
				wait[k] = 1;
			//否则就是刚填完一组，那么接触占用自己去做决策。
		}
		//如果当前买了之后有一个 check 请求，就会 check 当前工作台有没有 output
		//如果有 output，那就决策把这个送到哪去
		//如果因为挤占导致指令无法执行，就跳过决策并且等待。

		if (!buy[k])
		{
			if (frame_number >= parameter::Stop_frame)
			{
				available_car[k] = 1;
				clear_decision(k);
				return;
			}
		}

		//从指令组中导入新的指令,如果指令组为空就新构指令
		if (!wait_until_spare_3[k] && !wait_until_spare_7[k] && !wait[k] && !md_7[k] && !md_9[k] && total_destination[k].empty())
			md[k] = true;
		if (!wait_until_spare_3[k] && !total_destination[k].empty())
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

void DEBUG()
{
	fdebug << "time = " << frame_number << endl;
	for (int k = 0; k < cnt_desk; k++)
	{
		for (int i = 0; i <= 7; i++)
			fdebug << occupied[k][i] << ' ';
		fdebug << endl;
	}
	fdebug << endl << endl;
}

int main()
{
	//fdebug.open("data.txt");

	for (int k = 1; k <= 101; k++)
		scanf("%s", &map[k][1]);
	for (int k = 1; k <= 100; k++)
		for (int i = 1; i <= 100; i++)
			if (map[k][i] == '7')
				num_desk_7++;
			else if (map[k][i] == '9')
				num_desk_9++;

	init();

	printf("OK\n");
	fflush(stdout);
	//预处理

	while (scanf("%d %d", &frame_number, &money))
	{
		//if (frame_number % 100 == 0)
		//	DEBUG();

		reload_occupied();
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

		if (!init_dc)
		{
			for (int k = 0; k < 4; k++)
			{
				if (num_desk_7)
					make_decision(k);
				else
					make_decision_without_7(k);
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
			init_dc = 1;
		}
		//第一帧初始化决策

		for (int k = 0; k < 4; k++)
			if (!available_car[k])
				if (num_desk_7)
					decision_before_stop_frame(k);
				else
					decision_before_stop_frame_without_7(k);
			else
				decision_after_stop_frame(k);

		printf("OK\n");
		fflush(stdout);
	}

	//fdebug.close();

	return 0;
}
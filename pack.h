#pragma once

#include "Global.h"
#include "desk.h"
#include "car.h"

struct seed_c
{
	int seed = 0;
	int seeds[5] = { 0,352354535,350895017,351758063,350804994 };
	int seed_MOD = 998244353;
}seed_class;

struct desk_c
{
	int cnt_desk;
	Desk desk[52];
	bool is_corner[52];
	vector <int > total_desk[10];

	void init_desk();
}desk_class;

struct map_c
{
	int num_desk_7;
	int num_desk_9;
	char map[N][N];							// 地图
	double dis1[52][N][N];					// 各个工作台到地图上某个点的最小距离（小车上没有物品）
	double dis2[52][N][N];					// 各个工作台到地图上某个点的最小距离（小车上有物品）

	void dij(int desk_num);
	void get_dis();
}map_class;

struct car_c
{
	bool available_car[4] = { 0,0,0,0 }; // stop_frame 前转到 stop_frame 后时，小车空闲标记。
}car_class;

struct constant_c
{
	double Earning[10] = { 0,3000,3200,3400,7100,7800,8300,29000 };
	int money;
	int frame_number;
	int son[10][2];
	vector <int > father[10];			// son 和 father 描述了工作台之间的需求信息，在 init 中初始化
}constant_class;

struct wait_c
{
	bool is_waiting_for_7[10];							//某种物品有小车堵塞在了送到七号的过程种
	bool wait[4] = { 0,0,0,0 };							//当前小车送到了最后一个原料，但是当前产品还没有生产出来
	bool wait_until_spare_3[4] = { 0,0,0,0 };			//当前小车拿原料 1-3 时，发现 1-3 还没有生产好
	bool wait_until_spare_7[4] = { 0,0,0,0 };			//当前小车在第一层决策，准备拿取物品时发现没地方送导致等待
	bool wait_until_spare_sell[4] = { 0,0,0,0 };		//当前小车想要卖掉物品，但是没办法卖，导致等待（由优化引起）
	bool wait_stop_frame[4] = { 0,0,0,0 };				//stop_frame 后面的等待
}wait_class;

struct math_c
{
	double dddis1(int desk1, int desk2);
	double cddis1(int car1, int desk1);
	double dddis2(int desk1, int desk2);
	double cddis2(int car1, int desk1);
}math_class;

struct occupied_c
{
	int occupied[52][10];					// 工作台是否被占用
	int sol_occupied[52][10];				// 是否解决工作台的占用
	int occupied_goods[10];					// 场上某种物品的数量
	int occupied_stop_frame[52][10];		// stop_frame 后的 occupied
	int sol_occupied_stop_frame[52][10];    // stop_frame 后的 sol_occupied
	int ignore_occupied[52][10];			// ignore occupied 从而允许连续的运送
	bool sol_ignore_occupied[52][10];		// 清楚 ignore occupied
	int current_occupied[52][10];			// 单纯是 check_spare_7 用到的临时参数，用来标识是否占用了一个 7

	void reload_occupied();					// 每帧会 reload occupied，ocuupied 进行占用时可以本帧占用，但是解除时必须下一帧解除
}occupied_class;

struct assist_c
{
	bool full_6(int desk_num, int goods); //当前物品送到后，判断 4-6 号工作台是不是已经满了
	bool full_7(int dest, int goods);	 //当前物品送到后，判断 7 号工作台是不是已经满了
	bool check_spare_7(int type);		//当前是否有空闲的 7 号工作台
}assist_class;

struct command_c
{
	bool init_dc;
	int destination[5];                 // 小车在当前时间的目的地
	queue <int > total_destination[5];  // 小车经过上次决策后产生的目的地组
	int buy[5];                         // 1 为 buy,0 为 sel
	queue <int > total_buy[5];          // 小车经过上次决策后产生的 buy 组
	int check[5];						// 小车当前是否 check 一下是否有商品
	queue <int > total_check[5];		// 小车总的 check 组

	// Sel 是让小车去卖东西，Buy 是买
	void Sel(int car_num, int desk_num, int Check = 0);
	void Buy(int car_num, int desk_num, int Check = 0);
	void clear_decision(int k);

	bool md[4] = { 0,0,0,0 };
	bool md_7[4] = { 0,0,0,0 };
	bool md_9[4] = { 0,0,0,0 };
	bool md_stop_frame[4] = { 0,0,0,0 };
}command_class;


struct parameter_c
{
	int Stop_frame = 14500;
	double Time_Upscale = 1.2;
	double Earning_Upscale = 1.2;
	double End_frame = 14950;
	double fun1_desk_exist_num_downscale = 1;//1 - 未减权   0 - 所有工作台上的 4/5/6 产品不考虑
	double dis_pow_downscale = 5;

	void adjust_fun();
	double fun1(double remain, int type = 0);
	double fun2(bool output_is_ready, int output_is_doing, bool is_begin_now);
	double fun3(bool is_begin);
	double fun4(int current_frame, double distance, bool is_7, bool is_done);
	double fun5(bool is_7, bool is_empty, bool is_done, double is_doing, int desk_num, int goods);
	double fun6(int desk_num, int number_of_exists);
}parameter_class;


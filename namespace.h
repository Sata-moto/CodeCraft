#pragma once
#include "Global.h"
#include "desk.h"
extern ofstream output;

namespace seed_n
{
	extern int seed;
	extern int seeds[5];
	extern int seed_MOD;
}

namespace map_n
{
	extern int num_desk_7;
	extern int num_desk_9;
	extern char map[N][N];							// 地图
	extern double dis[2][52][N][N];
	extern bool can_not_move[2][8][N][N];			// 0↑ 1↗ 2→ 3↘ 4↓ 5↙ 6← 7↖

	extern bool is_MAX(int type, int desk_num, int x, int y);
	extern void dij(int desk_num, int type);
	extern void get_dis(int type);
	extern void init_wall();
	extern void init_desk();
}

namespace desk_n
{
	extern int desk_num[N][N];
	extern int cnt_desk;
	extern Desk desk[52];
	extern bool can_not_sell[52];
	extern vector <int > total_desk[10];
	extern void init_desk();
}

namespace car_n
{
	extern bool available_car[4]; // stop_frame 前转到 stop_frame 后时，小车空闲标记。
}

namespace constant_n
{
	extern double Earning[10];
	extern int money;
	extern int frame_number;
	extern int son[10][2];
	extern vector <int > father[10];			// son 和 father 描述了工作台之间的需求信息，在 init 中初始化
}

namespace wait_n
{
	extern bool is_waiting_for_7[10];							//某种物品有小车堵塞在了送到七号的过程种
	extern bool wait[4];							//当前小车送到了最后一个原料，但是当前产品还没有生产出来
	extern bool wait_until_spare_3[4];			//当前小车拿原料 1-3 时，发现 1-3 还没有生产好
	extern bool wait_until_spare_7[4];			//当前小车在第一层决策，准备拿取物品时发现没地方送导致等待
	extern bool wait_until_spare_sell[4];		//当前小车想要卖掉物品，但是没办法卖，导致等待（由优化引起）
	extern bool wait_stop_frame[4];				//stop_frame 后面的等待
}

namespace math_n
{
	extern int otoe(double x);									// 坐标到格子标号的映射（只约分，不区分横纵）
	extern pair <int, int > dtoe(int desk_num);					// 工作台标号到格子标号的映射
	extern pair <int, int > ctoe(int car_num);					// 小车标号到格子标号的映射
	extern pair <int, int > ztoe(double x, double y);			// 实坐标到格子标号的映射
	extern pair <double, double > etoz(int x, int y);			// 格子标号到实坐标的映射
	extern double dddis1(int desk1, int desk2);					// 某个工作台到另一个工作台，没有拿东西的距离
	extern double cddis1(int car1, int desk1);					// 某个小车到某个工作台，没有拿东西的距离
	extern double dddis2(int desk1, int desk2);					// 某个工作台到另一个工作台，拿东西的距离
	extern double cddis2(int car1, int desk1);					// 某个小车到某个工作台，拿东西的距离
}

namespace occupied_n
{
	extern int occupied[52][10];					// 工作台是否被占用
	extern int sol_occupied[52][10];				//
	extern int occupied_goods[10];					// 场上某种物品的数量
	extern int occupied_stop_frame[52][10];		// stop_frame 后的 occupied
	extern int sol_occupied_stop_frame[52][10];    // stop_frame 后的 sol_occupied
	extern int ignore_occupied[52][10];			// ignore occupied 从而允许连续的运送
	extern bool sol_ignore_occupied[52][10];		// 清楚 ignore occupied
	extern int current_occupied[52][10];			// 单纯是 check_spare_7 用到的临时参数，用来标识是否占用了一个 7

	extern void reload_occupied();					// 每帧会 reload occupied，ocuupied 进行占用时可以本帧占用，但是解除时必须下一帧解除
}

namespace assist_n
{
	extern bool full_6(int desk_num, int goods); //当前物品送到后，判断 4-6 号工作台是不是已经满了
	extern bool full_7(int dest, int goods);	 //当前物品送到后，判断 7 号工作台是不是已经满了
	extern bool check_spare_7(int type);		//当前是否有空闲的 7 号工作台
}

namespace command_n
{
	extern bool init_dc;
	extern int destination[5];                 // 小车在当前时间的目的地
	extern queue <int > total_destination[5];  // 小车经过上次决策后产生的目的地组
	extern int buy[5];                         // 1 为 buy,0 为 sel
	extern queue <int > total_buy[5];          // 小车经过上次决策后产生的 buy 组
	extern int check[5];						// 小车当前是否 check 一下是否有商品
	extern queue <int > total_check[5];		// 小车总的 check 组

	// Sel 是让小车去卖东西，Buy 是买
	extern void Sel(int car_num, int desk_num, int Check = 0);
	extern void Buy(int car_num, int desk_num, int Check = 0);
	extern void clear_decision(int k);

	extern bool md[4];
	extern bool md_7[4];
	extern bool md_9[4];
	extern bool md_stop_frame[4];
}

namespace parameter					//参数包
{
	extern int Stop_frame;
	extern double Time_Upscale;
	extern double Earning_Upscale;
	extern double End_frame;
	extern double fun1_desk_exist_num_downscale;//1 - 未减权   0 - 所有工作台上的 4/5/6 产品不考虑
	extern double dis_pow_downscale;

	extern void adjust_fun();
	extern double fun1(double remain, int type = 0);
	extern double fun2(bool output_is_ready, int output_is_doing, bool is_begin_now);
	extern double fun3(bool is_begin);
	extern double fun4(int current_frame, double distance, bool is_7, bool is_done);
	extern double fun5(bool is_7, bool is_empty, bool is_done, double is_doing, int desk_num, int goods);
	extern double fun6(int desk_num, int number_of_exists);
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

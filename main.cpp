#include "Global.h"
#include "car.h"
#include "desk.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include "namespace.h"
#include "namespace.cpp"
#include "decision.h"

ofstream fdebug;

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

void init()
{
	son[4][0] = 1, son[4][1] = 2;
	son[5][0] = 1, son[5][1] = 3;
	son[6][0] = 2, son[6][1] = 3;

	for (int k = 1; k <= 100; k++)
		for (int i = 1; i <= 100; i++)
			seed += (k * 100 + i) * map[k][i], seed %= seed_MOD;

	map_n::init_wall();
	map_n::init_desk();
	get_dis(0), get_dis(1);

	for (int k = 1; k <= 7; k++) father[k].push_back(9);
	for (int k = 4; k <= 6; k++) father[k].push_back(7);
	father[1].push_back(4), father[1].push_back(5);
	father[2].push_back(4), father[2].push_back(6);
	father[3].push_back(5), father[3].push_back(6);
	father[7].push_back(8);

	parameter::adjust_fun();
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
			desk_n::init_desk();
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

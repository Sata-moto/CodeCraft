#include "Global.h"
#include "car.h"
#include "desk.h"

char map[N][N]; // 地图
Car car[5];
Desk desk[52];
int cnt_desk;	// 一共有多少工作台

int money;        // 金钱数
int frame_number; // 帧序号

int destination[5];                 // 小车在当前时间的目的地
queue <int > total_destination[5];  // 小车经过上次决策后产生的目的地组
int buy[5];                         // 1 为 buy,0 为 sel
queue <int > total_buy[5];          // 小车经过上次决策后产生的 buy 组

// Sel 是让小车去卖东西，Buy 是买
void Sel(int car_num, int desk_num)
{
	total_destination[car_num].push(desk_num), total_buy[car_num].push(0);
}
void Buy(int car_num, int desk_num)
{
	total_destination[car_num].push(desk_num), total_buy[car_num].push(1);
}

vector <int > availalbe_desk[10];//各种工作台当前有哪些空闲的

//一次决策会产生一组指令，使用 Sel,Buy 构造它们
void make_decision(int car_num)
{
	//贪心决策：不考虑机器人之间的关系，所有决策看作：
	//从某个工作台上制作商品 A，从某些工作台上拿取 A
	//的原料，最终卖到 8/9 工作台。（即假设不会把物品
	//卖到某个工作台上直到结束）
	//任何一次决策后将该决策重复一次从而并行。并行的部
	//份是上一次的等待和这一次的原料获取。
	//完成子项目的顺序不影响最终时间

	for (int k = 1; k <= 9; k++)
		availalbe_desk[k].clear();
	for (int k = 0; k < cnt_desk; k++)
		if (desk[k].remain_time == -1)
			availalbe_desk[desk[k].type].push_back(k);
	//初始化工作台

	double max_earning = 0;
	int max_earning_desk_num = -1;

	if (max_earning_desk_num != -1)
	{

	}
	else
	{

	}
}

int main()
{
	for (int k = 1; k <= 101; k++)
		scanf("%s", &map[k][1]);
	//地图没有用，101行是因为最后一行 OK

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
			int input, input_cnt = 1, output;
			scanf("%d %d", &input, &output);
			while (input)
			{
				if (input % 2)
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
				destination[k] = total_destination[k].front();
				total_destination[k].pop();
				buy[k] = total_buy[k].front();
				total_buy[k].pop();
			}
		//第一帧初始化决策

		for (int k = 0; k < 4; k++)
		{
			if (car[k].workbench == destination[k])
			{
				if (buy[k] && desk[destination[k]].output_status == 1)
					printf("buy %d\n", k);
				else if (!buy[k] && (desk[destination[k]].input_status[car[k].goods] == 1))
					printf("sell %d\n", k);
				//对每个小车，如果已经到了目的地，并且可以做 buy/sell 指令，输出 buy,sell
				else continue;
				//如果因为挤占导致指令无法执行，就跳过决策并且等待。

				//从指令组中导入新的指令,如果指令组为空就新构指令
				if (total_destination[k].empty())
					make_decision(k);
				if (!total_destination[k].empty())
				{
					destination[k] = total_destination[k].front();
					total_destination[k].pop();
					buy[k] = total_buy[k].front();
					total_buy[k].pop();
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
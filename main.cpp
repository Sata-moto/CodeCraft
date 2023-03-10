#include "Global.h"
#include "car.h"
#include "desk.h"

char map[N][N];		//地图
Car car[5];
Desk desk[52];

int	money;			//金钱数
int frame_number;	//帧序号

int destination[5]; //小车在当前时间的目的地
int buy[5];			//1 为 buy,0 为 sel

//Sel 是让小车去卖东西，Buy 是买
void Sel(int car_num, int desk_num)
{
	destination[car_num] = desk_num, buy[car_num] = 0;
}
void Buy(int car_num, int desk_num)
{
	destination[car_num] = desk_num, buy[car_num] = 1;
}

int main()
{
	for (int k = 1; k <= 100; k++)
		scanf("%s", &map[k][1]);
	/*	int cnt_car = 0, cnt_desk = 0;
		for (int k = 1; k <= 100; k++)
			for (int i = 1; i <= 100; i++)
				if (map[k][i] == 'A')
				{
					car[++cnt_car].x = i / 2.0 - 0.25;
					car[cnt_car].y = 50.0 - k / 2.0 + 0.25;
				}
				else if (map[k][i] >= '1' && map[k][i] <= '9')
				{
					desk[++cnt_desk].type = map[k][i] - '0';
					desk[cnt_desk].x = i / 2.0 - 0.25;
					desk[cnt_desk].y = 50.0 - k / 2.0 + 0.25;
				}*/
				//后面会给出每个工作台和车的状态，这里似乎没必要初始化

	printf("OK\n");
	fflush(stdout);

	while (scanf("%d %d", &frame_number, money))
	{
		int cnt_desk;
		scanf("%d", &cnt_desk);
		for (register int k = 1; k <= cnt_desk; k++)
		{
			scanf("%d %lf %lf %d", &desk[k].type, &desk[k].x, &desk[k].y, &desk[k].remain_time);
			int input, input_cnt = 1;
			scanf("%d %d", &input, &desk[k].output_status);
			while (input)
			{
				if (input % 2)
					desk[k].input_status[input_cnt] = 1;
				input /= 2, input_cnt++;
			}
		}//初始化工作台
		for (register int k = 0; k < 4; k++)
		{
			scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf",
				&car[k].workbench, &car[k].goods, &car[k].timerate, &car[k].hitrate,
				&car[k].w, &car[k].vx, &car[k].vy, &car[k].ang, &car[k].x, &car[k].y);
		}
		char is_OK[10];
		scanf("%s", is_OK);

		//Decision();

		for (int k = 0; k < 4; k++)
		{
			pair <int, int > temp;
			temp = car[k].mov(desk[destination[k]].x, desk[destination[k]].y);
			printf("forward %d %lf\n", k, temp.first);
			printf("rotate %d %lf\n", k, temp.second);
			if (car[k].workbench == destination[k])
				if (buy[k]) printf("buy %d\n", k);
				else printf("sell %d\n", k);
		}
		printf("OK\n");
		fflush(stdout);
	}

	return 0;
}
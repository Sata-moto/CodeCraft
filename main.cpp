#include "Global.h"
#include "car.h"
#include "desk.h"

char map[N][N];		//��ͼ
Car car[5];
Desk desk[52];

int	money;			//��Ǯ��
int frame_number;	//֡���

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
				//��������ÿ������̨�ͳ���״̬�������ƺ�û��Ҫ��ʼ��

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
		}//��ʼ������̨
		for (register int k = 0; k < 4; k++)
		{
			scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf",
				&car[k].workbench, &car[k].goods, &car[k].timerate, &car[k].hitrate,
				&car[k].w, &car[k].vx, &car[k].vy, &car[k].ang, &car[k].x, &car[k].y);
		}
		char is_OK[10];
		scanf("%s", is_OK);
		
		//Decision();
	}

	return 0;
}
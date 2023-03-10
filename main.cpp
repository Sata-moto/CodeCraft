#include "Global.h"
#include "car.h"
#include "desk.h"

char map[N][N];		//µØÍ¼
Car car[4];
Desk desk[52];

int	money;			//½ðÇ®Êý
int frame_number;	//Ö¡ÐòºÅ

void work()
{


}

int main()
{
	for (int k = 1; k <= 100; k++)
		scanf("%s", &map[k][1]);
	int cnt_car = 0, cnt_desk = 0;
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
			}

	printf("OK\n");
	fflush(stdout);

	while(scanf("%d %d", &frame_number, money))
		work();

	return 0;
}
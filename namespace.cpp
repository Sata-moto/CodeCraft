#include "namespace.h"
#include "car.h"
#include "desk.h"
#include <queue>
#include <iostream>

ofstream output;
int seed_n::seed = 0;
int seed_n::seeds[5] = { 0,352354535,350895017,351758063,218668868 };
int seed_n::seed_MOD = 998244353;
int map_n::num_desk_7;
int map_n::num_desk_9;
char map_n::map[N][N];							// 地图
double map_n::dis[2][52][N][N];
bool map_n::can_not_move[2][8][N][N];			// 0↑ 1↗ 2→ 3↘ 4↓ 5↙ 6← 7↖
int desk_n::cnt_desk;
Desk desk_n::desk[52];
bool desk_n::can_not_sell[52];
int desk_n::desk_num[N][N];
vector <int > desk_n::total_desk[10];
bool car_n::available_car[4] = { 0,0,0,0 };
double constant_n::Earning[10] = { 0,3000,3200,3400,7100,7800,8300,29000 };
int constant_n::money;
int constant_n::frame_number;
int constant_n::son[10][2];
vector <int > constant_n::father[10];
bool wait_n::is_waiting_for_7[10];							//某种物品有小车堵塞在了送到七号的过程种
bool wait_n::wait[4] = { 0,0,0,0 };							//当前小车送到了最后一个原料，但是当前产品还没有生产出来
bool wait_n::wait_until_spare_3[4] = { 0,0,0,0 };			//当前小车拿原料 1-3 时，发现 1-3 还没有生产好
bool wait_n::wait_until_spare_7[4] = { 0,0,0,0 };			//当前小车在第一层决策，准备拿取物品时发现没地方送导致等待
bool wait_n::wait_until_spare_sell[4] = { 0,0,0,0 };		//当前小车想要卖掉物品，但是没办法卖，导致等待（由优化引起）
bool wait_n::wait_stop_frame[4] = { 0,0,0,0 };
int occupied_n::occupied[52][10];					// 工作台是否被占用
int occupied_n::sol_occupied[52][10];				//
int occupied_n::occupied_goods[10];					// 场上某种物品的数量
int occupied_n::occupied_stop_frame[52][10];		// stop_frame 后的 occupied
int occupied_n::sol_occupied_stop_frame[52][10];    // stop_frame 后的 sol_occupied
int occupied_n::ignore_occupied[52][10];			// ignore occupied 从而允许连续的运送
bool occupied_n::sol_ignore_occupied[52][10];		// 清楚 ignore occupied
int occupied_n::current_occupied[52][10];
bool command_n::init_dc;
int command_n::destination[5];                 // 小车在当前时间的目的地
queue <int > command_n::total_destination[5];  // 小车经过上次决策后产生的目的地组
int command_n::buy[5];                         // 1 为 buy,0 为 sel
queue <int > command_n::total_buy[5];          // 小车经过上次决策后产生的 buy 组
int command_n::check[5];						// 小车当前是否 check 一下是否有商品
queue <int > command_n::total_check[5];		// 小车总的 check 组
bool command_n::md[4] = { 0,0,0,0 };
bool command_n::md_7[4] = { 0,0,0,0 };
bool command_n::md_9[4] = { 0,0,0,0 };
bool command_n::md_stop_frame[4] = { 0,0,0,0 };
int parameter::Stop_frame = 14000;
double parameter::Time_Upscale = 1.2;
double parameter::Earning_Upscale = 1.2;
double parameter::End_frame = 14900;
double parameter::fun1_desk_exist_num_downscale = 1;//1 - 未减权   0 - 所有工作台上的 4/5/6 产品不考虑
double parameter::dis_pow_downscale = 5;

void desk_n::init_desk()
{
	for (int k = 1; k <= 9; k++)
		total_desk[k].clear();
	for (int k = 0; k < cnt_desk; k++)
	{
		if (!can_not_sell[k] || desk[k].type <= 3)
			total_desk[desk[k].type].push_back(k);
	}
}

#define MAXN 1000000000.0
bool map_n::is_MAX(int type, int desk_num, int x, int y)
{
	if (fabs(dis[type][desk_num][x][y] - MAXN) <= 0.01)
		return true;
	return false;
}

const int sun_shape[4][2] = { 2,1,2,-1,1,2,1,-2 };
const int big_sun_shape[4][2] = { 3,1,3,-1,1,3,1,-3 };
void map_n::init_wall()
{
	for (int k = 0; k <= 101; k++) map[k][0] = map[0][k] = map[k][101] = map[101][k] = '#';

	for (int k = 1; k <= 100; k++)
		for (int i = 1; i <= 100; i++)
			if (map[k][i] == '#')
			{
				// 直三空一直接连
				if (k + 2 <= 101 && map[k + 2][i] == '#')
					map[k + 1][i] = '#';
				if (i + 2 <= 101 && map[k][i + 2] == '#')
					map[k][i + 1] = '#';

				// 斜二紧连不跨越
				if (map[k + 1][i + 1] == '#')
					can_not_move[0][5][k][i + 1] = can_not_move[0][1][k + 1][i] = 1;
				if (map[k + 1][i - 1] == '#')
					can_not_move[0][3][k][i - 1] = can_not_move[0][7][k + 1][i] = 1;

				// 日字不过中轴线
				for (int j = 0; j < 4; j++)
				{
					int to_x = k + sun_shape[j][0];
					int to_y = i + sun_shape[j][1];
					if (to_x < 0 || to_x > 101 || to_y < 0 || to_y > 101)
						continue;
					if (map[to_x][to_y] != '#') continue;

					if (j == 0) can_not_move[0][5][k][i + 1] = can_not_move[0][5][k + 1][i + 1]
						= can_not_move[0][6][k + 1][i + 1] = can_not_move[0][2][k + 1][i]
						= can_not_move[0][1][k + 1][i] = can_not_move[0][1][k + 2][i] = 1;
					if (j == 1) can_not_move[0][3][k][i - 1] = can_not_move[0][3][k + 1][i - 1]
						= can_not_move[0][2][k + 1][i - 1] = can_not_move[0][6][k + 1][i]
						= can_not_move[0][7][k + 1][i] = can_not_move[0][7][k + 2][i] = 1;
					if (j == 2) can_not_move[0][4][k][i + 1] = can_not_move[0][5][k][i + 1]
						= can_not_move[0][5][k][i + 2] = can_not_move[0][1][k + 1][i]
						= can_not_move[0][1][k + 1][i + 1] = can_not_move[0][0][k + 1][i + 1] = 1;
					if (j == 3) can_not_move[0][3][k][i - 2] = can_not_move[0][3][k][i - 1]
						= can_not_move[0][4][k][i - 1] = can_not_move[0][0][k + 1][i - 1]
						= can_not_move[0][7][k + 1][i - 1] = can_not_move[0][7][k + 1][i] = 1;
				}

				// 田字中心不往前
				if (k + 2 <= 101 && i + 2 <= 101 && map[k + 2][i + 2] == '#')
				{
					for (int j = 0; j < 8; j++) can_not_move[0][j][k + 1][i + 1] = 1;
					can_not_move[0][5][k][i + 1] = can_not_move[0][1][k + 1][i] = 1;
					can_not_move[0][5][k + 1][i + 2] = can_not_move[0][1][k + 2][i + 1] = 1;
				}
				if (k + 2 <= 101 && i - 2 >= 0 && map[k + 2][i - 2] == '#')
				{
					for (int j = 0; j < 8; j++) can_not_move[0][j][k + 1][i + 1] = 1;
					can_not_move[0][3][k][i - 1] = can_not_move[0][7][k + 1][i] = 1;
					can_not_move[0][3][k + 1][i - 2] = can_not_move[0][7][k + 2][i - 1] = 1;
				}
			}
	memcpy(can_not_move[1], can_not_move[0], sizeof(can_not_move[0]));

	for (int k = 1; k <= 100; k++)
		for (int i = 1; i <= 100; i++)
			if (map[k][i] == '#')
			{
				// 真四空二也直连
				if (k + 3 <= 101 && map[k + 3][i] == '#')
				{
					if (map[k + 1][i] >= '0' && map[k + 1][i] <= '9')
						desk_n::can_not_sell[desk_num[k + 1][i]] = 1;
					if (map[k + 2][i] >= '0' && map[k + 2][i] <= '9')
						desk_n::can_not_sell[desk_num[k + 2][i]] = 1;
					can_not_move[1][3][k][i - 1] = can_not_move[1][5][k][i + 1]
						= can_not_move[1][3][k + 1][i - 1] = can_not_move[1][5][k + 1][i + 1]
						= can_not_move[1][2][k + 1][i - 1] = can_not_move[1][6][k + 1][i + 1]
						= can_not_move[1][2][k + 2][i - 1] = can_not_move[1][6][k + 2][i + 1]
						= can_not_move[1][1][k + 2][i - 1] = can_not_move[1][7][k + 2][i + 1]
						= can_not_move[1][1][k + 3][i - 1] = can_not_move[1][7][k + 3][i + 1] = 1;
				}
				if (i + 3 <= 101 && map[k][i + 3] == '#')
				{
					if (map[k][i + 1] >= '0' && map[k][i + 1] <= '9')
						desk_n::can_not_sell[desk_num[k][i + 1]] = 1;
					if (map[k][i + 2] >= '0' && map[k][i + 2] <= '9')
						desk_n::can_not_sell[desk_num[k][i + 2]] = 1;
					can_not_move[1][3][k - 1][i] = can_not_move[1][1][k + 1][i]
						= can_not_move[1][3][k - 1][i + 1] = can_not_move[1][1][k + 1][i + 1]
						= can_not_move[1][4][k - 1][i + 1] = can_not_move[1][0][k + 1][i + 1]
						= can_not_move[1][4][k - 1][i + 2] = can_not_move[1][0][k + 1][i + 2]
						= can_not_move[1][5][k - 1][i + 2] = can_not_move[1][7][k + 1][i + 2]
						= can_not_move[1][5][k - 1][i + 3] = can_not_move[1][7][k + 1][i + 3] = 1;
				}

				// 长日升权不能走
				for (int j = 0; j < 4; j++)
				{
					int to_x = k + big_sun_shape[j][0];
					int to_y = i + big_sun_shape[j][1];
					if (to_x < 0 || to_x > 101 || to_y < 0 || to_y > 101)
						continue;
					if (map[to_x][to_y] != '#') continue;

					if (j == 0) can_not_move[1][5][k][i + 1] = can_not_move[1][5][k + 1][i + 1]
						= can_not_move[1][6][k + 1][i + 1] = can_not_move[1][2][k + 1][i]
						= can_not_move[1][1][k + 1][i] = can_not_move[1][1][k + 2][i]
						= can_not_move[1][6][k + 2][i + 1] = can_not_move[1][5][k + 2][i + 1]
						= can_not_move[1][3][k + 1][i] = can_not_move[1][7][k + 2][i + 1]
						= can_not_move[1][2][k + 2][i] = can_not_move[1][1][k + 3][i] = 1;
					if (j == 1) can_not_move[1][3][k][i - 1] = can_not_move[1][3][k + 1][i - 1]
						= can_not_move[1][2][k + 1][i - 1] = can_not_move[1][6][k + 1][i]
						= can_not_move[1][7][k + 1][i] = can_not_move[1][7][k + 2][i]
						= can_not_move[1][2][k + 2][i - 1] = can_not_move[1][6][k + 2][i]
						= can_not_move[1][7][k + 3][i] = can_not_move[1][3][k + 2][i - 1]
						= can_not_move[1][1][k + 2][i - 1] = can_not_move[1][5][k + 1][i] = 1;
					if (j == 2) can_not_move[1][4][k][i + 1] = can_not_move[1][5][k][i + 1]
						= can_not_move[1][5][k][i + 2] = can_not_move[1][1][k + 1][i]
						= can_not_move[1][4][k][i + 2] = can_not_move[1][5][k][i + 3]
						= can_not_move[1][3][k][i + 1] = can_not_move[1][7][k + 1][i + 2]
						= can_not_move[1][0][k + 1][i + 2] = can_not_move[1][1][k + 1][i + 2]
						= can_not_move[1][1][k + 1][i + 1] = can_not_move[1][0][k + 1][i + 1] = 1;
					if (j == 3) can_not_move[1][3][k][i - 2] = can_not_move[1][3][k][i - 1]
						= can_not_move[1][4][k][i - 1] = can_not_move[1][0][k + 1][i - 1]
						= can_not_move[1][7][k + 1][i - 1] = can_not_move[1][7][k + 1][i]
						= can_not_move[1][3][k][i - 3] = can_not_move[1][4][k][i - 2]
						= can_not_move[1][0][k + 1][i - 2] = can_not_move[1][7][k + 1][i - 2]
						= can_not_move[1][5][k][i - 1] = can_not_move[1][1][k + 1][i - 2] = 1;
				}
			}
}

void map_n::init_desk()
{
	int cnt = 0;
	for (int k = 1; k <= 100; k++)
		for (int i = 1; i <= 100; i++)
			if (map[k][i] >= '0' && map[k][i] <= '9')
			{
				int temp_cnt = desk_num[k][i];
				cnt++;
				desk_n::desk[temp_cnt].x = math_n::etoz(k, i).first, desk_n::desk[temp_cnt].y = math_n::etoz(k, i).second;
				int flag[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };
				if (map[k - 1][i] == '#') flag[0] = 1;					//上
				if (map[k][i - 1] == '#') flag[1] = 1;					//左
				if (map[k + 1][i] == '#') flag[2] = 1;					//下
				if (map[k][i + 1] == '#') flag[3] = 1;					//右
				if (map[k - 1][i - 1] == '#') flag[4] = 1;				//左上
				if (map[k + 1][i - 1] == '#') flag[5] = 1;				//左下
				if (map[k + 1][i + 1] == '#') flag[6] = 1;				//右下
				if (map[k - 1][i + 1] == '#') flag[7] = 1;				//右上
				for (int k = 4; k <= 7; k++) flag[k + 4] = flag[k];

				for (int j = 0; j < 4; j++)
					if (flag[j] && flag[j + 5] || flag[j] && flag[j + 6])			//日字形
						desk_n::can_not_sell[temp_cnt] = 1;
				for (int j = 4; j <= 5; j++)
					if (flag[j] && flag[j + 2])
						desk_n::can_not_sell[temp_cnt] = 1;
				for (int j = 1; j <= 4; j++)
					if (flag[j] && flag[(j + 1) % 4])
						desk_n::can_not_sell[temp_cnt] = 1;
			}
	desk_n::cnt_desk = cnt;
}

#define mp make_pair
#define pii pair <int ,int > 
#define dis_z 0.5
#define dis_x 0.7071

const int Forward[8][2] = { -1,0,-1,1,0,1,1,1,1,0,1,-1,0,-1,-1,-1 };
void map_n::dij(int desk_num, int type)
{
	bool solved[N][N];
	priority_queue <pair < double, pii >, vector<pair < double, pii >>, greater<pair < double, pii >>> q;
	memset(solved, 0, sizeof(solved));
	q.push(mp(0, math_n::dtoe(desk_num)));
	dis[type][desk_num][math_n::dtoe(desk_num).first][math_n::dtoe(desk_num).second] = 0;

	while (!q.empty())
	{
		pair <double, pii > now = q.top();
		q.pop();
		int x = now.second.first;
		int y = now.second.second;
		double dist = now.first;
		if (solved[x][y]) continue;
		solved[x][y] = 1;

		for (int k = 0; k < 8; k++)
		{
			int to_x = x + Forward[k][0];
			int to_y = y + Forward[k][1];
			double new_dist = ((k % 2) ? dis_x : dis_z) + dist;

			if (can_not_move[type][k][x][y] || map[to_x][to_y] == '#')
				continue;
			if (dis[type][desk_num][to_x][to_y] > new_dist)
			{
				dis[type][desk_num][to_x][to_y] = new_dist;
				q.push(mp(new_dist, mp(to_x, to_y)));
			}
		}
	}
}
#undef mp
#undef pii

void map_n::get_dis(int type)
{
	for (int k = 0; k < desk_n::cnt_desk; k++)
		for (int i = 0; i <= 101; i++)
			for (int j = 0; j <= 101; j++)
				dis[type][k][i][j] = MAXN;

	for (int k = 0; k < desk_n::cnt_desk; k++)																																																									
		dij(k, type);

	/*
	output.precision(3);
	output.flags(ios::fixed);
	output.fill('0');
	for (int i = 1; i <= 100; i++) {
		for (int j = 1; j <= 100; j++) {
			output.width(7);
			if (fabs(dis[type][9][i][j] - 1e9) < 1e-2)
				output << 999.999 << "  ";
			else output << dis[type][9][i][j] << "  ";
		}
		output << endl;
	}
	output << endl;
	*/
	


}

#undef MAXN

int math_n::otoe(double x)
{
	return (int)floor(x * 2) + 1;
}

pair <int, int > math_n::dtoe(int desk_num)
{
	return make_pair(math_n::otoe(desk_n::desk[desk_num].x), math_n::otoe(desk_n::desk[desk_num].y));
}
pair <int, int > math_n::ctoe(int car_num)
{
	return make_pair(math_n::otoe(car[car_num].x), math_n::otoe(car[car_num].y));
}
pair <int, int > math_n::ztoe(double x, double y)
{
	return make_pair(math_n::otoe(x), math_n::otoe(y));
}
pair <double, double > math_n::etoz(int x, int y)
{
	return make_pair(x / 2.0 - 0.25, y / 2.0 - 0.25);
}


double math_n::dddis1(int desk1, int desk2)
{
	return map_n::dis[0][desk1][dtoe(desk2).first][dtoe(desk2).second];
}
double math_n::cddis1(int car1, int desk1)
{
	return map_n::dis[0][desk1][ctoe(car1).first][ctoe(car1).second];
}
double math_n::dddis2(int desk1, int desk2)
{
	return map_n::dis[1][desk1][dtoe(desk2).first][dtoe(desk2).second];
}
double math_n::cddis2(int car1, int desk1)
{
	return map_n::dis[1][desk1][ctoe(car1).first][ctoe(car1).second];
}

void occupied_n::reload_occupied()					// 每帧会 reload occupied，ocuupied 进行占用时可以本帧占用，但是解除时必须下一帧解除
{
	for (int k = 0; k < desk_n::cnt_desk; k++)
		for (int i = 0; i <= 9; i++)
			if (sol_occupied[k][i])
			{
				occupied[k][i] -= sol_occupied[k][i];
				occupied[k][i] = max(0, occupied[k][i]);
				sol_occupied[k][i] = 0;
			}
	for (int k = 0; k < desk_n::cnt_desk; k++)
		for (int i = 0; i <= 9; i++)
			if (sol_occupied_stop_frame[k][i])
			{
				occupied_stop_frame[k][i] = 0;
				sol_occupied_stop_frame[k][i] = 0;
			}
	for (int k = 0; k < desk_n::cnt_desk; k++)
		for (int i = 0; i <= 9; i++)
			if (sol_ignore_occupied[k][i])
			{
				ignore_occupied[k][i] = 0;
				sol_ignore_occupied[k][i] = 0;
			}
	memset(current_occupied, 0, sizeof(current_occupied));
}

bool assist_n::full_6(int desk_num, int goods) //当前物品送到后，判断 4-6 号工作台是不是已经满了
{
	if (desk_n::desk[desk_num].input_status[1] || desk_n::desk[desk_num].input_status[2] || desk_n::desk[desk_num].input_status[3])
		return true;
	return false;
}

bool assist_n::full_7(int dest, int goods)	 //当前物品送到后，判断 7 号工作台是不是已经满了
{
	if (goods == 4 && desk_n::desk[dest].input_status[5] && desk_n::desk[dest].input_status[6])
		return true;
	if (goods == 5 && desk_n::desk[dest].input_status[4] && desk_n::desk[dest].input_status[6])
		return true;
	if (goods == 6 && desk_n::desk[dest].input_status[4] && desk_n::desk[dest].input_status[5])
		return true;
	return false;
}

bool assist_n::check_spare_7(int type)		//当前是否有空闲的 7 号工作台
{
	//return true;//取消先等待再拿的决策
	if (map_n::num_desk_9 != 0) return true;
	for (int k = 0; k < (int)desk_n::total_desk[7].size(); k++)
	{
		int now = desk_n::total_desk[7][k];
		if (!occupied_n::occupied[now][type] && !desk_n::desk[now].input_status[type] && !occupied_n::current_occupied[now][type])
		{
			occupied_n::current_occupied[now][type] = 1;
			return true;
		}
	}
	return false;
}

void command_n::Sel(int car_num, int desk_num, int Check)
{
	total_destination[car_num].push(desk_num), total_buy[car_num].push(0), total_check[car_num].push(Check);
}
void command_n::Buy(int car_num, int desk_num, int Check)
{
	total_destination[car_num].push(desk_num), total_buy[car_num].push(1), total_check[car_num].push(Check);
}

void command_n::clear_decision(int k)
{
	if (!total_destination[k].empty())
	{
		total_destination[k].pop();
		total_buy[k].pop();
		total_check[k].pop();
	}
}

void parameter::adjust_fun()
{
	if (map_n::num_desk_7 != 0)
	{
		constant_n::Earning[1] = constant_n::Earning[3] = constant_n::Earning[2];
		constant_n::Earning[4] = constant_n::Earning[6] = constant_n::Earning[5];
	}

	if (seed_n::seed == seed_n::seeds[1])
	{

	}
	else if (seed_n::seed == seed_n::seeds[2])
	{

	}
	else if (seed_n::seed == seed_n::seeds[3])
	{

	}
	else if (seed_n::seed == seed_n::seeds[4])
	{

	}
}

double parameter::fun1(double remain, int type)
{
	if (map_n::num_desk_7 == 0) return 1;
	if (wait_n::is_waiting_for_7[type]) return 0;
	//if (seed == seeds[2]) return 1.0 / (remain + 1);
	else return pow(2.718, -remain);
}
double parameter::fun2(bool output_is_ready, int output_is_doing, bool is_begin_now)
{
	if (seed_n::seed == seed_n::seeds[1])
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
double parameter::fun3(bool is_begin)
{
	if (is_begin) return 1;
	else return 1.3;
}
double parameter::fun4(int current_frame, double distance, bool is_7, bool is_done)
{
	return 1;// fun4 不起作用
	if (current_frame > 8500)
		return 1.0 / distance;
	else return 1;
}
double parameter::fun5(bool is_7, bool is_empty, bool is_done, double is_doing, int desk_num, int goods)
{
	if (!is_7) return 0.5;
	else if (!is_empty) return -0.01;
	else if (is_done) return 1.5;
	else if (is_doing > 1000) return 1.3;
	else if (is_doing && !assist_n::full_7(desk_num, goods)) return max(1.0, 0.8 + is_doing / 1250.0);
	else if (is_doing) return 0.8 + is_doing / 1250.0;
	else return 1;
}
double parameter::fun6(int desk_num, int number_of_exists)
{
	if (desk_n::desk[desk_num].type != 7) return 1;
	return 1 + (number_of_exists + occupied_n::occupied[desk_num][4] + occupied_n::occupied[desk_num][5] + occupied_n::occupied[desk_num][6]) / 5.0;
}

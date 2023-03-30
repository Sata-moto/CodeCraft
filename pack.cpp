#include "Global.h"
#include "pack.h"

void init_desk()
{
	for (int k = 1; k <= 9; k++)
		total_desk[k].clear();
	for (int k = 0; k < cnt_desk; k++)
		if (!is_corner[k])
			total_desk[desk[k].type].push_back(k);
}

#define mp make_pair
#define pii pair <int ,int >
void dij(int desk_num)
{


}
#undef mp
#undef pii

void map_c::get_dis()
{
	for (int k = 0; k < desk_class.cnt_desk; k++)
		dij(k);
}

double math_c::dddis1(int desk1, int desk2)
{
	return map_class.dis1[desk1][(int)round(desk_class.desk[desk2].x * 2)][(int)round(desk_class.desk[desk2].y * 2)];
}
double math_c::cddis1(int car1, int desk1)
{
	return map_class.dis1[desk1][(int)round(car[car1].x * 2)][(int)round(car[car1].y * 2)];
}
double math_c::dddis2(int desk1, int desk2)
{
	return map_class.dis2[desk1][(int)round(desk_class.desk[desk2].x * 2)][(int)round(desk_class.desk[desk2].y * 2)];
}
double math_c::cddis2(int car1, int desk1)
{
	return map_class.dis2[desk1][(int)round(car[car1].x * 2)][(int)round(car[car1].y * 2)];
}

void occupied_c::reload_occupied()					// 每帧会 reload occupied，ocuupied 进行占用时可以本帧占用，但是解除时必须下一帧解除
{
	for (int k = 0; k < desk_class.cnt_desk; k++)
		for (int i = 0; i <= 9; i++)
			if (sol_occupied[k][i])
			{
				occupied[k][i] -= sol_occupied[k][i];
				occupied[k][i] = max(0, occupied[k][i]);
				sol_occupied[k][i] = 0;
			}
	for (int k = 0; k < desk_class.cnt_desk; k++)
		for (int i = 0; i <= 9; i++)
			if (sol_occupied_stop_frame[k][i])
			{
				occupied_stop_frame[k][i] = 0;
				sol_occupied_stop_frame[k][i] = 0;
			}
	for (int k = 0; k < desk_class.cnt_desk; k++)
		for (int i = 0; i <= 9; i++)
			if (sol_ignore_occupied[k][i])
			{
				ignore_occupied[k][i] = 0;
				sol_ignore_occupied[k][i] = 0;
			}
	memset(current_occupied, 0, sizeof(current_occupied));
}

bool assist_c::full_6(int desk_num, int goods) //当前物品送到后，判断 4-6 号工作台是不是已经满了
{
	if (desk_class.desk[desk_num].input_status[1] || desk_class.desk[desk_num].input_status[2] || desk_class.desk[desk_num].input_status[3])
		return true;
	return false;
}

bool assist_c::full_7(int dest, int goods)	 //当前物品送到后，判断 7 号工作台是不是已经满了
{
	if (goods == 4 && desk_class.desk[dest].input_status[5] && desk_class.desk[dest].input_status[6])
		return true;
	if (goods == 5 && desk_class.desk[dest].input_status[4] && desk_class.desk[dest].input_status[6])
		return true;
	if (goods == 6 && desk_class.desk[dest].input_status[4] && desk_class.desk[dest].input_status[5])
		return true;
	return false;
}

bool assist_c::check_spare_7(int type)		//当前是否有空闲的 7 号工作台
{
	//return true;//取消先等待再拿的决策
	if (map_class.num_desk_9 != 0) return true;
	for (int k = 0; k < (int)desk_class.total_desk[7].size(); k++)
	{
		int now = desk_class.total_desk[7][k];
		if (!occupied_class.occupied[now][type] && !desk_class.desk[now].input_status[type] && !occupied_class.current_occupied[now][type])
		{
			occupied_class.current_occupied[now][type] = 1;
			return true;
		}
	}
	return false;
}

void command_c::Sel(int car_num, int desk_num, int Check)
{
	total_destination[car_num].push(desk_num), total_buy[car_num].push(0), total_check[car_num].push(Check);
}
void command_c::Buy(int car_num, int desk_num, int Check)
{
	total_destination[car_num].push(desk_num), total_buy[car_num].push(1), total_check[car_num].push(Check);
}

void command_c::clear_decision(int k)
{
	if (!total_destination[k].empty())
	{
		total_destination[k].pop();
		total_buy[k].pop();
		total_check[k].pop();
	}
}

void parameter_c::adjust_fun()
{
	if (map_class.num_desk_7 != 0)
	{
		constant_class.Earning[1] = constant_class.Earning[3] = constant_class.Earning[2];
		constant_class.Earning[4] = constant_class.Earning[6] = constant_class.Earning[5];
	}

	if (seed_class.seed == seed_class.seeds[1])
	{

	}
	else if (seed_class.seed == seed_class.seeds[2])
	{

	}
	else if (seed_class.seed == seed_class.seeds[3])
	{

	}
	else if (seed_class.seed == seed_class.seeds[4])
	{

	}
}

double parameter_c::fun1(double remain, int type)
{
	if (map_class.num_desk_7 == 0) return 1;
	if (wait_class.is_waiting_for_7[type]) return 0;
	//if (seed == seeds[2]) return 1.0 / (remain + 1);
	else return pow(2.718, -remain);
}
double parameter_c::fun2(bool output_is_ready, int output_is_doing, bool is_begin_now)
{
	if (seed_class.seed == seed_class.seeds[1])
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
double parameter_c::fun3(bool is_begin)
{
	if (is_begin) return 1;
	else return 1.3;
}
double parameter_c::fun4(int current_frame, double distance, bool is_7, bool is_done)
{
	return 1;// fun4 不起作用
	if (current_frame > 8500)
		return 1.0 / distance;
	else return 1;
}
double parameter_c::fun5(bool is_7, bool is_empty, bool is_done, double is_doing, int desk_num, int goods)
{
	if (!is_7) return 0.5;
	else if (!is_empty) return -0.01;
	else if (is_done) return 1.5;
	else if (is_doing > 1000) return 1.3;
	else if (is_doing && !assist_class.full_7(desk_num, goods)) return max(1.0, 0.8 + is_doing / 1250.0);
	else if (is_doing) return 0.8 + is_doing / 1250.0;
	else return 1;
}
double parameter_c::fun6(int desk_num, int number_of_exists)
{
	if (desk_class.desk[desk_num].type != 7) return 1;
	return 1 + (number_of_exists + occupied_class.occupied[desk_num][4] + occupied_class.occupied[desk_num][5] + occupied_class.occupied[desk_num][6]) / 5.0;
}
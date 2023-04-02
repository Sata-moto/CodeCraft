#pragma once

#include "Global.h"
#include "namespace.h"

#define MAXN 1000000000.0

void make_decision(int car_num)
{
	double max_earning = 0;
	int max_earning_desk_num = -1;
	int son_desk;
	bool is_begin = false;

	for (register int k = 4; k <= 6; k++)
	{
		for (int i = 0; i < (int)total_desk[k].size(); i++)
		{
			int now = total_desk[k][i];
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
				for (int p = 0; p < (int)total_desk[son[k][0]].size(); p++)
				{
					int son_DESK = total_desk[son[k][0]][p];
					double DIS = cddis1(car_num, son_DESK) + dddis2(son_DESK, now);
					if (DIS < min_distance)
						min_distance = DIS, son_Desk = son_DESK;
				}
			if (cal_son2)
				for (int q = 0; q < (int)total_desk[son[k][1]].size(); q++)
				{
					int son_DESK = total_desk[son[k][1]][q];
					double DIS = cddis1(car_num, son_DESK) + dddis2(son_DESK, now);
					if (DIS < min_distance)
						min_distance = DIS, son_Desk = son_DESK;
				}
			//计算最小距离

			if (son_Desk == -1) continue;
			if (min_distance > MAXN) continue;

			double exist_count = occupied_goods[k];
			for (int j = 0; j < (int)total_desk[7].size(); j++)
				exist_count += desk[total_desk[7][j]].input_status[k];
			for (int j = 0; j < (int)total_desk[k].size(); j++)
				exist_count -= desk[total_desk[k][j]].output_status * (1 - parameter::fun1_desk_exist_num_downscale);
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
	double max_earning = 0.000000000001;
	int max_earning_desk_num = -1;
	for (register int k = 7; k <= 9; k++)
	{
		if (k == 8) continue;
		for (int i = 0; i < (int)total_desk[k].size(); i++)
		{
			int now = total_desk[k][i];
			double weight = 0;

			if (cddis2(car_num, now) - MAXN > -1) continue;
			weight = Earning[goods] / pow(cddis2(car_num, now), 1 / parameter::dis_pow_downscale) * parameter::fun4(frame_number, cddis2(car_num, now), k == 7 ? 1 : 0, desk[now].output_status)
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
	double cloest_distance = 9999999999;
	int cloest_desk = -1;

	for (register int k = 8; k <= 9; k++)
	{
		for (int i = 0; i < (int)total_desk[k].size(); i++)
		{
			int now = total_desk[k][i];
			if (cddis2(car_num, now) < cloest_distance)
			{
				cloest_desk = now;
				cloest_distance = cddis2(car_num, now);
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
}

void make_decision_stop_frame(int car_num)
{
	double max_weight = -1;
	int buy_desk = -1, sell_desk = -1;

	for (int k = 1; k <= 7; k++)
	{
		for (int i = 0; i < (int)total_desk[k].size(); i++)
		{
			int now = total_desk[k][i];
			if (!desk[now].output_status || occupied_stop_frame[now][0])
				continue;
			for (int j = 0; j < (int)father[k].size(); j++)
			{
				for (int t = 0; t < (int)total_desk[father[k][j]].size(); t++)
				{
					int to = total_desk[father[k][j]][t];
					if (desk[to].input_status[k] || occupied[to][k] || occupied[to][0])
						continue;
					if (father[k][j] <= 7 && occupied_stop_frame[to][k])
						continue;

					double dis = cddis1(car_num, now) + dddis2(now, to);
					if (dis > MAXN) continue;
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
}

void make_decision_without_7(int car_num)
{
	double max_earning = 0;
	int max_earning_desk_num = -1;
	int son_desk1 = -1, son_desk2 = -1;

	for (register int k = 4; k <= 6; k++)
	{

		for (int i = 0; i < (int)total_desk[k].size(); i++)
		{
			int now = total_desk[k][i];
			if (occupied[now][0]) continue;
			int son_Desk1 = -1, son_Desk2 = -1;
			double weight = 0, min_distance = 999999999;

			for (int p = 0; p < (int)total_desk[son[k][0]].size(); p++)
				for (int q = 0; q < (int)total_desk[son[k][1]].size(); q++)
				{
					int son_DESK1 = total_desk[son[k][0]][p];
					int son_DESK2 = total_desk[son[k][1]][q];
					double DIS1 = (cddis1(car_num, son_DESK1) + dddis2(son_DESK1, now) + dddis1(now, son_DESK2) + dddis2(now, son_DESK2));
					double DIS2 = (cddis1(car_num, son_DESK2) + dddis2(son_DESK2, now) + dddis1(now, son_DESK1) + dddis2(now, son_DESK1));
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

			if (son_Desk1 == -1) continue;

			double exist_count = occupied_goods[k];
			for (int j = 0; j < (int)total_desk[7].size(); j++)
				exist_count += desk[total_desk[7][j]].input_status[k];
			for (int j = 0; j < (int)total_desk[k].size(); j++)
				exist_count -= desk[total_desk[k][j]].output_status * (1 - parameter::fun1_desk_exist_num_downscale);
			if (min_distance > MAXN) continue;

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
}

#undef MAXN

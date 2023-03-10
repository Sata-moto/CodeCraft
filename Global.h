#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

using namespace std;
const int N = 102;

void Sel(int car_num, int desk_num); // 让编号为 k 的小车（编号从 0 开始）前往编号为 i 的工作台（编号从 1 开始）卖东西。
void Buy(int car_num, int desk_num); // 让编号为 k 的小车（编号从 0 开始）前往编号为 i 的工作台（编号从 1 开始）买东西。

#pragma once


//几何/数学
pair<double, double> getVec(double);                                                                                   // 得到指向对应角度的单位向量
double Sign(double);                                                                                                   // 提取输入参数的符号
double Dot(double, double, double, double);                                                                            // 点积（点对形式）
double Dot(pair<double, double>, pair<double, double>);                                                                // 点积（向量形式）
double Cross(double, double, double, double);                                                                          // 叉积（点对形式）
double Cross(pair<double, double>, pair<double, double>);                                                              // 叉积（向量形式）
pair<double, double> Rotate(pair<double, double>, double);                                                             // 旋转向量
pair<double, double> multi(pair<double, double>, double);                                                              // 向量乘标量
pair<double, double> Add(pair<double, double>, pair<double, double>);                                                  // 向量加向量
pair<double, double> Sub(pair<double, double>, pair<double, double>);                                                  // 向量减向量
double Dist(double, double, double, double);                                                                           // 计算两个点之间的距离（double点对形式）
double Dist(pair<double, double>, pair<double, double>);                                                               // 计算两个点之间的距离（pair形式）
double PointToLine(pair<double, double>, pair<double, double>, pair<double, double>);                                  // 计算点到直线距离
double PointToSegment(pair<double, double>, pair<double, double>, pair<double, double>);                               // 计算点到线段距离
bool SegmentCross(pair<double, double>, pair<double, double>, pair<double, double>, pair<double, double>);             // 判断线段是否相交
pair<double, double> CrossPoint(pair<double, double>, pair<double, double>, pair<double, double>, pair<double, double>);// 直线求交点
void AdjuAng(double&);                                                                                                 // 调整角度范围使其落在-Pi到Pi之间
double CombineV(double, double);                                                                                       // 计算合速度（点对形式，非负）
double CombineV(pair<double, double>);                                                                                 // 计算合速度（向量形式，非负）
void UnitV(pair<double, double>&);                                                                                     // 单位化向量
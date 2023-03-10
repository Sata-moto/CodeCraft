#pragma once

struct Car
{
	int workbench, goods;	  // ��������̨��ţ�Я����Ʒ���
	double timerate, hitrate; // ʱ���ֵϵ������ײ��ֵϵ��
	double vx, vy, w;		  // ��ά���ٶ����������ٶ�
	double ang, x, y;		  // ����Ƕȣ�����
	pair<double, double> mov(double, double);
};

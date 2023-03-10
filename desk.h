#pragma once

struct Desk
{
	int type;			  // ����̨������
	double x, y;		  // ����̨������
	int remain_time = -1; // ����̨ʣ������ʱ�䣨֡����-1 ����û��������0 ������Ϊ��������˶�������
	bool input_status[7]; // ����̨ԭ���ϸ�״̬��1 ��ʾ����Ʒ����
	bool output_status;	  // ����̨�����״̬��1 ��ʾ����Ʒ��
};
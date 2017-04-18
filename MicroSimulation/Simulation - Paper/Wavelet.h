#include "stdafx.h"
#include <windows.h>
#include <iostream>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <dos.h>
#include <vector>
#include <string.h>

using namespace std;

struct Wavelet_Information
{
	double Location;
	long Time;
	double Energy_value;
};

//struct wavelethistory
//{
//	vector<Wavelet_Information> WaveletData;		
//};

struct wavelet_history_time
{
	vector<vector<Wavelet_Information>> Wavelet_History;
};

vector<wavelet_history_time> Wavelet_History_Time;

double Energy;
int Car_Wavelet_History[10000];


struct Wavelet_Information wavelet (vector<data> vehicle_info, long time_counter)
{
	vector<double> Energy;
	double Energy_temp = 0;
	int a_max = 1000;
	double T_wavelet = 0;
	long b_max, b_min;
	Wavelet_Information temp_info;
	temp_info = NULL;
	temp_info = (Wavelet_Information*)malloc(sizeof(temp_info));

	for (long b = 0; b < vehicle_info.size(); b++)
	{
		for (float a = 0; a < a_max; a++)
		{
			for (long i = 0; i < vehicle_info.size(); i++)
			{
				T_wavelet = T_wavelet + (vehicle_info[i].t) * (1 - ((i - b) /a)) * exp((-1) * (1 - ((i - b) / (2 * a)))*(1 - ((i - b) / (2 * a))));
			}
			T_wavelet = (1 / sqrt(a)) * T_wavelet;
			Energy_temp = Energy_temp + (T_wavelet * T_wavelet);
		}
		Energy.push_back ((1 / a_max) * Energy_temp);
	}

	b_max = vehicle_info.size() - 1;
	b_min = b_max - 32;
	if(b_min < 0)
		b_min = 0;

	temp_info.Energy_value = Energy[b_min];
	temp_info.Location = vehicle_info[b_min].x;
	temp_info.Time = time_counter;

	return (temp_info);
}


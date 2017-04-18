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
#include "SpeedFlowDensity.h"

struct ramp_metering
{
	int ramp;
	double detector_location;
};
vector<ramp_metering> metering_info;
vector<float> occupancy_info;
vector<float> metering_rate;
vector<float> metering_interarrivaltime;
vector<float> metering_depart;
float current_interval = 0;

int ramp_metering_location(int a)
{
	ramp_metering temp;
	FILE *st;
//	st = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Detectors_Positions.txt","r");
	st = fopen("..\\I-O Files\\Detectors_Positions.txt","r");

	Mode_Previous = -1;
	while(!feof(st))
	{
		temp.ramp = g_Readnum(st);
		temp.detector_location = g_Readdouble(st);		
		metering_info.push_back(temp);
		
		occupancy_info.push_back(0);
		metering_rate.push_back(0);
		metering_interarrivaltime.push_back(0);
		metering_depart.push_back(0);
	}
	fclose(st);

	return (a);
};

int occupancy (car * p)
{	
	if (p != NULL)
	{
		while (p->next != NULL)
		{
			for (int i = 0; i < metering_info.size() - 1; i++)
			{
				if (p->x_prev <= metering_info[i].detector_location && p->x >= metering_info[i].detector_location && p->x <= (metering_info[i].detector_location + 1.8))
				{
					occupancy_info[i] = occupancy_info[i] + (p->x - metering_info[i].detector_location)/p->v;
					break;
				}
				else if (p->x_prev <= metering_info[i].detector_location && p->x > (metering_info[i].detector_location + 1.8))
				{
					occupancy_info[i] = occupancy_info[i] + (1.8 + 5)/p->v;
					break;
				}
				else if (p->x_prev >= metering_info[i].detector_location && p->x_prev <= (metering_info[i].detector_location + 1.8) && p->x > (metering_info[i].detector_location + 1.8))
				{
					occupancy_info[i] = occupancy_info[i] + (metering_info[i].detector_location + 1.8 - p->x_prev + 5)/p->v;
					break;
				}
				else if (p->x_prev >= metering_info[i].detector_location && p->x_prev <= (metering_info[i].detector_location + 1.8) && p->x <= (metering_info[i].detector_location + 1.8))
				{
					occupancy_info[i] = occupancy_info[i] + 0.1;
					break;
				}
			}
			p = p->next;
		}
	}
	return (1);
}
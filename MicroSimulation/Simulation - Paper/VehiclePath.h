/*
side 0
		   \  L20i \		  /	 L20i /
			\		\		 /		 /
			 \		 --------		/
			  \			(6)		   /
--------------- --  --  --  -- -- ---------------------
L0-lane 0		-->q  r2	(5)				(0)
-------------------------------------------------------
L1-lane 1		-->p  r1					(1)
-------------------------------------------------------
L2-lane 2		-->s  r3					(1)
-------------------------------------------------------
L3-lane 3					(3)				(2)
--------------- --  --  --  -- -- ---------------------
			  /		   (4)		  \
			 /      ---------	   \
			/      /		 \	    \
		   / L10i /			  \ L10i \
side 1
*/

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
#include "cognisim.h"
#include "Emission.h"



struct Ramp_Info
{
	short Ramp_Number;
	short Ramp_Type;//0 for onramp, 1 for offramp, 2 for both on and off ramps
	short Ramp_Side;//0 if on left, 1 if on right (normal case) on the direction of travel	
	float Ramp_Location;
	float Ramp_Merging_Length;
	float Ramp_StartPoint;
	float Ramp_EndPoint;
};
vector <Ramp_Info> ramp_information;

vector<vector<short>> path_information; // for  each path indicates link numbers

vector<vector<short>> Path; // for each vehicle indicates path and link numbers

vector<vector<short>> demand;

vector<vector<short>> MEAN_INTERARRIVAL_TIME;

int read_ramp_location(int a)
{
	Ramp_Info temp;
	FILE *st;
//	st = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Ramp Location.txt","r");
	st = fopen("..\\I-O Files\\Ramp Location.txt","r");


	Mode_Previous = -1;
	while(!feof(st))
	{
		temp.Ramp_Number = g_Readnum(st);
		temp.Ramp_Type = g_Readnum(st);
		temp.Ramp_Side = g_Readnum(st);
		temp.Ramp_Location = g_Readdouble(st);
		temp.Ramp_Merging_Length = g_Readdouble(st);
		temp.Ramp_StartPoint = g_Readdouble(st);
		temp.Ramp_EndPoint = g_Readdouble(st);
		
		ramp_information.push_back(temp);		
	}
	fclose(st);

	return (a);
};


int read_path (int a)
{
	vector<short> temp;
	short i = 0;

	FILE *st;
//	st = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Path Info.txt","r");
	st = fopen("..\\I-O Files\\Path Info.txt","r");

	while (!feof(st))
	{
		temp.clear();
		i = 0;
		while (i != -1000)
		{
			i = g_Readnum(st);
			if (i != -1000)
				temp.push_back(i);
			else
				break;
		}
		path_information.push_back(temp);		
	}
	return(a);
};

int read_demand(int a)
{
	vector<short> temp;
	short i = 0;

	FILE *st;
//	st = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Demand.txt","r");
	st = fopen("..\\I-O Files\\Demand.txt","r");


	while (!feof(st))
	{
		temp.clear();
		i = 0;
		while (i != -1000)
		{
			i = g_Readnum(st);
			if (i != -1000)
				temp.push_back(i);
			else
				break;
		}
		demand.push_back(temp);		
	}

	return(a);
};

int read_interarrivaltime(int a)
{
	vector<short> temp;
	short i = 0;

	FILE *st;
//	st = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Inter Arrival Time.txt","r");
	st = fopen("..\\I-O Files\\Inter Arrival Time.txt","r");


	while (!feof(st))
	{
		temp.clear();
		i = 0;
		while (i != -1000)
		{
			i = g_Readnum(st);
			if (i != -1000)
				temp.push_back(i);
			else
				break;
		}
		MEAN_INTERARRIVAL_TIME.push_back(temp);		
	}

	return(a);
};
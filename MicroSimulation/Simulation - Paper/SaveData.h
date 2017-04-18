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
#include "Initialization.h"



int save_data_mainstream (car * r, int TIME, short VSLActivation_temp, short VSLActivationFlag_temp, short VSLTIMECounter_temp)
{
	data tempData;
	short c = -1;
	while (r != NULL)
	{
		c = r->carID;
		tempData.x = r->x;
		tempData.l = r->lane;
		tempData.v = r->v;
		tempData.h = r->headway;
		tempData.t = TIME;
		tempData.crashlocation = r->crash;

		if (r->x > 0.0 && r->x_prev <=L)
		{				
			if (TIME > -1)//if (TIME%AGG_TIMEINTERVAL_TRAJECTORTY==0)
			{				
				AggCarsData[c].AggData.push_back(tempData);					
			}

			if (TIME % 100 == 0 && VSLActivation_temp == 1 && (VSLActivationFlag_temp == 0 || VSLTIMECounter_temp >= 6000))
			{
				//Wavelet_Information *wavelettemp;
				Wavelet_Information wavelettemp1;
				Wavelet_Information wavelettemp2;
				Wavelet_Information wavelettemp3;
				Wavelet_Information wavelettemp4;
				Wavelet_Information wavelettemp5;
				//wavelettemp = NULL;
				//wavelettemp= (Wavelet_Information*)malloc(sizeof(Wavelet_Information));
				wavelettemp1 = wavelet(AggCarsData[c].AggData,TIME,1);
				wavelettemp2 = wavelet(AggCarsData[c].AggData,TIME,2);
				wavelettemp3 = wavelet(AggCarsData[c].AggData,TIME,3);
				wavelettemp4 = wavelet(AggCarsData[c].AggData,TIME,4);
				wavelettemp5 = wavelet(AggCarsData[c].AggData,TIME,5);
				
				//wavelettemp1.Energy_value = wavelettemp->Energy_value;
				//wavelettemp1.Location = wavelettemp->Location;
				//wavelettemp1.Time = wavelettemp->Time;
				
				//Wavelet_History_Time[TIME].Wavelet_History[c].WaveletData.push_back(wavelettemp1);
				Wavelet_History_Time[int(TIME/100)].Wavelet_History[c].push_back (wavelettemp1);
				Wavelet_History_Time[int(TIME/100)].Wavelet_History[c].push_back (wavelettemp2);
				Wavelet_History_Time[int(TIME/100)].Wavelet_History[c].push_back (wavelettemp3);
				Wavelet_History_Time[int(TIME/100)].Wavelet_History[c].push_back (wavelettemp4);
				Wavelet_History_Time[int(TIME/100)].Wavelet_History[c].push_back (wavelettemp5);
				wavelet_pushback_counter.push_back(c);
			}
		}
		r = r->next;
	}
	return (0);
};

int save_data_ramp (car * r, int TIME)
{
	data tempData;
	short c = -1;
	short counter_ramp = 0;
	if (r != NULL)
	{
		while (r->lane != ramp_information[counter_ramp].Ramp_Number)
			counter_ramp++;

		while (r != NULL)
		{
			c = r->carID;
			tempData.x = r->x;
			tempData.l = r->lane;
			tempData.v = r->v;
			tempData.h = r->headway;
			tempData.t = TIME;
			tempData.crashlocation = r->crash;
							
			if (r->x > 0.0 && r->x_prev <= ramp_information[counter_ramp].Ramp_EndPoint)
			{				
				if (TIME > -1)//if (TIME%AGG_TIMEINTERVAL_TRAJECTORTY==0)
				{				
					AggCarsData[c].AggData.push_back(tempData);					
				}
			}
			r = r->next;
		}
	}
	return (0);
};

int section_output (car * saveLi_Hazard)
{
	int Mode_temp;
	data2 tempDataDensity; 
	data3 tempDataSpeed;
	data3 tempDataEmission;
	data4 tempDataAccHazard;
	car * check = saveLi_Hazard;

	tempDataDensity.Section1 = 0;
	tempDataDensity.Section2 = 0;
	tempDataDensity.Section3 = 0;
	tempDataDensity.Section4 = 0;
	tempDataDensity.Section5 = 0;
	tempDataDensity.Section6 = 0;
	tempDataDensity.Section7 = 0;
	tempDataDensity.Section8 = 0;
	tempDataDensity.Section9 = 0;
	tempDataDensity.Section10 = 0;
	tempDataDensity.Hazard1 = 0;
	tempDataDensity.Hazard2 = 0;
	tempDataDensity.Hazard3 = 0;
	tempDataDensity.Hazard4 = 0;
	tempDataDensity.Hazard5 = 0;
	tempDataDensity.Hazard6 = 0;
	tempDataDensity.Hazard7 = 0;
	tempDataDensity.Hazard8 = 0;
	tempDataDensity.Hazard9 = 0;
	tempDataDensity.Hazard10 = 0;
	tempDataDensity.Hazardcounter1 = 0;
	tempDataDensity.Hazardcounter2 = 0;
	tempDataDensity.Hazardcounter3 = 0;
	tempDataDensity.Hazardcounter4 = 0;
	tempDataDensity.Hazardcounter5 = 0;
	tempDataDensity.Hazardcounter6 = 0;
	tempDataDensity.Hazardcounter7 = 0;
	tempDataDensity.Hazardcounter8 = 0;
	tempDataDensity.Hazardcounter9 = 0;
	tempDataDensity.Hazardcounter10 = 0;
	tempDataSpeed.Section1 = 0;
	tempDataSpeed.Section2 = 0;
	tempDataSpeed.Section3 = 0;
	tempDataSpeed.Section4 = 0;
	tempDataSpeed.Section5 = 0;
	tempDataSpeed.Section6 = 0;
	tempDataSpeed.Section7 = 0;
	tempDataSpeed.Section8 = 0;
	tempDataSpeed.Section9 = 0;
	tempDataSpeed.Section10 = 0;
	tempDataEmission.Section1 = 0;
	tempDataEmission.Section2 = 0;
	tempDataEmission.Section3 = 0;
	tempDataEmission.Section4 = 0;
	tempDataEmission.Section5 = 0;
	tempDataEmission.Section6 = 0;
	tempDataEmission.Section7 = 0;
	tempDataEmission.Section8 = 0;
	tempDataEmission.Section9 = 0;
	tempDataEmission.Section10 = 0;
	tempDataAccHazard.Section1 = 0;
	tempDataAccHazard.Section2 = 0;
	tempDataAccHazard.Section3 = 0;
	tempDataAccHazard.Section4 = 0;
	tempDataAccHazard.Section5 = 0;
	tempDataAccHazard.Section6 = 0;
	tempDataAccHazard.Section7 = 0;
	tempDataAccHazard.Section8 = 0;
	tempDataAccHazard.Section9 = 0;
	tempDataAccHazard.Section10 = 0;
	while (saveLi_Hazard != NULL)
	{
		if (saveLi_Hazard->x > 0.0 && saveLi_Hazard->x_prev <L)
		{				
			if (saveLi_Hazard->x > 0 && saveLi_Hazard->x <= 500)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section1 = tempDataEmission.Section1 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section1 = tempDataDensity.Section1 + 1;
				tempDataSpeed.Section1 = tempDataSpeed.Section1 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard1 = tempDataDensity.Hazard1 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter1 = tempDataDensity.Hazardcounter1 + 1;
				}
				tempDataAccHazard.Section1 = tempDataAccHazard.Section1 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 500 && saveLi_Hazard->x <= 1000)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section2 = tempDataEmission.Section2 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section2 = tempDataDensity.Section2 + 1;
				tempDataSpeed.Section2 = tempDataSpeed.Section2 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard2 = tempDataDensity.Hazard2 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter2 = tempDataDensity.Hazardcounter2 + 1;
				}
				tempDataAccHazard.Section2 = tempDataAccHazard.Section2 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 1000 && saveLi_Hazard->x <= 1500)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section3 = tempDataEmission.Section3 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section3 = tempDataDensity.Section3 + 1;
				tempDataSpeed.Section3 = tempDataSpeed.Section3 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard3 = tempDataDensity.Hazard3 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter3 = tempDataDensity.Hazardcounter3 + 1;
				}
				tempDataAccHazard.Section3 = tempDataAccHazard.Section3 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 1500 && saveLi_Hazard->x <= 2000)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section4 = tempDataEmission.Section4 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section4 = tempDataDensity.Section4 + 1;
				tempDataSpeed.Section4 = tempDataSpeed.Section4 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard4 = tempDataDensity.Hazard4 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter4 = tempDataDensity.Hazardcounter4 + 1;
				}
				tempDataAccHazard.Section4 = tempDataAccHazard.Section4 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 2000 && saveLi_Hazard->x <= 2500)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section5 = tempDataEmission.Section5 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section5 = tempDataDensity.Section5 + 1;
				tempDataSpeed.Section5 = tempDataSpeed.Section5 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard5 = tempDataDensity.Hazard5 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter5 = tempDataDensity.Hazardcounter5 + 1;
				}
				tempDataAccHazard.Section5 = tempDataAccHazard.Section5 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 2500 && saveLi_Hazard->x <= 3000)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section6 = tempDataEmission.Section6 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section6 = tempDataDensity.Section6 + 1;
				tempDataSpeed.Section6 = tempDataSpeed.Section6 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard6 = tempDataDensity.Hazard6 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter6 = tempDataDensity.Hazardcounter6 + 1;
				}
				tempDataAccHazard.Section6 = tempDataAccHazard.Section6 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 3000 && saveLi_Hazard->x <= 3500)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section7 = tempDataEmission.Section7 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section7 = tempDataDensity.Section7 + 1;
				tempDataSpeed.Section7 = tempDataSpeed.Section7 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard7 = tempDataDensity.Hazard7 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter7 = tempDataDensity.Hazardcounter7 + 1;
				}
				tempDataAccHazard.Section7 = tempDataAccHazard.Section7 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 3500 && saveLi_Hazard->x <= 4000)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section8 = tempDataEmission.Section8 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section8 = tempDataDensity.Section8 + 1;
				tempDataSpeed.Section8 = tempDataSpeed.Section8 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard8 = tempDataDensity.Hazard8 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter8 = tempDataDensity.Hazardcounter8 + 1;
				}
				tempDataAccHazard.Section8 = tempDataAccHazard.Section8 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 4000 && saveLi_Hazard->x <= 4500)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section9 = tempDataEmission.Section9 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section9 = tempDataDensity.Section9 + 1;
				tempDataSpeed.Section9 = tempDataSpeed.Section9 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard9 = tempDataDensity.Hazard9 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter9 = tempDataDensity.Hazardcounter9 + 1;
				}
				tempDataAccHazard.Section9 = tempDataAccHazard.Section9 + saveLi_Hazard->Crash_Probability;
			}
			if (saveLi_Hazard->x > 4500 && saveLi_Hazard->x <= 5000)
			{
				Mode_temp = CO2 (saveLi_Hazard->v, saveLi_Hazard->a_update);
				tempDataEmission.Section10 = tempDataEmission.Section10 + ((emission_input[Mode_temp])/10);
				tempDataDensity.Section10 = tempDataDensity.Section10 + 1;
				tempDataSpeed.Section10 = tempDataSpeed.Section10 + saveLi_Hazard->v;
				if (saveLi_Hazard->basehazard != -1000)
				{
					tempDataDensity.Hazard10 = tempDataDensity.Hazard10 + saveLi_Hazard->basehazard;
					tempDataDensity.Hazardcounter10 = tempDataDensity.Hazardcounter10 + 1;
				}
				tempDataAccHazard.Section10 = tempDataAccHazard.Section10 + saveLi_Hazard->Crash_Probability;
			}
		}
		saveLi_Hazard = saveLi_Hazard->next;
	}

	if (check != NULL)
	{
		if (check->lane == 0)
		{
			AggDataHazardD_L0.push_back(tempDataDensity);
			AggDataHazardS_L0.push_back(tempDataSpeed);
			AggDataHazardE_L0.push_back(tempDataEmission);
			AggDataHazardA_L0.push_back(tempDataAccHazard);
		}
		if (check->lane == 1)
		{
			AggDataHazardD_L1.push_back(tempDataDensity);
			AggDataHazardS_L1.push_back(tempDataSpeed);
			AggDataHazardE_L1.push_back(tempDataEmission);
			AggDataHazardA_L1.push_back(tempDataAccHazard);
		}
		if (check->lane == 2)
		{
			AggDataHazardD_L2.push_back(tempDataDensity);
			AggDataHazardS_L2.push_back(tempDataSpeed);
			AggDataHazardE_L2.push_back(tempDataEmission);
			AggDataHazardA_L2.push_back(tempDataAccHazard);
		}
		if (check->lane == 3)
		{
			AggDataHazardD_L3.push_back(tempDataDensity);
			AggDataHazardS_L3.push_back(tempDataSpeed);
			AggDataHazardE_L3.push_back(tempDataEmission);
			AggDataHazardA_L3.push_back(tempDataAccHazard);
		}
	}
	return (0);
}
// Driver's Behavior from a Cognitive Perspective
// CogniSim: A Micro-Simulation Software
//  Requested softwares: 
//	Platform: Microsoft Visual C++ 2005
//	OS:	Microsoft Windows 

// CogniSim.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include <deque>
#include <iostream>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <dos.h>
#include <string.h>
#include <fstream>
#include <string> 
//#include "cognisim.h"
//#include "ControlPanel.h"
//#include "Reaction.h"
//#include "Car_Following.h"
#include "SaveData.h"
#include "Output.h"
//#include "Driving_History.h"
//#include "Wavelet.h"

void main()
{
//**************************DECLARE AND INITIALIZE***************************
	short loadSelect,inputFlag;
	float braking_rate; //Gipps: Variable for correcting for the braking rate in the Gipps Model
	short a,b,c,x,y; //general purpose integer variables
	short debug; //variable to catch infinite lange changing loop
	short flag1,flag2; //general purpose flags
	short riskIsZero = 0; //risk flag. If risk is specified zero, raise flag to implement pure Gipps model
	short *carDeparting; //lookup table for cars in departure time order.
							 //e.g. carDeparting[0] is the array index in the cars
							 //array of the car with the earliest departure time.	
	float f,g,h,i; //general purpose floating point variables
	//data tempData; //temp variable to store data to file
	data2 tempDataDensity; 
	data3 tempDataSpeed;
	data3 tempDataEmission;
	data4 tempDataAccHazard;	
	car * cars = NULL; //dynamically allocated array of N cars
	car * L0 = NULL, * L1 = NULL, * L2 = NULL, * L3 = NULL, * L10 = NULL, * L11 = NULL, * L12 = NULL, * L13 = NULL, * L14 = NULL, * L15 = NULL, * L16 = NULL, * L17 = NULL, * L18 = NULL;
	car * L0_head = NULL, * L1_head = NULL, * L2_head = NULL, * L3_head = NULL, * L10_head = NULL, * L11_head = NULL, * L12_head = NULL, * L13_head = NULL, * L14_head = NULL, * L15_head = NULL, * L16_head = NULL, * L17_head = NULL, * L18_head = NULL;
	car * L0qs = NULL, * L1qs = NULL, * L2qs = NULL, * L3qs = NULL, * L10qs = NULL, * L11qs = NULL, * L12qs = NULL, * L13qs = NULL, * L14qs = NULL, * L15qs = NULL, * L16qs = NULL, * L17qs = NULL, * L18qs = NULL; 
	car * L0qe = NULL, * L1qe = NULL, * L2qe = NULL, * L3qe = NULL, * L10qe = NULL, * L11qe = NULL, * L12qe = NULL, * L13qe = NULL, * L14qe = NULL, * L15qe = NULL, * L16qe = NULL, * L17qe = NULL, * L18qe = NULL;
	
	car * depart; //departure linked list start
	
	car * p, * q, * r, *z, *s, *D1_Side, *D2_Side, *D3_Side, *D_Total, *DRamp_Side, *DRamp_Total; //general purpose car pointers for navigating the linked lists (Modified by Samer Hamdar: 15 September 2008)
	FILE *out;     //output car data to random access file store.tmp
	FILE *inSim;  //input simulation data from .sim file (textmode)
	FILE *outSim; //output simulation data to .sim file (textmode)
	FILE *lane0; //output macroscopic data to lane0.txt file (textmode)
	FILE *lane1; //output macroscopic data to lane1.txt file (textmode)

	FILE *lane10; //output macroscopic data to lane1.txt file (textmode)

	//L0 = L1 = L10 = L0qs = L0qe = L1qs = L1qe = L10qs = L10qe = depart = 0;
	//L0_head = L1_head = L10_head =0;


	insert_output * Insert_Vehicle_temporary;
	lane_changing_data * Lane_Changing_temporary;
	lane_changing_data_RML * Lane_Changing_temporary_RML;

	int PreviousLeaderIDtmp;
	double NumberOfVehicles_Total;
	double NumberOfVehicles_Side1;
	double NumberOfVehicles_Side2;
	double NumberOfVehicles_Side3;
	double DistanceDensityBehind = 250.0;
	double DistanceDensityAhead = 250.0;
	double Distance_Density; //To be Transformed to Vehicles/km.lane
	double PercentLaneChange;


	//For Computing the Hazard with Heterogeneity
	double Hazard_tmp;
	double Survival_tmp;
	double Hazard2_tmp;
	double Survival2_tmp;
	double Lambda_tmp;
	double Teta_tmp;
	double Gamma_tmp;//Test
	double V_tmp = mrand(100);//Test
	double W_tmp;

	int SafetyCriterionBehind1;//1 if safe, 0 otherwise: Left Lane
	int SafetyCriterionAhead1;//1 if safe, 0 otherwise: Left Lane

	int SafetyCriterionBehind2;//1 if safe, 0 otherwise: Right Lane
	int SafetyCriterionAhead2;//1 if safe, 0 otherwise: Right Lane

	int SafetyCriterionBehind3;//1 if safe, 0 otherwise: Ramp
	int SafetyCriterionAhead3;//1 if safe, 0 otherwise: Ramp

	int IncentiveCriterion;//1 if exit episode, 0 otherwise


	struct dataset
	{
		float x; //position
		short l;  //lane
		float v; //velocity
		float h;   //headway
		long t; // current time
	};
	
	struct Trajectory
	{
		//int CarID;	
		vector<dataset> AggDataSet;
	};
	
	vector< Trajectory > TrajectoryData;


	loadSelect = 2;
	srand(time(NULL)); //seed rand() function

	
	SFDValues *VSLInputL0;
	SFDValues *VSLInputL1;
	SFDValues *VSLInputL2;
	SFDValues *VSLInputL3;
	
	double CheckDensity;
	double CheckSpeed; 
	double CheckFlow;

	double DesiredSpeedLimit = DesiredSpeed;

	int EvaluationFlag = 0;
	int VSLFlag = 0;
	int VSLTIMECounter = 0;
	int VSLActivationFlag = 0;


	vector <float> SelectedSpeedLimit;
	
//**************************END DECLARE AND INITIALIZE***************************

//*******************************INTRO*************************************
//*********************************LOAD previously generated input files*************************************

	if(loadSelect == 2)
	{
		// added by ART 07/08/11
		/*AggCars agg_car;
		for (a=0; a<N; a++)
		{
			agg_car.CarID = a;//cars[a].carID;
			AggCarsData.push_back(agg_car);
		}*/
		// end of added by ART 07/08/11


		//char dest[13];
		//getSimFile(1,dest); //find file to load, filename into 'dest'
		FILE *inSim;
//		inSim = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\MACRO0.txt","r");
		inSim = fopen("..\\I-O Files\\MACRO0.txt","r");
		
		if (inSim!=NULL)
		{
			a = read_emission_file(loadSelect);
			a = read_ramp_location(loadSelect);
			a = ramp_metering_location(loadSelect);			
			a = read_path(loadSelect);
			a = read_demand(loadSelect);
			a = read_interarrivaltime(loadSelect);
			a=loadSim2(inSim,&cars,&depart);

			if (a==-1)
			{
				fclose(inSim);
				return;
			}
			//carDeparting = (int *) malloc(sizeof(int) * N);

			if (a==0)
			{
				fclose(inSim);
				return;
			}
			else
			{
				fclose(inSim);
			}
		}
	
		int ss;
		AggCars agg_car;
		for (ss=0; ss<N; ss++)
		{
			agg_car.CarID = ss;//cars[a].carID;
			AggCarsData.push_back(agg_car);
		}
		// end of added by ART 07/08/11

		// added by ART 05/29/12
		vector<Wavelet_Information> wavelet_initialization;
		vector<vector<Wavelet_Information>> wavelet_history;
		wavelet_history_time WaveletHistoryTime;
		
		for (a = 0; a < N; a++)
		{				
			wavelet_history.push_back (wavelet_initialization);
		}
		
		WaveletHistoryTime.Wavelet_History = wavelet_history;
		for (int b = 0; b < int(T/100); b++)
		{			
			Wavelet_History_Time.push_back (WaveletHistoryTime);
		}



		//******************************INIT CARS*********************************************

		//declare array to indicate departure time usage

		cars = (car *) malloc(sizeof(car) * N);
		carDeparting = (short *) malloc(sizeof(int) * N);

		int driver_number;
		int vehicle_type;
		//Initialize car parameters for all cars
		for (c=0; c<N; c++)
		{ 
			float Randomtemp = mrand(rand_seed_platoon) * 100;//rand() % 100;  // 50
			if (Randomtemp > (CV_Market_Penetration_Rate + Autonomous_Market_Penetration_Rate))
				vehicle_type = 0;//0 is not a connected vehicle nor an autonomous vehicle
			else if (Randomtemp <= (CV_Market_Penetration_Rate + Autonomous_Market_Penetration_Rate) && Randomtemp > Autonomous_Market_Penetration_Rate)
				vehicle_type = 1;//1 is a connectd vehicle
			else
				vehicle_type = 2;//2 is an autonomous vehicle


			if (vehicle_type == 0)
			{
	
				driver_number = (int)((mrand(40))*8);
				if (driver_number == 8)
					driver_number = 7;

				//driver_number = 0;


				 cars[c].Connectivity_Status = 0;// 0 means not connected, 1 means connected, 2 means autonomous
				 //Vehicle Specific Parameters
				 cars[c].s = normal(mean_s, std_s, range_s,1,2);
				 cars[c].acc = normal(meanacc, stdacc, rangeacc,3,4);
				 cars[c].decc = -8;//normal(meandecc, stddecc, rangedecc,5,6);//*10;

				 /*a = mrand(7) * 100; //a is betwee 1 and 100%
				 if (a <= (int)percentVd_1) //two normal distributions, 40% and 60% probabilities.
					 cars[c].Vd = normal(meanVd_1, stdVd_1, rangeVd_1,8,9); //this distrib. 40% of the time
				 else
					 cars[c].Vd = normal(meanVd_2, stdVd_2, rangeVd_2,10,11); //this distrib. 60% of the time

				 */
				 cars[c].Vd = InitialSpeed;//DriversSpeedLimit(InitialSpeed,Compliance);
				 cars[c].overspeedingrate = normal(OverspeedingRate,0.05,OverspeedingRate,14,15);

				 cars[c].Crash_Probability = 0;

				 f = normal(meanRt, stdRt, rangeRt,12,13);
			 
				 cars[c].Rt = int(Drivers[driver_number].Rt);
				 cars[c].react = (int)(cars[c].Rt * 10 + 1); //ten times the reaction time in seconds rounded to the nearest 0.1 seconds
				 cars[c].Rt = cars[c].react;//Drivers[driver_number].Rt;

				 f = normal(meanLCT,stdLCT,rangeLCT,14,15);
				 //round using integers to nearest multiple of 0.1
				 cars[c].LCT = (int)((f * 10 + 0.5)/5); //ten times the generated lane changing time in seconds
				 cars[c].LCL = 0;
			 

				//PT Mode Parameters

				 cars[c].Gamma = Drivers[driver_number].Gamma;
			 
	
				cars[c].Wm = Drivers[driver_number].Wm;
			 
			 

				cars[c].Wc = Drivers[driver_number].Wc;
			 
			

				cars[c].Tmax = Drivers[driver_number].Tmax;
			 
			
	
				cars[c].Alpha = Drivers[driver_number].Alpha;
			


				cars[c].Beta = Drivers[driver_number].Beta;
			 

				cars[c].Tcorr = Drivers[driver_number].Tcorr;
				cars[c].So = normal(mean_So, std_So, range_So,30,31);

				cars[c].Wp = 1.0;
				cars[c].DeltaW = ( 1.0 - cars[c].Wm );
				cars[c].Sigma = ( 0.5 * ( 1.0 - cars[c].Gamma) );

				//Simulation Related Parameters
				cars[c].departure=0;
			
				cars[c].x = 0.0;
				cars[c].v = cars[c].Vd; //initial velocity
				cars[c].l=0;
				cars[c].headway = -99999.0;
				//cars[c].carID = c+1;
				cars[c].carID = c; 
				cars[c].crash = 0;
				cars[c].crashtimecounter = 0;
			 			
				//NB: xtra initial measures of effectivess
				cars[c].DV = -9999.0;
				cars[c].Spacing = cars[c].headway;
				cars[c].a_update = 0.2;
				cars[c].Yt = 0.0;
				cars[c].Yt_previous = 0.0;

				//Miscellanious Parameters
				if ( ((meanrisk<0.05)&&(meanrisk>-0.05)) && ((stdrisk<0.05)&&(stdrisk>-0.05)) && ((rangerisk<0.05)&&(rangerisk>-0.05)) )
				{
					cars[c].risk = 0;
				}
				else
				{
				cars[c].risk = normal(meanrisk, stdrisk, rangerisk,32,33);
				}


				if ( ((meanLCcrash<0.05)&&(meanLCcrash>-0.05)) && ((stdLCcrash<0.05)&&(stdLCcrash>-0.05)) && ((rangeLCcrash<0.05)&&(rangeLCcrash>-0.05)) )
				{
				f = 0.0;
				}
				else
				{
					f = normal(meanLCcrash,stdLCcrash,rangeLCcrash,34,35);
				}

				cars[c].LCcrash = (int)f;
				if (cars[c].LCcrash < 0)
					cars[c].LCcrash = 0;
				if (cars[c].LCcrash > 100)
					cars[c].LCcrash = 100;

				cars[c].Duration = 0;

				cars[c].previous = cars[c].next = NULL;
			 
				cars[c].lanechangingflag = 0;
				cars[c].basehazard = 0;

				f = mrand(50); 
				if (f > NonComplianceRate)
					cars[c].ComplianceValue = 0;
				else
					cars[c].ComplianceValue = 1;




				car *temp_car;
				temp_car = (car *) malloc(sizeof(car));
				deque<car *> temp_history;
				for (int vehicle_counter = 0; vehicle_counter < cars[c].react ; vehicle_counter++)
					temp_history.push_back(temp_car);
				Vehicle_History.push_back(temp_history);

				free(temp_car);
			 
				//vehicle_history[c][0]->acc = 0;
				//vehicle_history[c][1]->acc = 1;
				//vehicle_history[c][2]->acc = 2;



				//vehicle_history[0].pop_front();
				//vehicle_history[0].push_back(temp_car);

			}

			if (vehicle_type == 1)
			{

				driver_number = (int)((mrand(40))*8);
				if (driver_number == 8)
					driver_number = 7;


				 cars[c].Connectivity_Status = 1;// 0 means not connected, 1 means connected, 2 means autonomous
				 //Vehicle Specific Parameters
				 cars[c].s = normal(mean_s, std_s, range_s,1,2);
				 cars[c].acc = normal(meanacc, stdacc, rangeacc,3,4);
				 cars[c].decc = -8;//normal(meandecc, stddecc, rangedecc,5,6);//*10;

				 /*a = mrand(7) * 100; //a is betwee 1 and 100%
				 if (a <= (int)percentVd_1) //two normal distributions, 40% and 60% probabilities.
					 cars[c].Vd = normal(meanVd_1, stdVd_1, rangeVd_1,8,9); //this distrib. 40% of the time
				 else
					 cars[c].Vd = normal(meanVd_2, stdVd_2, rangeVd_2,10,11); //this distrib. 60% of the time

				 */
				 cars[c].Vd = InitialSpeed;//DriversSpeedLimit(InitialSpeed,Compliance);
				 cars[c].overspeedingrate = normal(OverspeedingRate,0.05,OverspeedingRate,14,15);

				 cars[c].Crash_Probability = 0;

				 f = normal(meanRt, stdRt, rangeRt,12,13);
			 
				 cars[c].Rt = int(Drivers[driver_number].Rt) * 0.5;
				 cars[c].react = (int)(cars[c].Rt * 10 + 1); //ten times the reaction time in seconds rounded to the nearest 0.1 seconds
				 cars[c].Rt = cars[c].react;//Drivers[driver_number].Rt;

				 f = normal(meanLCT,stdLCT,rangeLCT,14,15);
				 //round using integers to nearest multiple of 0.1
				 cars[c].LCT = (int)(f * 10 + 0.5); //ten times the generated lane changing time in seconds
				 cars[c].LCL = 0;
			 

				//PT Mode Parameters


				 cars[c].Deltan = Drivers[driver_number].Deltan;

				cars[c].Tn = Drivers[driver_number].Tn;
			 

				cars[c].Son = Drivers[driver_number].Son;
			 
	
				cars[c].an = Drivers[driver_number].an;
			 
	
				cars[c].bn = Drivers[driver_number].bn;
			


				cars[c].Von = Drivers[driver_number].Von;
			 
			
				cars[c].So = normal(mean_So, std_So, range_So,30,31);

				//Simulation Related Parameters
				cars[c].departure=0;
			
				cars[c].x = 0.0;
				cars[c].v = cars[c].Vd; //initial velocity
				cars[c].l=0;
				cars[c].headway = -99999.0;
				//cars[c].carID = c+1;
				cars[c].carID = c;  
				cars[c].crash = 0;
				cars[c].crashtimecounter = 0;
			 			
				//NB: xtra initial measures of effectivess
				cars[c].DV = -9999.0;
				cars[c].Spacing = cars[c].headway;
				cars[c].a_update = 0.2;
				cars[c].Yt = 0.0;
				cars[c].Yt_previous = 0.0;

				//Miscellanious Parameters
				if ( ((meanrisk<0.05)&&(meanrisk>-0.05)) && ((stdrisk<0.05)&&(stdrisk>-0.05)) && ((rangerisk<0.05)&&(rangerisk>-0.05)) )
				{
					cars[c].risk = 0;
				}
				else
				{
				cars[c].risk = normal(meanrisk, stdrisk, rangerisk,32,33);
				}


				if ( ((meanLCcrash<0.05)&&(meanLCcrash>-0.05)) && ((stdLCcrash<0.05)&&(stdLCcrash>-0.05)) && ((rangeLCcrash<0.05)&&(rangeLCcrash>-0.05)) )
				{
				f = 0.0;
				}
				else
				{
					f = normal(meanLCcrash,stdLCcrash,rangeLCcrash,34,35);
				}

				cars[c].LCcrash = (int)f;
				if (cars[c].LCcrash < 0)
					cars[c].LCcrash = 0;
				if (cars[c].LCcrash > 100)
					cars[c].LCcrash = 100;

				cars[c].Duration = 0;

				cars[c].previous = cars[c].next = NULL;
			 
				cars[c].lanechangingflag = 0;
				cars[c].basehazard = 0;

				f = mrand(50); 
				if (f > NonComplianceRate)
					cars[c].ComplianceValue = 0;
				else
					cars[c].ComplianceValue = 1;

 
 
				car *temp_car;
				temp_car = (car *) malloc(sizeof(car));
				deque<car *> temp_history;
				for (int vehicle_counter = 0; vehicle_counter < cars[c].react ; vehicle_counter++)
					temp_history.push_back(temp_car);
				Vehicle_History.push_back(temp_history);

				free(temp_car);
			 
				//vehicle_history[c][0]->acc = 0;
				//vehicle_history[c][1]->acc = 1;
				//vehicle_history[c][2]->acc = 2;



				//vehicle_history[0].pop_front();
				//vehicle_history[0].push_back(temp_car);

 
			}

 
			if (vehicle_type == 2)
			{
				 cars[c].Connectivity_Status = 2;// 0 means not connected, 1 means connected, 2 means autonomous
				 //Vehicle Specific Parameters
				 cars[c].s = 1;//normal(mean_s, std_s, range_s,1,2);
				 cars[c].acc = 4;//normal(meanacc, stdacc, rangeacc,3,4);
				 cars[c].decc = -8;//normal(meandecc, stddecc, rangedecc,5,6);//*10;

				 /*a = mrand(7) * 100; //a is betwee 1 and 100%
				 if (a <= (int)percentVd_1) //two normal distributions, 40% and 60% probabilities.
					 cars[c].Vd = normal(meanVd_1, stdVd_1, rangeVd_1,8,9); //this distrib. 40% of the time
				 else
					 cars[c].Vd = normal(meanVd_2, stdVd_2, rangeVd_2,10,11); //this distrib. 60% of the time

				 */
				 cars[c].Vd = InitialSpeed;//DriversSpeedLimit(InitialSpeed,Compliance);
				 cars[c].overspeedingrate = 0;//normal(OverspeedingRate,0.05,OverspeedingRate,14,15);

				 cars[c].Crash_Probability = 0;
			 
				 cars[c].Rt = 0.05;
				 cars[c].react = (int)(cars[c].Rt * 10 + 1); //ten times the reaction time in seconds rounded to the nearest 0.1 seconds
				 cars[c].Rt = cars[c].react;//Drivers[driver_number].Rt;

				 f = normal(meanLCT,stdLCT,rangeLCT,14,15);
				 //round using integers to nearest multiple of 0.1
				 cars[c].LCT = (int)(f * 10 + 0.5); //ten times the generated lane changing time in seconds
				 cars[c].LCL = 0;
			 
				 cars[c].K = 0.3;
			     cars[c].KPrime = 0.3;
				 cars[c].Ka = 1.0;
				 cars[c].Kv = 0.58;
				 cars[c].Kd = 0.1;
			     
				//Simulation Related Parameters
				cars[c].departure=0;
			
				cars[c].x = 0.0;
				cars[c].v = cars[c].Vd; //initial velocity
				cars[c].l=0;
				cars[c].headway = -99999.0;
				//cars[c].carID = c+1;
				cars[c].carID = c; //this was the above line, Changed in Jan 13, 2012 by ART
				cars[c].crash = 0;
				cars[c].crashtimecounter = 0;
			 			
				//NB: xtra initial measures of effectivess
				cars[c].DV = -9999.0;
				cars[c].Spacing = cars[c].headway;
				cars[c].a_update = 0.2;
				cars[c].Yt = 0.0;
				cars[c].Yt_previous = 0.0;

				//Miscellanious Parameters
				if ( ((meanrisk<0.05)&&(meanrisk>-0.05)) && ((stdrisk<0.05)&&(stdrisk>-0.05)) && ((rangerisk<0.05)&&(rangerisk>-0.05)) )
				{
					cars[c].risk = 0;
				}
				else
				{
				cars[c].risk = normal(meanrisk, stdrisk, rangerisk,32,33);
				}


				if ( ((meanLCcrash<0.05)&&(meanLCcrash>-0.05)) && ((stdLCcrash<0.05)&&(stdLCcrash>-0.05)) && ((rangeLCcrash<0.05)&&(rangeLCcrash>-0.05)) )
				{
				f = 0.0;
				}
				else
				{
					f = normal(meanLCcrash,stdLCcrash,rangeLCcrash,34,35);
				}

				cars[c].LCcrash = (int)f;
				if (cars[c].LCcrash < 0)
					cars[c].LCcrash = 0;
				if (cars[c].LCcrash > 100)
					cars[c].LCcrash = 100;

				cars[c].Duration = 0;

				cars[c].previous = cars[c].next = NULL;
			 
				cars[c].lanechangingflag = 0;
				cars[c].basehazard = 0;

				f = mrand(50); 
				if (f > NonComplianceRate)
					cars[c].ComplianceValue = 0;
				else
					cars[c].ComplianceValue = 1;

 
 
				car *temp_car;
				temp_car = (car *) malloc(sizeof(car));
				deque<car *> temp_history;
				for (int vehicle_counter = 0; vehicle_counter < cars[c].react ; vehicle_counter++)
					temp_history.push_back(temp_car);
				Vehicle_History.push_back(temp_history);

				free(temp_car);
				 
			}
 
		}
		//******************************END INIT CARS*********************************************
		//*****************GENERATE DEPARTURE TIMES*****************************

		
 

		double a1=0;
		double a2=0;
		int rempcounter=0;

		vector<float> departure_time;
		for (int i = 0; i < 50; i++)
			departure_time.push_back(0);

		int TotalDemand1 = 0;
		for(int i = 0; i < demand[0].size() - 1; i++)
			TotalDemand1 = TotalDemand1 + demand[0][i];
		int TotalDemand2 = 0;
		for(int i = 0; i < demand[1].size() - 1; i++)
			TotalDemand2 = TotalDemand2 + demand[1][i];
		short case_number = -1;
		float temp_demand = 0;
		short counter_ramp = 0;

		
		for (c=0; c<N; c++)
		{			
			if (departure_time[0] < 36000)
			{
				a1 = mrand(37);
				case_number = -1;

				if (a1 > 0 && a1 < (float(demand[0][0])/TotalDemand1))
				{					
					case_number = 0;
				}
				else
				{
					temp_demand = 0;
					for(int i = 1; i < demand[0].size() - 1; i++)
					{
						temp_demand = temp_demand + float(demand[0][i-1]) / TotalDemand1;
						if (a1 > temp_demand && a1 < (temp_demand + float(demand[0][i]) / TotalDemand1))
						{
							case_number = i;
							break;
						}
					}
				}
				
				
				if (path_information[case_number][0] == 1)
				{
					a2 = mrand(40);
					
					if (a2 <= 0.25)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 0;
						cars[c].lane_prev = 0;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.25 && a2 <= 0.50)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 1;
						cars[c].lane_prev = 1;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.50 && a2 <= 0.75)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 2;
						cars[c].lane_prev = 2;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.75)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 3;
						cars[c].lane_prev = 3;
						cars[c].changelaneinstance = 0;
					}					
				}
				else
				{
					departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
					cars[c].departure = departure_time[path_information[case_number][0]];
					Path.push_back(path_information[case_number]);
					cars[c].lane = path_information[case_number][0];
					cars[c].lane_prev = path_information[case_number][0];
					cars[c].changelaneinstance = 0;
					counter_ramp = 0;
					while (path_information[case_number][0] != ramp_information[counter_ramp].Ramp_Number)
						counter_ramp ++;
					cars[c].x = ramp_information[counter_ramp].Ramp_StartPoint;
				}
			}
			else
			{
				a1 = mrand(37);
				case_number = -1;

				if (a1 > 0 && a1 < (float(demand[0][0])/TotalDemand1))
				{					
					case_number = 0;
				}
				else
				{
					temp_demand = 0;
					for(int i = 1; i < demand[0].size() - 1; i++)
					{
						temp_demand = temp_demand + (float(demand[0][i-1])/TotalDemand1);
						if (a1 > temp_demand && a1 < (temp_demand + (float(demand[0][i])/TotalDemand1)))
						{
							case_number = i;
							break;
						}
					}
				}

				if (case_number == 33)
					getch();
				
				
				if (path_information[case_number][0] == 1)
				{
					a2 = mrand(37);
					
					if (a2 < 0.25)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 0;
						cars[c].lane_prev = 0;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.25 && a2 < 0.50)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 1;
						cars[c].lane_prev = 1;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.50 && a2 < 0.75)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 2;
						cars[c].lane_prev = 2;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.75)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 3;
						cars[c].lane_prev = 3;
						cars[c].changelaneinstance = 0;
					}					
				}
				else
				{
					departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
					cars[c].departure = departure_time[path_information[case_number][0]];
					Path.push_back(path_information[case_number]);
					cars[c].lane = path_information[case_number][0];
					cars[c].lane_prev = path_information[case_number][0];
					cars[c].changelaneinstance = 0;
					counter_ramp = 0;
					while (path_information[case_number][0] != ramp_information[counter_ramp].Ramp_Number)
						counter_ramp ++;
					cars[c].x = ramp_information[counter_ramp].Ramp_StartPoint;
				}
			}						
			
			
	

			// Insert car into departure list
			// depart is the first departure car, which means its departure time is the least one.
			  if (c == 0) //departure list empty
			  {
				  depart = &cars[0];
			  }
			  else
			  {
				  p = depart;
				  a = cars[c].departure;

				  if (a < p->departure) //car has earliest departure time so far
				  {                     //=> insert at beginning of list
						cars[c].next = depart;
						depart->previous = &cars[c];
						depart = &cars[c];
				  }
				  else
				  {
						//find place to insert car in departure list
						while ((p->next!=NULL) && ((p->next->departure) < a))
							p = p->next;

						//insert car
						cars[c].previous = p;
						cars[c].next = p->next;
						if (p->next!=NULL)
							p->next->previous = &cars[c];
						p->next = &cars[c];
					}
			}
		}

		//outSim = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\carinfo.txt","w");
		//if (saveSim(outSim,cars) )
		//{
		//	fclose(outSim);
			//return;
		//}


		double test1;
		test1=Hazard_ParameterB;
	
		// TEST 07/08/14	
		FILE *st1;
		char filename_dep[9999];
//		st1 = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Departure.txt", "w");
		sprintf(filename_dep, "..\\I-O Files_%d\\Departure.txt",InitializationSeed);
		// cout << filename_dep;
		st1 = fopen(filename_dep, "w");
//		sprintf("..\\I-O Files\\Departure.txt"
//		st1 = fopen("..\\I-O Files\\Departure.txt", "w");
	
		if(st1 !=NULL)
		{

					fprintf(st1,"%s	%s	%s	%s	%s	%s	%s\n" ,"Count",
					"carID",
					"Connectivity_Status",
					"location",
					"departure_time",
					"lane",
					"lanechangingflag");
		
		for (int i=0; i<N; i++)
		{
				fprintf(st1, "%d %d %d %d %d	%d %d\n",i,
						cars[i].carID,
						cars[i].Connectivity_Status,
						cars[i].x,
						cars[i].departure,
						cars[i].lane,
						cars[i].lanechangingflag,
						cars[i].overspeedingrate,
						cars[i].ComplianceValue);
			}
		};
		fprintf(st1, "%d\n", rempcounter);
		
		fclose(st1);
		// end of TEST 07/08/14 

		// TEST 04/16/15	
		FILE *st2;
		char filename_dep2[9999];
//		st2 = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Departure_depart.txt", "w");
		sprintf(filename_dep2, "..\\I-O Files_%d\\Departure_depart.txt",InitializationSeed);
		// cout << filename_dep2;
		st2 = fopen(filename_dep2, "w");
//		st2 = fopen("..\\I-O Files\\Departure_depart.txt", "w");
		
		car *departtest;
		departtest = depart;
		if(st2 !=NULL)
		{
			while (departtest!= NULL)
			{
				fprintf(st2, "%d %d	%d	%d\n",
						departtest->carID,
						departtest->departure,
						departtest->lane,
						departtest->Connectivity_Status);
			departtest=departtest->next;
			}
			
		};
		fprintf(st2, "%d\n", rempcounter);
		
		fclose(st2);
		// end of TEST 04/16/15

		//*****************END GENERATE DEPARTURE TIMES*****************************
		
	}


//*****************************END LOAD*************************************

//************************GENERATE NEW SIMULATION***********************************

	if (loadSelect==1) //generate a new simulation from scratch
	{
		
		//char dest[13];
		//getSimFile(1,dest); //find file to load, filename into 'dest'
		FILE *inSim;
//		inSim = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\MACRO0.txt","r");
		inSim = fopen("..\\I-O Files\\MACRO0.txt","r");
		
		if (inSim!=NULL)
		{
			a = read_emission_file(loadSelect);
			a = read_ramp_location(loadSelect);
			a = ramp_metering_location(loadSelect);			
			a = read_path(loadSelect);
			a = read_demand(loadSelect);
			a = read_interarrivaltime(loadSelect);
			a=loadSim1(inSim,&cars,&depart);
			if (a==-1)
			{
				fclose(inSim);
				return;
			}
			fclose(inSim);
		}

 
		AggCars agg_car;
		for (a=0; a<N; a++)
		{
			agg_car.CarID = a;//cars[a].carID;
			AggCarsData.push_back(agg_car);
		}
 
		vector<Wavelet_Information> wavelet_initialization;
		vector<vector<Wavelet_Information>> wavelet_history;
		wavelet_history_time WaveletHistoryTime;
		
		for (a = 0; a < N; a++)
		{				
			wavelet_history.push_back (wavelet_initialization);
		}
		
		WaveletHistoryTime.Wavelet_History = wavelet_history;
		for (int b = 0; b < int(T/100); b++)
		{			
			Wavelet_History_Time.push_back (WaveletHistoryTime);
		}
 
	
		//******************************INIT CARS*********************************************

		//declare array to indicate departure time usage

		cars = (car *) malloc(sizeof(car) * N);
		carDeparting = (short *) malloc(sizeof(int) * N);

		//Initialize car parameters for all cars
		for (c=0; c<N; c++)
		{ 
 
			 cars[c].Connectivity_Status = 0;
			 //Vehicle Specific Parameters
			 cars[c].s = normal(mean_s, std_s, range_s,1,2);
			 cars[c].acc = normal(meanacc, stdacc, rangeacc,3,4);
			 cars[c].decc = normal(meandecc, stddecc, rangedecc,5,6);//*10;

			 /*a = mrand(7) * 100; //a is betwee 1 and 100%
			 if (a <= (int)percentVd_1) //two normal distributions, 40% and 60% probabilities.
				 cars[c].Vd = normal(meanVd_1, stdVd_1, rangeVd_1,8,9); //this distrib. 40% of the time
			 else
				 cars[c].Vd = normal(meanVd_2, stdVd_2, rangeVd_2,10,11); //this distrib. 60% of the time

			 */
			 cars[c].Vd = InitialSpeed;//DriversSpeedLimit(InitialSpeed,Compliance);
			 cars[c].overspeedingrate = normal(OverspeedingRate,0.05,OverspeedingRate,14,15);

			 cars[c].Crash_Probability = 0;


			 f = normal(meanRt, stdRt, rangeRt,12,13);

			 cars[c].react = (int)(f * 10 + 0.5); //ten times the reaction time in seconds rounded to the nearest 0.1 seconds

			 cars[c].Rt = cars[c].react / 10.0;

			 f = normal(meanLCT,stdLCT,rangeLCT,14,15);
			 //round using integers to nearest multiple of 0.1
			 cars[c].LCT = (int)(f * 10 + 0.5); //ten times the generated lane changing time in seconds
			 cars[c].LCL = 0;
			 

			//PT Mode Parameters

			 cars[c].Gamma = normal(mean_Gamma, std_Gamma, range_Gamma,16,17);
			 cars[c].Wm = normal(mean_Wm, std_Wm, range_Wm,18,19);
			 cars[c].Wc = normal(mean_Wc, std_Wc, range_Wc,20,21);
			 cars[c].Tmax = normal(mean_Tmax, std_Tmax, range_Tmax,22,23);
			 cars[c].Alpha = normal(mean_Alpha, std_Alpha, range_Alpha,24,25);
			 cars[c].Beta = normal(mean_Beta, std_Beta, range_Beta,26,27);
			 cars[c].Tcorr = normal(mean_Tcorr, std_Tcorr, range_Tcorr,f,29);
			 cars[c].So = normal(mean_So, std_So, range_So,30,31);

			 cars[c].Wp = 1.0;
			 cars[c].DeltaW = ( 1.0 - cars[c].Wm );
			 cars[c].Sigma = ( 0.5 * ( 1.0 - cars[c].Gamma) );

			 //Simulation Related Parameters
			 cars[c].departure=0;
			
			 cars[c].x = 0.0;
			 cars[c].v = cars[c].Vd; //initial velocity
			 cars[c].l=0;
			 cars[c].headway = -99999.0;
			 //cars[c].carID = c+1;
			 cars[c].carID = c;  
			 cars[c].crash = 0;
			 cars[c].crashtimecounter_ramp = 0;
			 cars[c].crashtimecounter = 0;
			 			
			 //NB: xtra initial measures of effectivess
			 cars[c].DV = -9999.0;
			 cars[c].Spacing = cars[c].headway;
			 cars[c].a_update = 0.2;
			 cars[c].Yt = 0.0;
			 cars[c].Yt_previous = 0.0;

			 //Miscellanious Parameters
			 if ( ((meanrisk<0.05)&&(meanrisk>-0.05)) && ((stdrisk<0.05)&&(stdrisk>-0.05)) && ((rangerisk<0.05)&&(rangerisk>-0.05)) )
			 {
				 cars[c].risk = 0;
			 }
			 else
			 {
				cars[c].risk = normal(meanrisk, stdrisk, rangerisk,32,33);
			 }


			 if ( ((meanLCcrash<0.05)&&(meanLCcrash>-0.05)) && ((stdLCcrash<0.05)&&(stdLCcrash>-0.05)) && ((rangeLCcrash<0.05)&&(rangeLCcrash>-0.05)) )
			 {
				f = 0.0;
			 }
			 else
			 {
				 f = normal(meanLCcrash,stdLCcrash,rangeLCcrash,34,35);
			 }

			 cars[c].LCcrash = (int)f;
			 if (cars[c].LCcrash < 0)
				 cars[c].LCcrash = 0;
			 if (cars[c].LCcrash > 100)
				 cars[c].LCcrash = 100;

			 cars[c].Duration = 0;

			 cars[c].previous = cars[c].next = NULL;
			 
			 cars[c].lanechangingflag = 0;
			 cars[c].basehazard = 0;

			 f = mrand(50); 
			 if (f > NonComplianceRate)
				 cars[c].ComplianceValue = 0;
			 else
				 cars[c].ComplianceValue = 1;

 
				car *temp_car;
				temp_car = (car *) malloc(sizeof(car));
				deque<car *> temp_history;
				for (int vehicle_counter = 0; vehicle_counter < cars[c].react ; vehicle_counter++)
					temp_history.push_back(temp_car);
				Vehicle_History.push_back(temp_history);

				free(temp_car);
	 
		}
	//******************************END INIT CARS*********************************************
	
	//*****************GENERATE DEPARTURE TIMES*****************************

		
 

		double a1=0;
		double a2=0;
		int rempcounter=0;

		vector<float> departure_time;
		for (int i = 0; i < 50; i++)
			departure_time.push_back(0);

		int TotalDemand1 = 0;
		for(int i = 0; i < demand[0].size() - 1; i++)
			TotalDemand1 = TotalDemand1 + demand[0][i];
		int TotalDemand2 = 0;
		for(int i = 0; i < demand[1].size() - 1; i++)
			TotalDemand2 = TotalDemand2 + demand[1][i];
		short case_number = -1;
		float temp_demand = 0;
		short counter_ramp = 0;

		
		for (c=0; c<N; c++)
		{			
			if (departure_time[0] < 36000)
			{
				a1 = mrand(37);
				case_number = -1;

				if (a1 > 0 && a1 < (float(demand[0][0])/TotalDemand1))
				{					
					case_number = 0;
				}
				else
				{
					temp_demand = 0;
					for(int i = 1; i < demand[0].size() - 1; i++)
					{
						temp_demand = temp_demand + float(demand[0][i-1]) / TotalDemand1;
						if (a1 > temp_demand && a1 < (temp_demand + float(demand[0][i]) / TotalDemand1))
						{
							case_number = i;
							break;
						}
					}
				}
				
				
				if (path_information[case_number][0] == 1)
				{
					a2 = mrand(40);
					
					if (a2 <= 0.25)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 0;
						cars[c].lane_prev = 0;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.25 && a2 <= 0.50)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 1;
						cars[c].lane_prev = 1;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.50 && a2 <= 0.75)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 2;
						cars[c].lane_prev = 2;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.75)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 3;
						cars[c].lane_prev = 3;
						cars[c].changelaneinstance = 0;
					}					
				}
				else
				{
					departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
					cars[c].departure = departure_time[path_information[case_number][0]];
					Path.push_back(path_information[case_number]);
					cars[c].lane = path_information[case_number][0];
					cars[c].lane_prev = path_information[case_number][0];
					cars[c].changelaneinstance = 0;
					counter_ramp = 0;
					while (path_information[case_number][0] != ramp_information[counter_ramp].Ramp_Number)
						counter_ramp ++;
					cars[c].x = ramp_information[counter_ramp].Ramp_StartPoint;
				}
			}
			else
			{
				a1 = mrand(37);
				case_number = -1;

				if (a1 > 0 && a1 < (float(demand[0][0])/TotalDemand1))
				{					
					case_number = 0;
				}
				else
				{
					temp_demand = 0;
					for(int i = 1; i < demand[0].size() - 1; i++)
					{
						temp_demand = temp_demand + (float(demand[0][i-1])/TotalDemand1);
						if (a1 > temp_demand && a1 < (temp_demand + (float(demand[0][i])/TotalDemand1)))
						{
							case_number = i;
							break;
						}
					}
				}

				if (case_number == 33)
					getch();
				
				
				if (path_information[case_number][0] == 1)
				{
					a2 = mrand(37);
					
					if (a2 < 0.25)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 0;
						cars[c].lane_prev = 0;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.25 && a2 < 0.50)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 1;
						cars[c].lane_prev = 1;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.50 && a2 < 0.75)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 2;
						cars[c].lane_prev = 2;
						cars[c].changelaneinstance = 0;
					}
					if (a2 > 0.75)
					{
						departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
						cars[c].departure = departure_time[path_information[case_number][0]];
						Path.push_back(path_information[case_number]);
						cars[c].lane = 3;
						cars[c].lane_prev = 3;
						cars[c].changelaneinstance = 0;
					}					
				}
				else
				{
					departure_time[path_information[case_number][0]] = departure_time[path_information[case_number][0]] + (long)(expon(MEAN_INTERARRIVAL_TIME[0][case_number],InitializationSeed));					
					cars[c].departure = departure_time[path_information[case_number][0]];
					Path.push_back(path_information[case_number]);
					cars[c].lane = path_information[case_number][0];
					cars[c].lane_prev = path_information[case_number][0];
					cars[c].changelaneinstance = 0;
					counter_ramp = 0;
					while (path_information[case_number][0] != ramp_information[counter_ramp].Ramp_Number)
						counter_ramp ++;
					cars[c].x = ramp_information[counter_ramp].Ramp_StartPoint;
				}
			}						
			
			
	

			// Insert car into departure list
			// depart is the first departure car, which means its departure time is the least one.
			  if (c == 0) //departure list empty
			  {
				  depart = &cars[0];
			  }
			  else
			  {
				  p = depart;
				  a = cars[c].departure;

				  if (a < p->departure) //car has earliest departure time so far
				  {                     //=> insert at beginning of list
						cars[c].next = depart;
						depart->previous = &cars[c];
						depart = &cars[c];
				  }
				  else
				  {
						//find place to insert car in departure list
						while ((p->next!=NULL) && ((p->next->departure) < a))
							p = p->next;

						//insert car
						cars[c].previous = p;
						cars[c].next = p->next;
						if (p->next!=NULL)
							p->next->previous = &cars[c];
						p->next = &cars[c];
					}
			}
		}

//		outSim = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\carinfo.txt","w");
		char filename_car[9999];
		sprintf(filename_car, "..\\I-O Files_%d\\carinfo.txt",InitializationSeed);
		// cout << filename_car;
		outSim = fopen(filename_car, "w");

//		outSim = fopen("..\\I-O Files\\carinfo.txt","w");
		if (saveSim(outSim,cars) )
		{
			fclose(outSim);
			//return;
		}


		double test1;
		test1=Hazard_ParameterB;
	
	// TEST 07/08/14	
	FILE *st1;
	char filename_st1[9999];
//	st1 = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Departure.txt", "w");
	sprintf(filename_st1, "..\\I-O Files_%d\\Departure.txt",InitializationSeed);
	// cout << filename_st1;
	st1 = fopen(filename_st1, "w");
	
	if(st1 !=NULL)
	{

		fprintf(st1,"%s	%s	%s	%s	%s	%s	%s\n" ,"Count",
					"carID",
					"Connectivity_Status",
					"location",
					"departure_time",
					"lane",
					"lanechangingflag");
		
		for (int i=0; i<N; i++)
		{
			fprintf(st1, "%d %d %d %d %d	%d %d\n",i,
					cars[i].carID,
					cars[i].Connectivity_Status,
					cars[i].x,
					cars[i].departure,
					cars[i].lane,
					cars[i].lanechangingflag);
		}
	};
	fprintf(st1, "%d\n", rempcounter);
	
	fclose(st1);
	// end of TEST 07/08/14

	//*****************END GENERATE DEPARTURE TIMES*****************************
}
//************************END GENERATE NEW SIMULATION***********************************


//*****************************BEGIN SIMULATION******************************

	//Option Kept Open
	//if ( ((meanrisk<0.05)&&(meanrisk>-0.05)) && ((stdrisk<0.05)&&(stdrisk>-0.05)) && ((rangerisk<0.05)&&(rangerisk>-0.05)) )
	//	riskIsZero = 1;
	//else
	//	riskIsZero = 0;
	//End of Option Kept Open

	L0qe = NULL;
	L0qs = NULL;
	L1qe = NULL;
	L1qs = NULL;
	L0 = NULL;
	L0_head = NULL;
	L1 = NULL;
	L1_head = NULL;

 
	L10qe = NULL;
	L10qs = NULL;
	L10 = NULL;
	L10_head = NULL;
	Average_Starting_Speed.resize(20);
	Starting_Speed_Counter.resize(20);
 

	int a1=0;
	int departure_flag=0;

	for (TIME = 0; TIME < T; TIME++)
	{
		
		//if (TIME == 69)
		//{
		//	cout<<TIME<<endl;
		//	getch();			
		//}
		if (TIME % 1000 == 0)
			cout<<TIME<<endl;

			
		/*if (L1_head != NULL)
		{
		car *test1;
		test1 = L1;
		while (test1->next != NULL)
		{
			test1 = test1->next;
		}
		cout<<test1->x<<"		"<<test1->carID<<"		"<<L1_head->x<<"		"<<L1_head->carID<<"		"<<TIME<<endl;
		//getch();
		}*/
				
		int a2=0;
		//****************FEED NEW CARS ENTERING******************
		for (c=0; (depart!=NULL) && ((depart->departure == TIME) || (departure_flag == 1)) ; c++) //(possibility of 2 cars departing)==> editted to solve this by ART 07/09/11
		{
			if (depart->next != NULL)
			{	
				p = depart;

				double test1;
				test1=Hazard_ParameterB;

				depart = depart->next; //remove from departure list
				
 
				if (depart->departure == depart->previous->departure)
				{
					departure_flag = 1;
				}
				else
				{
					departure_flag = 0;
				};

		 

		 
				if (depart!=NULL)
				{
					depart->previous = NULL;
				}

				//insert car to beginning of its lane

				if (p->lane == 0)
				{
					Insert_Vehicle_temporary = Insert_Vehicle(p, L0, L0_head, L0qe, L0qs);
					L0 = Insert_Vehicle_temporary->lane_tail;
					L0_head = Insert_Vehicle_temporary->lane_head;
					L0qe = Insert_Vehicle_temporary->queue_end;
					L0qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 1)
				{
					Insert_Vehicle_temporary = Insert_Vehicle(p, L1, L1_head, L1qe, L1qs);
					L1 = Insert_Vehicle_temporary->lane_tail;
					L1_head = Insert_Vehicle_temporary->lane_head;
					L1qe = Insert_Vehicle_temporary->queue_end;
					L1qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 2)
				{
					Insert_Vehicle_temporary = Insert_Vehicle(p, L2, L2_head, L2qe, L2qs);
					L2 = Insert_Vehicle_temporary->lane_tail;
					L2_head = Insert_Vehicle_temporary->lane_head;
					L2qe = Insert_Vehicle_temporary->queue_end;
					L2qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 3)
				{
					Insert_Vehicle_temporary = Insert_Vehicle(p, L3, L3_head, L3qe, L3qs);
					L3 = Insert_Vehicle_temporary->lane_tail;
					L3_head = Insert_Vehicle_temporary->lane_head;
					L3qe = Insert_Vehicle_temporary->queue_end;
					L3qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 10)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L10, L10_head, L10qe, L10qs, p->lane);
					L10 = Insert_Vehicle_temporary->lane_tail;
					L10_head = Insert_Vehicle_temporary->lane_head;
					L10qe = Insert_Vehicle_temporary->queue_end;
					L10qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 11)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L11, L11_head, L11qe, L11qs, p->lane);
					L11 = Insert_Vehicle_temporary->lane_tail;
					L11_head = Insert_Vehicle_temporary->lane_head;
					L11qe = Insert_Vehicle_temporary->queue_end;
					L11qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 12)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L12, L12_head, L12qe, L12qs, p->lane);
					L12 = Insert_Vehicle_temporary->lane_tail;
					L12_head = Insert_Vehicle_temporary->lane_head;
					L12qe = Insert_Vehicle_temporary->queue_end;
					L12qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 13)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L13, L13_head, L13qe, L13qs, p->lane);
					L13 = Insert_Vehicle_temporary->lane_tail;
					L13_head = Insert_Vehicle_temporary->lane_head;
					L13qe = Insert_Vehicle_temporary->queue_end;
					L13qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 14)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L14, L14_head, L14qe, L14qs, p->lane);
					L14 = Insert_Vehicle_temporary->lane_tail;
					L14_head = Insert_Vehicle_temporary->lane_head;
					L14qe = Insert_Vehicle_temporary->queue_end;
					L14qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 15)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L15, L15_head, L15qe, L15qs, p->lane);
					L15 = Insert_Vehicle_temporary->lane_tail;
					L15_head = Insert_Vehicle_temporary->lane_head;
					L15qe = Insert_Vehicle_temporary->queue_end;
					L15qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 16)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L16, L16_head, L16qe, L16qs, p->lane);
					L16 = Insert_Vehicle_temporary->lane_tail;
					L16_head = Insert_Vehicle_temporary->lane_head;
					L16qe = Insert_Vehicle_temporary->queue_end;
					L16qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 17)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L17, L17_head, L17qe, L17qs, p->lane);
					L17 = Insert_Vehicle_temporary->lane_tail;
					L17_head = Insert_Vehicle_temporary->lane_head;
					L17qe = Insert_Vehicle_temporary->queue_end;
					L17qs = Insert_Vehicle_temporary->queue_start;
				}

				if (p->lane == 18)
				{
					Insert_Vehicle_temporary = Insert_Vehicle_Ramp(p, L18, L18_head, L18qe, L18qs, p->lane);
					L18 = Insert_Vehicle_temporary->lane_tail;
					L18_head = Insert_Vehicle_temporary->lane_head;
					L18qe = Insert_Vehicle_temporary->queue_end;
					L18qs = Insert_Vehicle_temporary->queue_start;
				}				
			}
			else
			{
				depart = NULL;
			}
		}

		//****************END FEED NEW CARS ENTERING***************

		//check queue for lane 0;
		if (L0qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_II(L0, L0_head, L0qe, L0qs);
			L0 = Insert_Vehicle_temporary->lane_tail;
			L0_head = Insert_Vehicle_temporary->lane_head;
			L0qe = Insert_Vehicle_temporary->queue_end;
			L0qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L1qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_II(L1, L1_head, L1qe, L1qs);
			L1 = Insert_Vehicle_temporary->lane_tail;
			L1_head = Insert_Vehicle_temporary->lane_head;
			L1qe = Insert_Vehicle_temporary->queue_end;
			L1qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L2qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_II(L2, L2_head, L2qe, L2qs);
			L2 = Insert_Vehicle_temporary->lane_tail;
			L2_head = Insert_Vehicle_temporary->lane_head;
			L2qe = Insert_Vehicle_temporary->queue_end;
			L2qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L3qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_II(L3, L3_head, L3qe, L3qs);
			L3 = Insert_Vehicle_temporary->lane_tail;
			L3_head = Insert_Vehicle_temporary->lane_head;
			L3qe = Insert_Vehicle_temporary->queue_end;
			L3qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L10qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L10, L10_head, L10qe, L10qs, L10qe->lane);
			L10 = Insert_Vehicle_temporary->lane_tail;
			L10_head = Insert_Vehicle_temporary->lane_head;
			L10qe = Insert_Vehicle_temporary->queue_end;
			L10qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L11qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L11, L11_head, L11qe, L11qs, L11qe->lane);
			L11 = Insert_Vehicle_temporary->lane_tail;
			L11_head = Insert_Vehicle_temporary->lane_head;
			L11qe = Insert_Vehicle_temporary->queue_end;
			L11qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L12qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L12, L12_head, L12qe, L12qs, L12qe->lane);
			L12 = Insert_Vehicle_temporary->lane_tail;
			L12_head = Insert_Vehicle_temporary->lane_head;
			L12qe = Insert_Vehicle_temporary->queue_end;
			L12qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L13qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L13, L13_head, L13qe, L13qs, L13qe->lane);
			L13 = Insert_Vehicle_temporary->lane_tail;
			L13_head = Insert_Vehicle_temporary->lane_head;
			L13qe = Insert_Vehicle_temporary->queue_end;
			L13qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L14qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L14, L14_head, L14qe, L14qs, L14qe->lane);
			L14 = Insert_Vehicle_temporary->lane_tail;
			L14_head = Insert_Vehicle_temporary->lane_head;
			L14qe = Insert_Vehicle_temporary->queue_end;
			L14qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L15qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L15, L15_head, L15qe, L15qs, L15qe->lane);
			L15 = Insert_Vehicle_temporary->lane_tail;
			L15_head = Insert_Vehicle_temporary->lane_head;
			L15qe = Insert_Vehicle_temporary->queue_end;
			L15qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L16qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L16, L16_head, L16qe, L16qs, L16qe->lane);
			L16 = Insert_Vehicle_temporary->lane_tail;
			L16_head = Insert_Vehicle_temporary->lane_head;
			L16qe = Insert_Vehicle_temporary->queue_end;
			L16qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L17qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L17, L17_head, L17qe, L17qs, L17qe->lane);
			L17 = Insert_Vehicle_temporary->lane_tail;
			L17_head = Insert_Vehicle_temporary->lane_head;
			L17qe = Insert_Vehicle_temporary->queue_end;
			L17qs = Insert_Vehicle_temporary->queue_start;
		}

		if (L18qe!=NULL)
		{
			Insert_Vehicle_temporary = Insert_Vehicle_Ramp_II(L18, L18_head, L18qe, L18qs, L18qe->lane);
			L18 = Insert_Vehicle_temporary->lane_tail;
			L18_head = Insert_Vehicle_temporary->lane_head;
			L18qe = Insert_Vehicle_temporary->queue_end;
			L18qs = Insert_Vehicle_temporary->queue_start;
		}		
		//******************Variable Speed Limit Implementation***********************
		
		//if (EvaluationFlag == 1)
		//{
			//EvaluationFlag = 0;
			//VSLFlag == 1;

			//VSLInputL0 = NULL;
			//VSLInputL1 = NULL;
			//CheckDensity = 0;
			//CheckSpeed = 0;
			//CheckFlow = 0;

			//VSLInputL0 = SFDL0 (AggDataHazardD_L0,AggDataHazardS_L0, TIME-1);
			//VSLInputL1 = SFDL1 (AggDataHazardD_L1,AggDataHazardS_L1, TIME-1);

			//CheckDensity = VSLInputL0->SDFDensity + VSLInputL1->SDFDensity;
			//CheckSpeed = ((VSLInputL0->SDFSpeed * VSLInputL0->SDFDensity) + (VSLInputL1->SDFSpeed * VSLInputL1->SDFDensity))/CheckDensity;
			//CheckFlow = CheckSpeed * CheckDensity;

			//if (CheckFlow > TargetFlow)///3.6)
			//{
			//	if (CheckSpeed > (DesiredSpeed - 5))
			//		DesiredSpeedLimit = DesiredSpeed;
			//	if (CheckSpeed > (DesiredSpeed - 10) && CheckSpeed <= (DesiredSpeed - 5))
			//		DesiredSpeedLimit = DesiredSpeed - 5;
			//	if (CheckSpeed <= (DesiredSpeed - 10))
			//		DesiredSpeedLimit = DesiredSpeed - 10;
			//}
			//else
			//{
			//	if (CheckDensity <= TargetDensity)
			//		DesiredSpeedLimit = DesiredSpeed;
			//	else
			//	{
			//	if (CheckSpeed > (DesiredSpeed - 5))
			//		DesiredSpeedLimit = DesiredSpeed;
			//	if (CheckSpeed > (DesiredSpeed - 10) && CheckSpeed <= (DesiredSpeed - 5))
			//		DesiredSpeedLimit = DesiredSpeed - 5;
			//	if (CheckSpeed <= (DesiredSpeed - 10))
			//		DesiredSpeedLimit = DesiredSpeed - 10;
			//	}
			//}

			if (L0 != NULL)
			{
				L0 = SpeedUpdate(L0, VSLStartLocation, VSLEndLocation, VSLActivationFlag, DesiredSpeedLimit);
				p = L0;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L0_head = p;
			}

			if (L1 != NULL)
			{
				L1 = SpeedUpdate(L1, VSLStartLocation, VSLEndLocation, VSLActivationFlag, DesiredSpeedLimit);
				p = L1;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L1_head = p;
			}

			if (L2 != NULL)
			{
				L2 = SpeedUpdate(L2, VSLStartLocation, VSLEndLocation, VSLActivationFlag, DesiredSpeedLimit);
				p = L2;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L2_head = p;
			}

			if (L3 != NULL)
			{
				L3 = SpeedUpdate(L3, VSLStartLocation, VSLEndLocation, VSLActivationFlag, DesiredSpeedLimit);
				p = L3;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L3_head = p;
			}

			if (L10 != NULL)
			{
				L10 = SpeedUpdate_Ramp(L10);
				p = L10;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L10_head = p;
			}

			if (L11 != NULL)
			{
				L11 = SpeedUpdate_Ramp(L11);				
				p = L11;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L11_head = p;
			}

			if (L12 != NULL)
			{
				L12 = SpeedUpdate_Ramp(L12);				
				p = L12;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L12_head = p;
			}

			if (L13 != NULL)
			{
				L13 = SpeedUpdate_Ramp(L13);
				p = L13;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L13_head = p;
			}

			if (L14 != NULL)
			{
				L14 = SpeedUpdate_Ramp(L14);
				p = L14;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L14_head = p;
			}

			if (L15 != NULL)
			{
				L15 = SpeedUpdate_Ramp(L15);
				p = L15;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L15_head = p;
			}

			if (L16 != NULL)
			{
				L16 = SpeedUpdate_Ramp(L16);
				p = L16;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L16_head = p;
			}

			if (L17 != NULL)
			{
				L17 = SpeedUpdate_Ramp(L17);
				p = L17;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L17_head = p;
			}

			if (L18 != NULL)
			{
				L18 = SpeedUpdate_Ramp(L18);
				p = L18;
				if (p != NULL)
				{
					while (p->next != NULL)
					p = p->next;
				}
				L18_head = p;
			}
			
			
			//if (DesiredSpeedLimit != DesiredSpeed)
			//	VSLActivationFlag = 1;
		//}
		//else
		//{
		//	if (L0 != NULL)
		//	{
		//	p = L0;
		//	while (p != NULL)
		//	{
		//		if (p->x >= VSLStartLocation)
		//		{			
		//			p->Vd =(p->Compliance * OverspeedingRate * DesiredSpeedLimit) + DesiredSpeedLimit;//(p->overspeedingrate * DesiredSpeedLimit) + DesiredSpeedLimit;// DriversSpeedLimit(DesiredSpeedLimit, Compliance);
		//		}
		//		p = p->next;
		//	}
		//	//p = NULL;
		//	}
			
		//	if (L1 != NULL)
		//	{
		//	p = L1;
		//	while (p != NULL)
		//	{
		//		if (p->x >= VSLStartLocation)
		//		{			
		//			p->Vd = (p->Compliance * OverspeedingRate * DesiredSpeedLimit) + DesiredSpeedLimit;//(p->overspeedingrate * DesiredSpeedLimit) + DesiredSpeedLimit;//DriversSpeedLimit(DesiredSpeedLimit, Compliance);
		//		}
		//		p = p->next;
		//	}
		//	//p = NULL;
		//	}
		//}

						
		if (VSLActivationFlag == 1)
			VSLTIMECounter = VSLTIMECounter + 1;

		//if (VSLActivationFlag == 0 || VSLTIMECounter >= 9000)
		//{
			//if (TIME % 300 == 0 && TIME != 0)
			//{
		//		EvaluationFlag = VSLActivation;// if 1 then VSL active, 0 otherwise
		//		VSLTIMECounter = 0;
		//		VSLActivationFlag = 0;
			//}
		//}
				
		/*if (DesiredSpeedLimit != 20)
		{
			cout<<DesiredSpeedLimit<<"		"<<TIME<<endl;
			getch();
		}*/
			SelectedSpeedLimit.push_back(DesiredSpeedLimit);

		//******************End of Variable Speed Limit Implementation****************

		
		
		//******************CAR FOLLOWING DECISION**************************
			for (int c_L = 0; c_L < 20; c_L++)
			{
				Average_Starting_Speed[c_L] = 0;
				Starting_Speed_Counter[c_L] = 0;
			}
					
			L0 = car_following (L0_head);
			L1 = car_following (L1_head);
			L2 = car_following (L2_head);			
			L3 = car_following (L3_head);			
			L10 = car_following (L10_head);
			L11 = car_following (L11_head);
			L12 = car_following (L12_head);
			L13 = car_following (L13_head);
			L14 = car_following (L14_head);
			L15 = car_following (L15_head);
			L16 = car_following (L16_head);
			L17 = car_following (L17_head);
			L18 = car_following (L18_head);

			for (int c_L = 0; c_L < 20; c_L++)
			{
				if (Starting_Speed_Counter[c_L] != 0)
					Average_Starting_Speed[c_L] = Average_Starting_Speed[c_L] / Starting_Speed_Counter[c_L];
				else
					Average_Starting_Speed[c_L] = 0;
			}

			
		//****************END CAR FOLLOWING DECISION************************

		/*if (L0 != NULL)
			cout<<"			"<<L0->carID<<endl;
		else
			cout<<"			"<<"None"<<endl;
		getch();*/


		//***********************HEADWAY/CRASH DETECTION**************************
			L0_head = CrashDetection (L0);
			p = L0_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L0 = p;
			}
			else
				L0 = p;

			L1_head = CrashDetection (L1);
			p = L1_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L1 = p;
			}
			else
				L1 = p;

			L2_head = CrashDetection (L2);
			p = L2_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L2 = p;
			}
			else
				L2 = p;

			L3_head = CrashDetection (L3);
			p = L3_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L3 = p;
			}
			else
				L3 = p;

			L10_head = CrashDetection (L10);
			p = L10_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L10 = p;
			}
			else
				L10 = p;

			L11_head = CrashDetection (L11);
			p = L11_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L11 = p;
			}
			else
				L11 = p;

			L12_head = CrashDetection (L12);
			p = L12_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L12 = p;
			}
			else
				L12 = p;

			L13_head = CrashDetection (L13);
			p = L13_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L13 = p;
			}
			else
				L13 = p;

			L14_head = CrashDetection (L14);
			p = L14_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L14 = p;
			}
			else
				L14 = p;

			L15_head = CrashDetection (L15);
			p = L15_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L15 = p;
			}
			else
				L15 = p;

			L16_head = CrashDetection (L16);
			p = L16_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L16 = p;
			}
			else
				L16 = p;

			L17_head = CrashDetection (L17);
			p = L17_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L17 = p;
			}
			else
				L17 = p;

			L18_head = CrashDetection (L18);
			p = L18_head;
			if (p != NULL)
			{
				while (p->previous != NULL)
					p = p->previous;
				L18 = p;
			}
			else
				L18 = p;
			
		//*******************END OF HEADWAY/CRASH DETECTION***********************
		



		//******************OCCUPANCY CALCUlATION****************************
		if (RampMeteringActivation == 1)
		{
			if (int(TIME/600) > current_interval)
			{
				current_interval = int (TIME / 600);			

				for (int i = 0; i < metering_info.size() - 1; i++)
				{				
					metering_rate [i] = metering_rate [i] + 70 * (20 - ((occupancy_info[i] * 100) / 60));
					if (metering_rate[i] <= 300)
						metering_rate[i] = 300;
					metering_interarrivaltime[i] = 36000 / metering_rate[i];
					occupancy_info[i] = 0;
				}			
			}
		}
		else
		{
			for (int i = 0; i < metering_info.size() - 1; i++)
				metering_interarrivaltime[i] = 1;
		}
		
		a = occupancy (L0);
		a = occupancy (L1);
		a = occupancy (L2);
		a = occupancy (L3);

		//******************END oF OCCUPANCY CALCUlATION****************************
		


				
		//******************LANE CHANGING DECISION****************************
			s = NULL;
			Lane_Changing_temporary = Lane_Changing(L0, s, L1, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L0 = Lane_Changing_temporary->Ltmp1;
				L0_head = Lane_Changing_temporary->Ltmp1_head;
				L1 = Lane_Changing_temporary->Ltmp3;
				L1_head = Lane_Changing_temporary->Ltmp3_head;
			

 			Lane_Changing_temporary = Lane_Changing(L1, L0, L2, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L1 = Lane_Changing_temporary->Ltmp1;
				L1_head = Lane_Changing_temporary->Ltmp1_head;
				L0 = Lane_Changing_temporary->Ltmp2;
				L0_head = Lane_Changing_temporary->Ltmp2_head;
				L2 = Lane_Changing_temporary->Ltmp3;
				L2_head = Lane_Changing_temporary->Ltmp3_head;
			

			Lane_Changing_temporary = Lane_Changing(L2, L1, L3, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L2 = Lane_Changing_temporary->Ltmp1;
				L2_head = Lane_Changing_temporary->Ltmp1_head;
				L1 = Lane_Changing_temporary->Ltmp2;
				L1_head = Lane_Changing_temporary->Ltmp2_head;
				L3 = Lane_Changing_temporary->Ltmp3;
				L3_head = Lane_Changing_temporary->Ltmp3_head;
				

			Lane_Changing_temporary_RML = Lane_Changing_RML(L3, L2, L10, L11, L12, L13, L14, L15, L16, L17, L18, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L3 = Lane_Changing_temporary_RML->LtmpRML1;
				L3_head = Lane_Changing_temporary_RML->LtmpRML1_head;
				L2 = Lane_Changing_temporary_RML->LtmpRML2;
				L2_head = Lane_Changing_temporary_RML->LtmpRML2_head;
				L10 = Lane_Changing_temporary_RML->LtmpRML10;
				L10_head = Lane_Changing_temporary_RML->LtmpRML10_head;
				L11 = Lane_Changing_temporary_RML->LtmpRML11;
				L11_head = Lane_Changing_temporary_RML->LtmpRML11_head;
				L12 = Lane_Changing_temporary_RML->LtmpRML12;
				L12_head = Lane_Changing_temporary_RML->LtmpRML12_head;
				L13 = Lane_Changing_temporary_RML->LtmpRML13;
				L13_head = Lane_Changing_temporary_RML->LtmpRML13_head;
				L14 = Lane_Changing_temporary_RML->LtmpRML14;
				L14_head = Lane_Changing_temporary_RML->LtmpRML14_head;
				L15 = Lane_Changing_temporary_RML->LtmpRML15;
				L15_head = Lane_Changing_temporary_RML->LtmpRML15_head;
				L16 = Lane_Changing_temporary_RML->LtmpRML16;
				L16_head = Lane_Changing_temporary_RML->LtmpRML16_head;
				L17 = Lane_Changing_temporary_RML->LtmpRML17;
				L17_head = Lane_Changing_temporary_RML->LtmpRML17_head;
				L18 = Lane_Changing_temporary_RML->LtmpRML18;
				L18_head = Lane_Changing_temporary_RML->LtmpRML18_head;				

				//if (L12 != NULL)
				//	cout<<L12->lane<<endl;
				
			
			Lane_Changing_temporary = Lane_Changing(L10, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L10 = Lane_Changing_temporary->Ltmp1;
				L10_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;

			Lane_Changing_temporary = Lane_Changing(L11, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L11 = Lane_Changing_temporary->Ltmp1;
				L11_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;

			Lane_Changing_temporary = Lane_Changing(L12, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L12 = Lane_Changing_temporary->Ltmp1;
				L12_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;

			Lane_Changing_temporary = Lane_Changing(L13, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L13 = Lane_Changing_temporary->Ltmp1;
				L13_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;

			Lane_Changing_temporary = Lane_Changing(L14, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L14 = Lane_Changing_temporary->Ltmp1;
				L14_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;

			Lane_Changing_temporary = Lane_Changing(L15, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L15 = Lane_Changing_temporary->Ltmp1;
				L15_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;

			Lane_Changing_temporary = Lane_Changing(L16, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L16 = Lane_Changing_temporary->Ltmp1;
				L16_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;

			Lane_Changing_temporary = Lane_Changing(L17, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L17 = Lane_Changing_temporary->Ltmp1;
				L17_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;

			Lane_Changing_temporary = Lane_Changing(L18, L3, s, DesiredSpeedLimit, VSLStartLocation, VSLEndLocation);
				L18 = Lane_Changing_temporary->Ltmp1;
				L18_head = Lane_Changing_temporary->Ltmp1_head;
				L3 = Lane_Changing_temporary->Ltmp2;
				L3_head = Lane_Changing_temporary->Ltmp2_head;				
			
		//******************END LANE CHANGING DECISION**************************



		//***********************REMOVE EXITING CARS**************************
		short counter_ramp = 0;
				
		p = L0;
		while(p != NULL)
		{
			if (p->x >= L)
			{
			  if (p->previous != NULL)
			  {
				  p->previous->next = NULL;
				  L0_head = p->previous;
			  }
			  else
			  {
				L0 = NULL;
				L0_head = NULL;
			  }
			  break;
			}
			else
			  p=p->next;
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L1;
		while(p != NULL)
		{
			if (p->x >= L)
			{
			  if (p->previous != NULL)
			  {
				  p->previous->next = NULL;
				  L1_head = p->previous;
			  }
			  else
			  {
				L1 = NULL;
				L1_head = NULL;
			  }
			  break;
			}
			else
			  p=p->next;
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L2;
		while(p != NULL)
		{
			if (p->x >= L)
			{
			  if (p->previous != NULL)
			  {
				  p->previous->next = NULL;
				  L2_head = p->previous;
			  }
			  else
			  {
				L2 = NULL;
				L2_head = NULL;
			  }
			  break;
			}
			else
			  p=p->next;
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L3;
		while(p != NULL)
		{
			if (p->x >= L)
			{
			  if (p->previous != NULL)
			  {
				  p->previous->next = NULL;
				  L3_head = p->previous;
			  }
			  else
			  { 
				L3 = NULL;
				L3_head = NULL;
			  }
			  break;
			}
			else
			  p=p->next;
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L10;
		while(p != NULL)
		{
			counter_ramp = 0;
			a = Path[p->carID].size() - 1;
			if (p->lane == Path[p->carID][a])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L10_head = p->previous;
				  }
				  else
				  {
					L10 = NULL;
					L10_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L11;
		while(p != NULL)
		{
			counter_ramp = 0;
			if (p->lane == Path[p->carID][Path[p->carID].size() - 1])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L11_head = p->previous;
				  }
				  else
				  {
					L11 = NULL;
					L11_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L12;
		while(p != NULL)
		{
			counter_ramp = 0;
			if (p->lane == Path[p->carID][Path[p->carID].size() - 1])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L12_head = p->previous;
				  }
				  else
				  {
					L12 = NULL;
					L12_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L13;
		while(p != NULL)
		{
			counter_ramp = 0;
			if (p->lane == Path[p->carID][Path[p->carID].size() - 1])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L13_head = p->previous;
				  }
				  else
				  {
					L13 = NULL;
					L13_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L14;
		while(p != NULL)
		{
			counter_ramp = 0;
			if (p->lane == Path[p->carID][Path[p->carID].size() - 1])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L14_head = p->previous;
				  }
				  else
				  {
					L14 = NULL;
					L14_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L15;
		while(p != NULL)
		{
			counter_ramp = 0;
			if (p->lane == Path[p->carID][Path[p->carID].size() - 1])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L15_head = p->previous;
				  }
				  else
				  {
					L15 = NULL;
					L15_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L16;
		while(p != NULL)
		{
			counter_ramp = 0;
			if (p->lane == Path[p->carID][Path[p->carID].size() - 1])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L16_head = p->previous;
				  }
				  else
				  {
					L16 = NULL;
					L16_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L17;
		while(p != NULL)
		{
			counter_ramp = 0;
			if (p->lane == Path[p->carID][Path[p->carID].size() - 1])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L17_head = p->previous;
				  }
				  else
				  {
					L17 = NULL;
					L17_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		p = L18;
		while(p != NULL)
		{
			counter_ramp = 0;
			if (p->lane == Path[p->carID][Path[p->carID].size() - 1])
			{
				while (p->lane != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;
				
				if (p->x >= ramp_information[counter_ramp].Ramp_EndPoint)
				{
				  if (p->previous != NULL)
				  {
					  p->previous->next = NULL;
					  L18_head = p->previous;
				  }
				  else
				  {
					L18 = NULL;
					L18_head = NULL;
				  }
				  break;
				}
				else
				  p=p->next;
			}
			else
			{			
				p=p->next;
			}
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		
		//***********************END REMOVE EXITING CARS**************************

		//************************SAVE DATA*********************************
		wavelet_pushback_counter.clear();
		
		a = save_data_mainstream(L0, TIME, VSLActivation, VSLActivationFlag, VSLTIMECounter);
		a = save_data_mainstream(L1, TIME, VSLActivation, VSLActivationFlag, VSLTIMECounter);
		a = save_data_mainstream(L2, TIME, VSLActivation, VSLActivationFlag, VSLTIMECounter);
		a = save_data_mainstream(L3, TIME, VSLActivation, VSLActivationFlag, VSLTIMECounter);
		a = save_data_ramp(L10, TIME);
		a = save_data_ramp(L11, TIME);
		a = save_data_ramp(L12, TIME);
		a = save_data_ramp(L13, TIME);
		a = save_data_ramp(L14, TIME);
		a = save_data_ramp(L15, TIME);
		a = save_data_ramp(L16, TIME);
		a = save_data_ramp(L17, TIME);
		a = save_data_ramp(L18, TIME);		
		//----------------------------------Mesh Data---------------------------------------
		//car *saveL0_High_Hazard;
		//car *saveL1_High_Hazard;
		//car *saveL2_High_Hazard;
		//car *saveL3_High_Hazard;
		saveL0_High_Hazard = NULL;
		saveL1_High_Hazard = NULL;
		saveL2_High_Hazard = NULL;
		saveL3_High_Hazard = NULL;
		saveL0_High_Hazard = L0;
		saveL1_High_Hazard = L1;
		saveL2_High_Hazard = L2;
		saveL3_High_Hazard = L3;

		memset (tempMesh.Density, 0, sizeof (tempMesh.Density));
		memset (tempMesh.Hazard, 0, sizeof (tempMesh.Hazard));
		memset (tempMesh.Hazardcounter, 0, sizeof (tempMesh.Hazardcounter));
		memset (tempMesh.Speed, 0, sizeof (tempMesh.Speed));
		
		while (saveL0_High_Hazard != NULL)
		{
			if (saveL0_High_Hazard->x > 0.0 && saveL0_High_Hazard->x_prev <L)
			{				
				for (int Mesh_Counter = 0; Mesh_Counter < (int)(L / 20); Mesh_Counter++)
				{
					if (saveL0_High_Hazard->x >= (Mesh_Counter * 20) && saveL0_High_Hazard->x < ((Mesh_Counter + 1) * 20))
					{
						tempMesh.Density[Mesh_Counter] = tempMesh.Density[Mesh_Counter] + 1;
						tempMesh.Speed[Mesh_Counter] = tempMesh.Speed[Mesh_Counter] + saveL0_High_Hazard->v;
						if (saveL0_High_Hazard->basehazard != -1000)
						{
							tempMesh.Hazard[Mesh_Counter] = tempMesh.Hazard[Mesh_Counter] + saveL0_High_Hazard->basehazard;
							tempMesh.Hazardcounter[Mesh_Counter] = tempMesh.Hazardcounter[Mesh_Counter] + 1;
						}
					}
				}
			}
			saveL0_High_Hazard = saveL0_High_Hazard->next;
		}
		mesh_data_L0.push_back(tempMesh);

		
		while (saveL1_High_Hazard != NULL)
		{
			if (saveL1_High_Hazard->x > 0.0 && saveL1_High_Hazard->x_prev <L)
			{				
				for (int Mesh_Counter = 0; Mesh_Counter < (int)(L / 20); Mesh_Counter++)
				{
					if (saveL1_High_Hazard->x >= (Mesh_Counter * 20) && saveL1_High_Hazard->x < ((Mesh_Counter + 1) * 20))
					{
						tempMesh.Density[Mesh_Counter] = tempMesh.Density[Mesh_Counter] + 1;
						tempMesh.Speed[Mesh_Counter] = tempMesh.Speed[Mesh_Counter] + saveL1_High_Hazard->v;
						if (saveL1_High_Hazard->basehazard != -1000)
						{
							tempMesh.Hazard[Mesh_Counter] = tempMesh.Hazard[Mesh_Counter] + saveL1_High_Hazard->basehazard;
							tempMesh.Hazardcounter[Mesh_Counter] = tempMesh.Hazardcounter[Mesh_Counter] + 1;
						}
					}
				}
			}
			saveL1_High_Hazard = saveL1_High_Hazard->next;
		}
		mesh_data_L1.push_back(tempMesh);


		while (saveL2_High_Hazard != NULL)
		{
			if (saveL2_High_Hazard->x > 0.0 && saveL2_High_Hazard->x_prev <L)
			{				
				for (int Mesh_Counter = 0; Mesh_Counter < (int)(L / 20); Mesh_Counter++)
				{
					if (saveL2_High_Hazard->x >= (Mesh_Counter * 20) && saveL2_High_Hazard->x < ((Mesh_Counter + 1) * 20))
					{
						tempMesh.Density[Mesh_Counter] = tempMesh.Density[Mesh_Counter] + 1;
						tempMesh.Speed[Mesh_Counter] = tempMesh.Speed[Mesh_Counter] + saveL2_High_Hazard->v;
						if (saveL2_High_Hazard->basehazard != -1000)
						{
							tempMesh.Hazard[Mesh_Counter] = tempMesh.Hazard[Mesh_Counter] + saveL2_High_Hazard->basehazard;
							tempMesh.Hazardcounter[Mesh_Counter] = tempMesh.Hazardcounter[Mesh_Counter] + 1;
						}
					}
				}
			}
			saveL2_High_Hazard = saveL2_High_Hazard->next;
		}
		mesh_data_L2.push_back(tempMesh);


		while (saveL3_High_Hazard != NULL)
		{
			if (saveL3_High_Hazard->x > 0.0 && saveL3_High_Hazard->x_prev <L)
			{				
				for (int Mesh_Counter = 0; Mesh_Counter < (int)(L / 20); Mesh_Counter++)
				{
					if (saveL3_High_Hazard->x >= (Mesh_Counter * 20) && saveL3_High_Hazard->x < ((Mesh_Counter + 1) * 20))
					{
						tempMesh.Density[Mesh_Counter] = tempMesh.Density[Mesh_Counter] + 1;
						tempMesh.Speed[Mesh_Counter] = tempMesh.Speed[Mesh_Counter] + saveL3_High_Hazard->v;
						if (saveL3_High_Hazard->basehazard != -1000)
						{
							tempMesh.Hazard[Mesh_Counter] = tempMesh.Hazard[Mesh_Counter] + saveL3_High_Hazard->basehazard;
							tempMesh.Hazardcounter[Mesh_Counter] = tempMesh.Hazardcounter[Mesh_Counter] + 1;
						}
					}
				}
			}
			saveL3_High_Hazard = saveL3_High_Hazard->next;
		}
		mesh_data_L3.push_back(tempMesh);
		//----------------------------------------------------------------------------------

		//----------------------------- Wavelet Identifier----------------------------------	

		if (TIME > 20 && TIME % 100 == 0 && VSLActivation == 1 && (VSLActivationFlag == 0 || VSLTIMECounter >= 6000))//Wavelet_History_Time[TIME].Wavelet_History.size() != 0)
		{
			for (int car_wavelet = 0; car_wavelet < wavelet_pushback_counter.size(); car_wavelet++)
			//for (int car_wavelet= 0; car_wavelet < wavelet_pushback_counter; car_wavelet++)//Wavelet_History_Time[TIME].Wavelet_History.size(); car_wavelet++)
			{			
				Energy1 = Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][0].Energy_value;
				Energy2 = Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][1].Energy_value;
				Energy3 = Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][2].Energy_value;
				Energy4 = Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][3].Energy_value;
				Energy5 = Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][4].Energy_value;

				Evaluate = Evaluation(Energy1, Energy2, Energy3, Energy4, Energy4);

				if (Evaluate == 1)//Energy < 600)
				{
					Car_Wavelet_History[wavelet_pushback_counter[car_wavelet]] = 1;//Car_Wavelet_History[wavelet_pushback_counter[car_wavelet]] + 1;
				}
				else
				{
					Car_Wavelet_History[wavelet_pushback_counter[car_wavelet]] = 0;
				}
			}

			//memset (Car_Wavelet_Segment_Counter, 0, sizeof (Car_Wavelet_Segment_Counter));

			for (int car_wavelet = 0; car_wavelet < wavelet_pushback_counter.size(); car_wavelet++)
			{
				Shockwave_counter[wavelet_pushback_counter[car_wavelet]] = 0;
				if (Car_Wavelet_History[wavelet_pushback_counter[car_wavelet]] == 1)
				{
					for (int car_wavelet_prime = 0; car_wavelet_prime < wavelet_pushback_counter.size(); car_wavelet_prime++)
					{
						if (abs((int) (Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][2].Location - Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet_prime]][2].Location)) < 50)
							Shockwave_counter[wavelet_pushback_counter[car_wavelet]] = Shockwave_counter[wavelet_pushback_counter[car_wavelet]]+ 1;
					}
				}
			}
				
				
				//for (int Mesh_Counter = 0; Mesh_Counter < (int)(L / 20); Mesh_Counter++)
				//{
				//	if (Wavelet_History_Time[TIME].Wavelet_History[wavelet_pushback_counter[car_wavelet]][0].Location >= (Mesh_Counter * 20) && Wavelet_History_Time[TIME].Wavelet_History[wavelet_pushback_counter[car_wavelet]][0].Location < ((Mesh_Counter + 1) * 20) && Car_Wavelet_History[wavelet_pushback_counter[car_wavelet]] = 1)
				//	{
				//		Car_Wavelet_Segment_Counter[Mesh_Counter] = Car_Wavelet_Segment_Counter[Mesh_Counter] + 1;
				//	}
				//}
			//}

			int Minimum = 0;//10000;
			for (int car_wavelet = wavelet_pushback_counter.size() - 1; car_wavelet > 0; car_wavelet += -1)
			{
				if (Shockwave_counter[wavelet_pushback_counter[car_wavelet]] > 0 && Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][2].Location > VSLStartLocation_Fix && Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][2].Location < 6000)
					if (Minimum < (int)(Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][2].Location / 20))
						Minimum = (int)(Wavelet_History_Time[int(TIME/100)].Wavelet_History[wavelet_pushback_counter[car_wavelet]][2].Location / 20);
			}
			
			//int Minimum = 0;
			//for (int Mesh_Counter = 0; Mesh_Counter < (int)(L / 20); Mesh_Counter++)
			//{				
			//		if (Car_Wavelet_Segment_Counter[Mesh_Counter] >= 2)
			//			if (Minimum > Mesh_Counter)
			//				Minimum = Mesh_Counter;
			//}


			
			/*int minimum, car_number_minimum;
			minimum = abs( (int) (Wavelet_History_Time[TIME].Wavelet_History[wavelet_pushback_counter[0]][0].Location - 6000));
			car_number_minimum = wavelet_pushback_counter[0];

			for (int car_wavelet = 0; car_wavelet < wavelet_pushback_counter.size(); car_wavelet++)
			{
				if (Car_Wavelet_History[wavelet_pushback_counter[car_wavelet]] > 1)
				{
					if ( Wavelet_History_Time[TIME].Wavelet_History[wavelet_pushback_counter[car_wavelet]][0].Location > 4000)
					{
						if (minimum < abs( (int) (Wavelet_History_Time[TIME].Wavelet_History[wavelet_pushback_counter[car_wavelet]][0].Location - 6000)))
						{
							minimum < abs( (int) (Wavelet_History_Time[TIME].Wavelet_History[wavelet_pushback_counter[car_wavelet]][0].Location - 6000));
							car_number_minimum = wavelet_pushback_counter[car_wavelet];
						}
					}
				}
			}*/
			
			//if(Car_Wavelet_History[car_number_minimum] > 1)
			if (Minimum > (int)(VSLStartLocation_Fix / 20) && Minimum < (int)(VSLEndLocation_Fix / 20))
			{
				//Car_Wavelet_History[car_number_minimum] = 0;
				//double Wavelocation = Wavelet_History_Time[TIME].Wavelet_History[car_number_minimum][0].Location;
				double Wavelocation = Minimum * 20;
				Wavelocation = max((int)(Wavelocation/20), 0);
				VSLStartLocation = max(Wavelocation - 10000, 0);
				VSLEndLocation = max(Wavelocation + 10000, 0);//VSLStartLocation + 3000;
				//if (Wavelocation < 0)
				//	Wavelocation = 0;

				
				VSLInputL0 = NULL;
				VSLInputL1 = NULL;
				VSLInputL2 = NULL;
				VSLInputL3 = NULL;

				CheckDensity = 0;
				CheckSpeed = 0;
				CheckFlow = 0;

				VSLInputL0 = SFDLi_Wavelet (mesh_data_L0, TIME, max(Wavelocation - 4 , 0));
				VSLInputL1 = SFDLi_Wavelet (mesh_data_L1, TIME, max(Wavelocation - 4 , 0));
				VSLInputL2 = SFDLi_Wavelet (mesh_data_L2, TIME, max(Wavelocation - 4 , 0));
				VSLInputL3 = SFDLi_Wavelet (mesh_data_L3, TIME, max(Wavelocation - 4 , 0));

				CheckDensity = VSLInputL0->SDFDensity + VSLInputL1->SDFDensity + VSLInputL2->SDFDensity + VSLInputL3->SDFDensity;
				CheckSpeed = ((VSLInputL0->SDFSpeed * VSLInputL0->SDFDensity) + (VSLInputL1->SDFSpeed * VSLInputL1->SDFDensity) + (VSLInputL2->SDFSpeed * VSLInputL2->SDFDensity) + (VSLInputL3->SDFSpeed * VSLInputL3->SDFDensity))/CheckDensity;
				CheckFlow = CheckSpeed * CheckDensity * 3.6;

				if (CheckFlow > TargetFlow)///3.6)
				{
					/*if (CheckSpeed > (DesiredSpeed - 2.5))
						DesiredSpeedLimit = DesiredSpeed;
					if (CheckSpeed > (DesiredSpeed - 5) && CheckSpeed <= (DesiredSpeed - 2.5))
						DesiredSpeedLimit = DesiredSpeed - 2.5;
					if (CheckSpeed > (DesiredSpeed - 7.5) && CheckSpeed <= (DesiredSpeed - 5))
						DesiredSpeedLimit = DesiredSpeed - 5;
					if (CheckSpeed > (DesiredSpeed - 10) && CheckSpeed <= (DesiredSpeed - 7.5))
						DesiredSpeedLimit = DesiredSpeed - 7.5;
					if (CheckSpeed <= (DesiredSpeed - 10))
						DesiredSpeedLimit = DesiredSpeed - 10;*/
					
					if (CheckSpeed > (DesiredSpeed - 5))
						DesiredSpeedLimit = DesiredSpeed;
					if (CheckSpeed > (DesiredSpeed - 10) && CheckSpeed <= (DesiredSpeed - 5))
						DesiredSpeedLimit = DesiredSpeed - 5;					
					if (CheckSpeed <= (DesiredSpeed - 10))
						DesiredSpeedLimit = DesiredSpeed - 10;
				}
				else
				{
					if (CheckDensity <= TargetDensity)
						DesiredSpeedLimit = DesiredSpeed;
					else
					{
						/*if (CheckSpeed > (DesiredSpeed - 2.5))
							DesiredSpeedLimit = DesiredSpeed;
						if (CheckSpeed > (DesiredSpeed - 5) && CheckSpeed <= (DesiredSpeed - 2.5))
							DesiredSpeedLimit = DesiredSpeed - 2.5;
						if (CheckSpeed > (DesiredSpeed - 7.5) && CheckSpeed <= (DesiredSpeed - 5))
							DesiredSpeedLimit = DesiredSpeed - 5;
						if (CheckSpeed > (DesiredSpeed - 10) && CheckSpeed <= (DesiredSpeed - 7.5))
							DesiredSpeedLimit = DesiredSpeed - 7.5;
						if (CheckSpeed <= (DesiredSpeed - 10))
							DesiredSpeedLimit = DesiredSpeed - 10;*/
						if (CheckSpeed > (DesiredSpeed - 5))
							DesiredSpeedLimit = DesiredSpeed;
						if (CheckSpeed > (DesiredSpeed - 10) && CheckSpeed <= (DesiredSpeed - 5))
							DesiredSpeedLimit = DesiredSpeed - 5;					
						if (CheckSpeed <= (DesiredSpeed - 10))
							DesiredSpeedLimit = DesiredSpeed - 10;
					}
				}
				if (DesiredSpeedLimit != DesiredSpeed)
				{
					//EvaluationFlag = VSLActivation;// if 1 then VSL active, 0 otherwise
					VSLTIMECounter = 0;
					VSLActivationFlag = 1;
				}
				else
				{
					//EvaluationFlag = 0;
					VSLTIMECounter = 0;
					VSLActivationFlag = 0;
				}
			}

			if (VSLActivationFlag == 1 && VSLTIMECounter >= 6000)
			{
				DesiredSpeedLimit = DesiredSpeed;
				VSLTIMECounter = 0;
				VSLActivationFlag = 0;
			}
			//else
			//{
			//	CheckDensity = 0;
			//	CheckSpeed = 0;
			//	CheckFlow = 0;
			//}
		}
		//else
		//{
		//	CheckDensity = 0;
		//	CheckSpeed = 0;
		//	CheckFlow = 0;
		//}

		//----------------------------------------------Saving Output----------------------------------------------
		a = section_output(L0);
		a = section_output(L1);
		a = section_output(L2);
		a = section_output(L3);	

		/*if (TIME > 0)
		{
			if (L1 != NULL)
				cout<<"			"<<L1->carID<<endl;
			else
				cout<<"	 		"<<"None"<<endl;
			
			cout<<"				"<<TIME - AggDataHazardD_L1.size()<<endl;
			//if (TIME % 1000 == 0)
				getch();
		}*/
		/*//-------------------------------------------Saving Detector Data----------------------------------------------------
		//Added by ART 02/02/12
		car *DetectorL0;
		car *DetectorL1;
		DetectorL0 = NULL;
		DetectorL1 = NULL;
		DetectorL0 = L0;
		DetectorL1 = L1;
		
		

		double Occupancy = 0;
		double CarCounter = 0;
		double Speed = 0;

		while (DetectorL0 != NULL)
		{
			if (DetectorL0->x >= detectorlocation && (DetectorL0->x - DetectorL0->Deltax) < detectorlocation)
			{
				Occupancy = Occupancy + (DetectorL0->s + detectorlength) / DetectorL0->v;
				Speed = Speed + DetectorL0->v;
				CarCounter = CarCounter + 1;
			}
			DetectorL0 = DetectorL0->next;
		}

		while (DetectorL1 != NULL)
		{
			if (DetectorL1->x >= detectorlocation && (DetectorL1->x - DetectorL1->Deltax) < detectorlocation)
			{
				Occupancy = Occupancy + (DetectorL1->s + detectorlength) / DetectorL1->v;
				Speed = Speed + DetectorL1->v;
				CarCounter = CarCounter + 1;
			}
			DetectorL1 = DetectorL1->next;
		}

		
		DetectorOccupancy.push_back(Occupancy);
		DetectorFlow.push_back(CarCounter);
		DetectorSpeed.push_back(Speed);*/
		//End of Added by ART 02/02/12
		
		//-------------------------------------------End of Saving Detector Data---------------------------------------------		
	}



		//************************OUTPUT*********************************
//	arrangeCars(cars,carDeparting);
	//index the cars array with the carDeparting[x] to get the car
	//that departs 'x'th. e.g. cars[carsDeparting[0]] is the car that
	//departs first.


	
    FILE *st;
	char outputfilename[_MAX_PATH];

	sprintf(outputfilename, "%s\\Trajectory.dat", workingfolder);
//	st = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Simulated-Trajectory.txt", "w");
	char filename_st[9999];
	sprintf(filename_st, "..\\I-O Files_%d\\Simulated-Trajectory.txt",InitializationSeed);
//	cout << filename_st;
	st = fopen(filename_st, "w");
	int counter = 0;

	if(st !=NULL)
	{
		for (int i=0; i<AggCarsData.size(); i++)
		{
			//fprintf(st,"%d\n", i);
			for (int j=0; j<AggCarsData[i].AggData.size(); j++)
			{
				if (AggCarsData[i].AggData[j].x <= L)
				{
					fprintf(st, "%d	%d	%f	%f	%d\n",i,
						AggCarsData[i].AggData[j].t, 
						AggCarsData[i].AggData[j].x, 
						AggCarsData[i].AggData[j].v,
						AggCarsData[i].AggData[j].l);
						counter = j;
				};
			}
		}
	}

	fclose(st);

	


	a = section_output_final (0, AggDataHazardD_L0, AggDataHazardS_L0, AggDataHazardE_L0, AggDataHazardA_L0, SelectedSpeedLimit);
	a = section_output_final (1, AggDataHazardD_L1, AggDataHazardS_L1, AggDataHazardE_L1, AggDataHazardA_L1, SelectedSpeedLimit);
	a = section_output_final (2, AggDataHazardD_L2, AggDataHazardS_L2, AggDataHazardE_L2, AggDataHazardA_L2, SelectedSpeedLimit);
	a = section_output_final (3, AggDataHazardD_L3, AggDataHazardS_L3, AggDataHazardE_L3, AggDataHazardA_L3, SelectedSpeedLimit);


	FILE *st1;

//	st1 = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\TravelTime.txt", "w");
	char filename_tt[9999];
	sprintf(filename_tt, "..\\I-O Files_%d\\TravelTime.txt",InitializationSeed);
	// cout << filename_tt;
	st1= fopen(filename_tt, "w");

//	st1 = fopen("..\\I-O Files\\TravelTime.txt", "w");


	double travel_time= 0;
	int lastdata;

	if(st1 !=NULL)
	{
		for (int i=0; i<AggCarsData.size(); i++)
		{
			
			lastdata = AggCarsData[i].AggData.size()-1;
			
			if (lastdata > 0)
			{
				if (AggCarsData[i].AggData[lastdata].x > (L-5) && AggCarsData[i].AggData[0].x < 100)
				{
					
					travel_time = AggCarsData[i].AggData[lastdata].t - AggCarsData[i].AggData[0].t;
					fprintf(st1, "%d	%f\n",
						i,
						travel_time);
				}
				else
				{
					fprintf(st1, "%d	%f\n",
						i,
						0.0);
				}

			}
		}
	}

	fclose(st1);


	
	crashinfo *crashh = NULL;
	crashh = (crashinfo *) malloc(sizeof(crashh) * N);
	

	for (int i=0; i<AggCarsData.size(); i++)
		{
			crashh[i].flag = 0;
			crashh[i].segment = -10;
			for (int j=0; j<AggCarsData[i].AggData.size(); j++)
			{
				if (AggCarsData[i].AggData[j].crashlocation == 1 && crashh[i].flag == 0)
				{
					if (AggCarsData[i].AggData[j].x > 0 && AggCarsData[i].AggData[j].x <= 1000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 1;
						}
					}
					if (AggCarsData[i].AggData[j].x > 1000 && AggCarsData[i].AggData[j].x <= 2000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 2;
						}
					}
					if (AggCarsData[i].AggData[j].x > 2000 && AggCarsData[i].AggData[j].x <= 3000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 3;
						}
					}
					if (AggCarsData[i].AggData[j].x > 3000 && AggCarsData[i].AggData[j].x <= 4000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 4;
						}
					}
					if (AggCarsData[i].AggData[j].x > 4000 && AggCarsData[i].AggData[j].x <= 5000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 5;
						}
					}
					if (AggCarsData[i].AggData[j].x > 5000 && AggCarsData[i].AggData[j].x <= 6000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 6;
						}
					}
					if (AggCarsData[i].AggData[j].x > 6000 && AggCarsData[i].AggData[j].x <= 7000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 7;
						}
					}
					if (AggCarsData[i].AggData[j].x > 7000 && AggCarsData[i].AggData[j].x <= 8000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 8;
						}
					}
					if (AggCarsData[i].AggData[j].x > 8000 && AggCarsData[i].AggData[j].x <= 9000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 9;
						}
					}
					if (AggCarsData[i].AggData[j].x > 0 && AggCarsData[i].AggData[j].x <= 10000)
					{
						if (crashh[i].flag == 0)
						{
							crashh[i].flag = 1;
							crashh[i].segment = 10;
						}
					}
				}
			}
		}

	int crashsegment1 = 0;
	int crashsegment2 = 0;
	int crashsegment3 = 0;
	int crashsegment4 = 0;
	int crashsegment5 = 0;
	int crashsegment6 = 0;
	int crashsegment7 = 0;
	int crashsegment8 = 0;
	int crashsegment9 = 0;
	int crashsegment10 = 0;

		for (int i=0; i<AggCarsData.size(); i++)
		{
			if (crashh[i].flag == 1)
			{
				if (crashh[i].segment == 1)
					crashsegment1 = crashsegment1 + 1;
				if (crashh[i].segment == 2)
					crashsegment2 = crashsegment2 + 1;
				if (crashh[i].segment == 3)
					crashsegment3 = crashsegment3 + 1;
				if (crashh[i].segment == 4)
					crashsegment4 = crashsegment4 + 1;
				if (crashh[i].segment == 5)
					crashsegment5 = crashsegment5 + 1;
				if (crashh[i].segment == 6)
					crashsegment6 = crashsegment6 + 1;
				if (crashh[i].segment == 7)
					crashsegment7 = crashsegment7 + 1;
				if (crashh[i].segment == 8)
					crashsegment8 = crashsegment8 + 1;
				if (crashh[i].segment == 9)
					crashsegment9 = crashsegment9 + 1;
				if (crashh[i].segment == 10)
					crashsegment10 = crashsegment10 + 1;
			}
		}

	FILE *Crashcounter;
//	Crashcounter= fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Crash-Count.txt", "w");
	char filename_cc[9999];
	sprintf(filename_cc, "..\\I-O Files_%d\\Crash-Count.txt",InitializationSeed);
	// cout << filename_cc;
	Crashcounter= fopen(filename_cc, "w");
//	Crashcounter= fopen("..\\I-O Files\\Crash-Count.txt", "w");


	if(Crashcounter !=NULL)
	{
		fprintf(Crashcounter, "%d	%d	%d	%d	%d	%d	%d	%d	%d	%d\n", 
			crashsegment1,
			crashsegment2,
			crashsegment3,
			crashsegment4,
			crashsegment5,
			crashsegment6,
			crashsegment7,
			crashsegment8,
			crashsegment9,
			crashsegment10);
	}
	fclose(Crashcounter);

	//----------------------------------------------------------------------------------------------------------------------

	
	FILE *stDetector;
//	stDetector = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\Detector Information.txt", "w");
	char filename_det_info[9999];
	sprintf(filename_det_info, "..\\I-O Files_%d\\Detector Information.txt",InitializationSeed);
	// cout << filename_det_info;
	stDetector = fopen(filename_det_info, "w");

	int j;

	for (int i=1; i<DetectorOccupancy.size(); i++)
	{
		if (i % 300 == 0)
		{
			j = i-299;
			while (j<i-1)
			{
				j = j + 1;
				DetectorOccupancy[i] = DetectorOccupancy[i] + DetectorOccupancy[j];
				DetectorFlow[i] = DetectorFlow[i] + DetectorFlow[j];
				if (DetectorFlow[j]!=0)
					DetectorSpeed[j] = DetectorSpeed[j] / DetectorFlow[j];
				else
					DetectorSpeed[j] = 0;

				DetectorSpeed[i] = DetectorSpeed[i] + DetectorSpeed[j];
			}
			DetectorOccupancy[i] = (DetectorOccupancy[i]* 100) / (30*2);
			DetectorFlow[i] = DetectorFlow[i] * (36000/(300*2));
			DetectorSpeed[i] = DetectorSpeed[i] / (30);
			if(stDetector !=NULL)	
			{
				fprintf(stDetector, "%d	%f	%f	%f\n",
					i,
					DetectorOccupancy[i],
					DetectorFlow[i],
					DetectorSpeed[i]);
			}
		}
	}

	fclose (stDetector);
	






	/* Temporally deleted by ART 07/08/14
	sprintf(outputfilename, "%s\\SegmentOutput.dat", workingfolder);
	st = fopen(outputfilename, "w");
	if (st !=NULL)
	{
		fprintf(st, "%d	%d\n",AggregateSegments.size(),AggregateSegments[0].Lane0.size());

		fprintf(st,"travel time:\n");
		//avg travel time for lane0 in second
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				if (AggregateSegments[i].Lane0[j].NumOfCarsPassAtLane>0) 
				{
					AggregateSegments[i].Lane0[j].TravelTimeAtLane = (float) AggregateSegments[i].Lane0[j].TravelTimeAtLane/AggregateSegments[i].Lane0[j].NumOfCarsPassAtLane/10; 
				}
				else
				{
					AggregateSegments[i].Lane0[j].TravelTimeAtLane = 9999;
				}
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane0[j].TravelTimeAtLane);
			}
			fprintf(st,"\n");
		}

		//avg travel time for lane1 in second
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				if (AggregateSegments[i].Lane1[j].NumOfCarsPassAtLane>0)
				{
					AggregateSegments[i].Lane1[j].TravelTimeAtLane = (float) AggregateSegments[i].Lane1[j].TravelTimeAtLane/AggregateSegments[i].Lane1[j].NumOfCarsPassAtLane/10; 
				}
				else
				{
					AggregateSegments[i].Lane1[j].TravelTimeAtLane = 9999;
				}
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane1[j].TravelTimeAtLane);
			}
			fprintf(st,"\n");
		}

		fprintf(st,"speed:\n");

		//speed for lane 0
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				if (AggregateSegments[i].Lane0[j].NumOfCarsForSpeed>0)
				{
					AggregateSegments[i].Lane0[j].SpeedAverageAtLane = (float) AggregateSegments[i].Lane0[j].SpeedAverageAtLane/AggregateSegments[i].Lane0[j].NumOfCarsForSpeed; 
				}
				else
				{
					AggregateSegments[i].Lane0[j].SpeedAverageAtLane = 9999;
				}
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane0[j].SpeedAverageAtLane);
			}
			fprintf(st,"\n");
		}

		//speed for lane 1
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				if (AggregateSegments[i].Lane1[j].NumOfCarsForSpeed > 0)
				{
					AggregateSegments[i].Lane1[j].SpeedAverageAtLane = (float) AggregateSegments[i].Lane1[j].SpeedAverageAtLane/AggregateSegments[i].Lane1[j].NumOfCarsForSpeed; 
				}
				else
				{
					AggregateSegments[i].Lane1[j].SpeedAverageAtLane = 9999;
				}
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane1[j].SpeedAverageAtLane);
			}
			fprintf(st,"\n");
		}

		fprintf(st, "headway:\n");
		//headway for lane 0
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				if (AggregateSegments[i].Lane0[j].NumOfCarsForHeadway >0)
				{
					AggregateSegments[i].Lane0[j].HeadwayAtLane = (float) AggregateSegments[i].Lane0[j].HeadwayAtLane /AggregateSegments[i].Lane0[j].NumOfCarsForHeadway; 
				}
				else
				{
					AggregateSegments[i].Lane0[j].HeadwayAtLane = 9999;
				}
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane0[j].HeadwayAtLane);
			}
			fprintf(st,"\n");
		}

		//headway for lane 1
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				if (AggregateSegments[i].Lane1[j].NumOfCarsForHeadway >0)
				{
					AggregateSegments[i].Lane1[j].HeadwayAtLane = (float) AggregateSegments[i].Lane1[j].HeadwayAtLane /AggregateSegments[i].Lane1[j].NumOfCarsForHeadway; 
				}
				else
				{
					AggregateSegments[i].Lane1[j].HeadwayAtLane = 9999;
				}
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane1[j].HeadwayAtLane);
			}
			fprintf(st,"\n");
		}

		fprintf(st, "accidents:\n");
		//accidents for lane 0
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				fprintf(st, "%d	",AggregateSegments[i].Lane0[j].NumOfAccidentsAtLane);
			}
			fprintf(st,"\n");
		}

		//accidents for lane 1
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				fprintf(st, "%d	",AggregateSegments[i].Lane1[j].NumOfAccidentsAtLane);
			}
			fprintf(st,"\n");
		}

		fprintf(st, "num of lane changes:\n");
		//num of lane changes for lane 0
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				fprintf(st, "%d	",AggregateSegments[i].Lane0[j].NumOfLaneChanges);
			}
			fprintf(st,"\n");
		}

		fprintf(st,"flowrate:\n");
		// flowrate for lane 0
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				AggregateSegments[i].Lane0[j].FlowRateAtLane = (float) AggregateSegments[i].Lane0[j].NumOfCarsPassAtLane * TIMEITERVALS_HOUR/AGG_TIMEINTERVAL_SEGMENT; 
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane0[j].FlowRateAtLane);
			}
			fprintf(st,"\n");
		}

		// flowrate for lane 1
		for (int j=0; j<AggregateSegments[0].Lane1.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				AggregateSegments[i].Lane1[j].FlowRateAtLane = (float) AggregateSegments[i].Lane1[j].NumOfCarsPassAtLane * TIMEITERVALS_HOUR/AGG_TIMEINTERVAL_SEGMENT; 
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane1[j].FlowRateAtLane);
			}
			fprintf(st,"\n");
		}
		
		fprintf(st,"density:\n");
		//density for lane 0
		for (int j=0; j<AggregateSegments[0].Lane0.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				AggregateSegments[i].Lane0[j].DensityAtLane = 1000 * (float)AggregateSegments[i].Lane0[j].NumOfCarsUseAtLane/AGG_TIMEINTERVAL_SEGMENT/AGG_SEGMENT_LENGTH; 
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane0[j].DensityAtLane);
			}
			fprintf(st,"\n");
		}

		//density for lane 1
		for (int j=0; j<AggregateSegments[0].Lane1.size();j++)
		{
			for (int i=0; i<AggregateSegments.size();i++)
			{
				AggregateSegments[i].Lane1[j].DensityAtLane = 1000*(float)AggregateSegments[i].Lane1[j].NumOfCarsUseAtLane/AGG_TIMEINTERVAL_SEGMENT/AGG_SEGMENT_LENGTH; 
				fprintf(st, "%8.2f	",AggregateSegments[i].Lane1[j].DensityAtLane);
			}
			fprintf(st,"\n");
		}
	}

	sprintf(outputfilename, "%s\\AccidentOutput.dat", workingfolder);
	st = fopen(outputfilename, "w");
	if (st !=NULL)
	{
		fprintf(st, "%d\n",crashes.size());

		fprintf(st,"accidents:\n");
		//accedents
		for (int j=0; j<crashes.size();j++)
		{
			fprintf(st, "%d	%8.2f	%8.2f	%d	%d",crashes[j].CrashIndex,crashes[j].CrashTime * 0.1,crashes[j].CrashPostion,crashes[j].CrashLane,crashes[j].CarID);
			fprintf(st,"\n");
		}
	}
	fclose(st);
	*///Temporally deleted by ART 07/08/14
	//**********************END OUTPUT*******************************
	free(cars);    //no longer needed => free memory

//********************END SIMULATION*********************

}


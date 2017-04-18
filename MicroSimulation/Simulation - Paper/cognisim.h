	// ARTDriver's Behavior from a Cognitive Perspective
	// CogniSim: A Micro-Simulation Software
 
	//  Requested softwares: 
	//	Platform: Microsoft Visual C++ 2005
	//	OS:	Microsoft Windows  

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
	#include "mrand.h"
	

	#ifndef COGNISIM_H
	#define COGNISIM_H

	// Added for VC
	using namespace std;
	//char workingfolder[_MAX_PATH];
	LPWSTR workingfolder;
	char filename[_MAX_PATH];


//*******************GLOBAL VARIABLES & DEFAULT VALUES*********************

long T = 180000; // total number of timesteps. Must be multiple of 10!
long TIME = 0; //current timestep
long N = 1000; // number of cars
double L = 2000; //length of simulated road 3.0
//double safeD = 50.0; //safety margin for car to enter simulation
//double safeLC = 30.0; //safety margin on either side for car to prevent 3.0
							//lane change (if within safety margin, the lane change
							//crash percentage is used to decide lane change or not.
int TIMEITERVALS_HOUR  = 36000; // 1 hr
int AGG_TIMEINTERVAL = 1;
long CUR_TIMEINTERVAL_INDEX = 0;
int NUM_AGG_TIMEINTERVALS = T/AGG_TIMEINTERVAL;
float HEADWAY_FIRST_CAR = 99999;

/*float MEAN_INTERARRIVAL_TIME1 = 60;
float MEAN_INTERARRIVAL_TIME2 = 35;
float MEAN_INTERARRIVAL_TIME3 = 60;
float MEAN_INTERARRIVAL_TIME = 40;// was = 1

double DesiredSpeed = 25;
double InitialSpeed = 20;

float TargetFlow = 1200;
float TargetDensity = 10;

float VSLStartLocation = 4500;

float NonComplianceRate = 0.05;
float OverspeedingRate = 0.1;

int VSLActivation = 1;


float detectorlocation = 5950;
float detectorlength = 2;*/


float DV_FIRST_CAR = 9999;
double PI = 3.141592653589793238462643383279502884197169399375;
 
double mean_s = 6.5; // parameters of the vehicles' lengths distribution
double std_s = 0.3;
double range_s = 2.0;
 

double meanacc = 2.0;//Treat as Maximum Acceleration for the PT Model
double stdacc = 0.3;
double rangeacc = 2.0;

double meandecc = -3.0;
double stddecc = 0.3;
double rangedecc = 2.0;

double meanVd_1 = 13.3333;//Treat as Desired Velocity for in the PT Model
double stdVd_1 = 3.2;
double rangeVd_1 = 18.0;

double meanVd_2 = 35.5555;
double stdVd_2 = 4.0;
double rangeVd_2 = 52.0;

double percentVd_1 = 40.0;
double percentVd_2 = 60.0;

double meanrisk = 15.0;
double stdrisk = 5.0;
double rangerisk = 20.0;

double meanRt = 1.0;//Treat as Reaction Time in the PT Model
double stdRt = 0.4;
double rangeRt = 1.4;

double meanLCT = 2.5;
double stdLCT = 0.4;
double rangeLCT = 1.0;

double meanLCcrash = 40.0;
double stdLCcrash = 30.0;
double rangeLCcrash = 80.0;

double startV = 20.00; //initial velocity upon entering simulation
 

//_____________PT_Parameters

double mean_Gamma = 0.2;
double std_Gamma = 0.1;
double range_Gamma = 0.1;

double mean_Wm = 2.0;
double std_Wm = 1.0;
double range_Wm = 1.0;

double mean_Wc = 100000.0;
double std_Wc = 50000.0;
double range_Wc = 50000.0;

double mean_Tmax = 4.0;
double std_Tmax = 2.0;
double range_Tmax = 2.0;

double mean_Alpha = 0.08;
double std_Alpha = 0.04;
double range_Alpha = 0.04;

double mean_Beta = 5.0;
double std_Beta = 2.0;
double range_Beta = 2.0;

double mean_Tcorr = 20.0;
double std_Tcorr = 5.0;
double range_Tcorr = 5.0;

double mean_So = 3.0;
double std_So = 0.0;
double range_So = 0.0;

//____________Hazard Parameters

double Hazard_ParameterA = 1.0; //Alpha of the Duration Model
double Hazard_ParameterB = 1.0; //Teta of the Duration Model

double R_Heterogeneity = 1.0; //R Parameter for the Gamma Heterogeneity
double K_Heterogeneity = 1.0; //K Parameter for the Gamma Heterogeneity

	//Exogenous Covariates Parameters

//Car-Following Episodes
double BetaOne_CF = 1.0;

double BetaLCL_CF = 1.0;

double BetaV_CF = 1.0;

double BetaDXL1_CF = 1.0;
double BetaDVL1_CF = 1.0;

double BetaDXL2_CF = 1.0;
double BetaDVL2_CF = 1.0;

double BetaDXF1_CF = 1.0;
double BetaDVF1_CF = 1.0;

double BetaDXL1R_CF = 1.0;
double BetaDVL1R_CF = 1.0;
double BetaDXF1R_CF = 1.0;
double BetaDVF1R_CF = 1.0;

double BetaDXL1L_CF = 1.0;
double BetaDVL1L_CF = 1.0;
double BetaDXF1L_CF = 1.0;
double BetaDVF1L_CF = 1.0;

double BetaKL_CF = 1.0;
double BetaKR_CF = 1.0;

double BetaK_CF = 1.0;

//Free_Flow Episodes
double BetaOne_FF = 1.0;

double BetaLCL_FF = 1.0;

double BetaV_FF = 1.0;

double BetaDXL1_FF = 1.0;
double BetaDVL1_FF = 1.0;

double BetaDXL2_FF = 1.0;
double BetaDVL2_FF = 1.0;

double BetaDXF1_FF = 1.0;
double BetaDVF1_FF = 1.0;

double BetaDXL1R_FF = 1.0;
double BetaDVL1R_FF = 1.0;
double BetaDXF1R_FF = 1.0;
double BetaDVF1R_FF = 1.0;

double BetaDXL1L_FF = 1.0;
double BetaDVL1L_FF = 1.0;
double BetaDXF1L_FF = 1.0;
double BetaDVF1L_FF = 1.0;

double BetaKR_FF = 1.0;
double BetaKL_FF = 1.0;

double BetaK_FF = 1.0;
 

// To be Added Later on in the Input/Output File
double RampLocation = 5000.0;//The location of the start of the merging area
double MergingLength = 300.0;//The length of the merging area where a lane change is possible

double BetaRamp_CF = 1.0;
double BetaLeftLane_CF = 1.0;
double BetaRightLane_CF = 1.0;
double BetaDistanceToRamp_CF = 1.0;

double BetaRamp_FF = 1.0;
double BetaLeftLane_FF = 1.0;
double BetaRightLane_FF = 1.0;
double BetaDistanceToRamp_FF = 0.0001;
 

int sens[10]; //sensitivity selection matrix
int sel; //sensitivity selection variable
double SAFE_HEADWAY = 50.0;
double CRTICAL_HEADWAY = 2.0;
double MAX_DECC = -5.0;
double VD_MINIMAL = 5.0;
double CRASH_LANE = 2;
double CRASH_POSITION = 0.0;
double CRASH_ALERT_LENGTH = 1000;
double CRASH_ALERT_PARAM1=100;
double CRASH_ALERT_PARAM2=1500;

struct Crash
{
	int CrashIndex;
	int CarID;
	int CrashLane;
	long CrashTime;
	double CrashPostion;
};
vector<Crash> crashes;

//*****************END GLOBAL VARIABLES & DEFAULT VALUES*******************

//*********************************STRUCTS*********************************

struct car
{
	short Connectivity_Status; //0 if not connected, 1 if connected, 2 if autonomous
	short VSLFalg;

	float acc; //acceleration
	float decc; //decceleration
	float Vd; //desirable speed--> PT Model Mapped

	short lane;
	short l; //lane change timestep delay integer

	float a_update;//for updating the acceleration
	float s; //length of car
	float x; //position of car
	float v; //current velocity
	float Deltax;//The movement from the previous time step

	float headway; //distance to car in front
 
	float DV;
	float Spacing;
 

	float risk; //risk (aggressiveness) variable
	float Rt; //reaction time of vehicle. Multiple of 0.1  --> PT Model Mapped
	short react; //integer map of reaction time (multiply it by ten) --> PT Model Mapped
	short LCT; //integer map of lane changing time (i.e. multiplied by ten)
	short LCcrash; //lane changing hazard crash percentage (always between 0 and 100 inc.)
	short departure; //departure timestep of vehicle (0 to T-1)

	short carID;
	short crash; //zero means no crash, one means crash


 
	short crashtimecounter_ramp; //check if the ramp flow blocks by an accident, then remove the car
	short crashtimecounter; //check if the L0 flow blocks by an accident, then remove the car
 

	car * previous;//
	car * next;//
	float x_prev;
	long dt_prev;
	short crash_prev;
	short lane_prev;
	float v_prev;
	//float risk_original;
	//float decc_original;
	//float Vd_original;


	//_________PT Model
	float Yt;
	float Yt_previous;
	float Leader_Previous;
	
	float Gamma;
	float Wm;
	float Wc;
	float Tmax;
	float Alpha;
	float Beta;
	float Tcorr;

	float Deltan;
	float Tn;
	float Son;
	float an;
	float bn;
	float Von;

	//float Amax;//mayve will not need it
	//float Vo;//maybe will not need it
	float So;
	//float Ao;

	float Wp;
	float DeltaW;
	float Sigma;

	//_________Hazard Model: Nothing Specific to One Driver verus Another for Parameters
 

	 
	short Dummy_Shockwave;
 
	//Hazard Based Model Variables to Store for Survival Probability Computation
	short LCL;
	/*float DXL1;
	float DVL1;
	float DXL2;
	float DVL2;
	float DXF1;
	float DVF1;
	float DXL1R;
	float DVL1R;
	float DXF1R;
	float DVF1R;
	float DXL1L;
	float DVL1L;
	float DXF1L;
	float DVF1L;
	float KR;
	float KL;
	float K;*/

	float Duration;//in seconds: multiple of time steps divided by 10
	short Mode; // = 1 if CF and 0 if FF and -1 if accident
 
	short PreviousLeaderID;//To determin if a duration ends or not if it changed 
 

 
	float GammaHeterogeneity;
	
	short RampDummy;
	short LeftLaneDummy;
	short RightLaneDummy;

	float DistanceToRamp; //In Absolute Value
	float Hazard;

	float DummyDXL1;
	float DummyDVL1;
	 

 
	short lanechangingflag;
	 
	float basehazard;
 
	int ComplianceValue;
	int Compliance; //if 0 then not comply with speed limit, 1 otherwise.
	float overspeedingrate; 
 

	float Crash_Probability;

 
	
	short targetrampside; // 0 if current target in on left, 1 if it is on right, 2 if there is no target ramp (change lane freely)
 
	short changelane_direction;//0 for the left, 1 for the rigth, -1 for no decision
	short destlane;
	short changelaneinstance; // 1 if the change lane happened in the previous time, 0 otherwise.

	float K;
	float KPrime;
	float Ka;
	float Kv;
	float Kd;

};

struct crashinfo
{
	int flag;
	int segment;
};

struct data  //class to store needed output information at each iteration
{
	float x; //position
	short l;  //lane
	float v; //velocity
	float h;   //headway
	long t; // current time
	short crashlocation;// crash location

	//double a;//maybe will not use it
};

struct data2  //class to store needed output information at each iteration
{
	float Section1;
	float Section2;
	float Section3;
	float Section4;
	float Section5;
	float Section6;
	float Section7;
	float Section8;
	float Section9;
	float Section10;
	float Hazard1;
	float Hazard2;
	float Hazard3;
	float Hazard4;
	float Hazard5;
	float Hazard6;
	float Hazard7;
	float Hazard8;
	float Hazard9;
	float Hazard10;
	float Hazardcounter1;
	float Hazardcounter2;
	float Hazardcounter3;
	float Hazardcounter4;
	float Hazardcounter5;
	float Hazardcounter6;
	float Hazardcounter7;
	float Hazardcounter8;
	float Hazardcounter9;
	float Hazardcounter10;
};

struct data3  //class to store needed output information at each iteration
{
	float Section1;
	float Section2;
	float Section3;
	float Section4;
	float Section5;
	float Section6;
	float Section7;
	float Section8;
	float Section9;
	float Section10;
};

struct data4
{
	float Section1;
	float Section2;
	float Section3;
	float Section4;
	float Section5;
	float Section6;
	float Section7;
	float Section8;
	float Section9;
	float Section10;
};

 
struct Mesh_Data
{
	float Density[500];
	float Hazard[500];
	float Hazardcounter[500];
	float Speed[500];
};

 


struct CarAggregateTime // aggregate data for each car at each time interval such as 1 min
{
	double DeparturePosition;
	double TravelDistanceAtTime;
	int LaneAtTime;
	double SpeedTimeAverage;
	double HeadwayTimeAverage;
	int NumOfLaneChangesAtTime;
	int AccidentsFlagAtTime;
};

struct CarAggregateSegment // aggregate data for each car at each segement such as 1 km
{
	int DepartureTime; 
	int TravelTimeAtSegment; 
	double SpeedSegmentAverage;
	int NumOfLaneChangesAtSegment;
	int AccidentFlagAtSegment;
};

struct CarAggregateData
{
	int CarID;
	vector<CarAggregateTime> TimeAverageData;
	vector<CarAggregateSegment> DistanceAverageData;
};

// segement time average
/*struct MacroAggregateSegmentLane //aggregate data for each segement and lane at each time 
{
	double FlowRateAtLane;
	double DensityAtLane;
	double TravelTimeAtLane;
	double SpeedAverageAtLane;
	double HeadwayAtLane;
	long NumOfCarsPassAtLane;
	long NumOfCarsUseAtLane;
	long NumOfAccidentsAtLane;
	long NumOfCarsForSpeed;
	long NumOfCarsForHeadway;
	long NumOfLaneChanges; // for segment
};

struct SegmentAggregateData
{
	int SegmentID;
	int NumOfLanesAtSegment;
	long NumOfLaneChangesAtSegment;
	double FlowRateAtSegment;
	double DensityAtSegment;
	double TravelTimeAtSegment;
	double SpeedAverageAtSegment;
	vector<MacroAggregateSegmentLane> Lane0;
	vector<MacroAggregateSegmentLane> Lane1;
};*/

struct AggCars
{
	int CarID;
	vector<data> AggData;
		
};

struct Driver_Information
{
	float Rt;
	float Wm;
	float Wc;
	float Tmax;
	float Tcorr;
	float Alpha;
	float Gamma;
	float Beta;

	float Deltan;
	float Tn;
	float Son;
	float an;
	float bn;
	float Von;
};

vector<Driver_Information> Drivers;
Driver_Information tempDriver;


vector<data2> AggDataHazardD_L0;
vector<data2> AggDataHazardD_L1;
vector<data2> AggDataHazardD_L2;
vector<data2> AggDataHazardD_L3;

vector<data3> AggDataHazardS_L0;
vector<data3> AggDataHazardS_L1;
vector<data3> AggDataHazardS_L2;
vector<data3> AggDataHazardS_L3;

vector<data3> AggDataHazardE_L0;
vector<data3> AggDataHazardE_L1;
vector<data3> AggDataHazardE_L2;
vector<data3> AggDataHazardE_L3;

vector<data4> AggDataHazardA_L0;
vector<data4> AggDataHazardA_L1;
vector<data4> AggDataHazardA_L2;
vector<data4> AggDataHazardA_L3;

vector<Mesh_Data> mesh_data_L0;
vector<Mesh_Data> mesh_data_L1;
vector<Mesh_Data> mesh_data_L2;
vector<Mesh_Data> mesh_data_L3;

vector<double> DetectorOccupancy;
vector<double> DetectorFlow;
vector<double> DetectorSpeed;


vector<AggCars> AggCarsData;
//vector<SegmentAggregateData> AggregateSegments;
//vector<CarAggregateData> AggregateCars;
 

car *saveL0_High_Hazard;
car *saveL1_High_Hazard;
car *saveL2_High_Hazard;
car *saveL3_High_Hazard;
Mesh_Data tempMesh;

//******************************END STRUCTS*********************************

//**************************FUNCTION PROTOTYPES**************************

void getSimFile(int,char[13]); //save or load a simulation file. 0=save, 1=load.
										//second argument is array in which to write
										//filename in case of a 'load'.

int sensitivity(char[13]); //draws sensitivity menu and returns choice

char * convert(double, char *);  //converts a float to a string with 2 decimal places
char * convert2(double, char *); //same as above, but with 1 decimal place

double normal(double , double, double,int,int);
//give it mean, std dev., and range, and this function returns a random
//variable with normal distribution within the given parameters.

//**********************END FUNCTION PROTOTYPES**************************

void getSimFile(int mode, char destination[13]) //0=Save 1=Load
{//

	//DWORD i1=0;
	GetCurrentDirectoryW(_MAX_PATH,workingfolder);
	
	
	//TCHAR filename[_MAX_PATH];

	sprintf(filename, "%s\\MACRO0.SIM", workingfolder);
}

char * convert(double f, char *str)
{
	int c;
	int flag =0;
	int a,b;
	char *r,*s;

	a = (int) f;
	if (a >= 10000)
	{
		flag = 2;
		f/=100; //to stop gcvt from returning standard index form
	}
	else
		if (a >= 1000)
		{
			flag = 1;
			f/=10;//to stop gcvt from returning standard index form
		}
	for (b=10,c=3; (a/b) != 0 ;c++)
		b*=10;

	gcvt(f,c,str);

	r = str;
	while ((*r!='.')&&(*r!=0))
		r++;
	if (*r == 0)
		strcat(str,".00");
	else
	{
		if (*(r+1) == 0)
			strcat(str,"00");
		else
			if (*(r+2) == 0)
			strcat(str,"0");
	}

	if (flag)
	{
		r = str;
		while (*r != '.')
			r++;

		while (flag) //move decimal point right 'flag' digits
		{
			flag--;
			s = r;
			s++;
			*r = *s;
			*s = '.';
			r++;
		}
	}

	r = str;
	while ((*r!='.')&&(*r!=0))
		r++;
	if (*r == 0)
		strcat(str,".00");
	else
	{
		if (*(r+1) == 0)
			strcat(str,"00");
		else
			if (*(r+2) == 0)
				strcat(str,"0");
	}

	return str;
}

char * convert2(double f, char *str)
{
	int c;
	int flag =0;
	int a,b;
	char *r,*s;

	a = (int) f;
	if (a >= 10000)
	{
		flag = 2;
		f/=100; //to stop gcvt from returning standard index form
	}
	else
		if (a >= 1000)
		{
			flag = 1;
			f/=10;//to stop gcvt from returning standard index form
		}

	for (b=10,c=2; (a/b) != 0 ;c++)
		b*=10;

	gcvt(f,c,str);

	r = str;
	while ((*r!='.')&&(*r!=0))
		r++;
	if (*r == 0)
		strcat(str,".0");
	else
		if (*(r+1) == 0)
			strcat(str,"0");

	if (flag)
	{
		r = str;
		while (*r != '.')
			r++;

		while (flag) //move decimal point right 'flag' digits
		{
			flag--;
			s = r;
			s++;
			*r = *s;
			*s = '.';
			r++;
		}
	}

	r = str;
	while ((*r!='.')&&(*r!=0))
		r++;
	if (*r == 0)
		strcat(str,".0");
	else
		if (*(r+1) == 0)
			strcat(str,"0");

	return str;
}

 
//double normal(double mean, double sigma, double range)
//{
//	double U1, U2, W;
//	int flag =0;
//
//	while (1)
//	{
//		flag=0;
//		while (!flag)
//		{
//			U1 = (double) rand();
//			U2 = (double) rand();
//			U1 /= (double) RAND_MAX;
//			U2 /= (double) RAND_MAX;
//			U1 = 2*U1 - 1;
//			U2 = 2*U2 - 1;
//			W = U1*U1 + U2*U2;
//			if (W<=1)
//			{
//				flag = 1;
//				W = sqrt( (-2 * log(W) ) / W );
//				W = U1 * W;
//			}
//		}
//
//		W = mean + W*sigma;
//		if ( ( W > (mean - (range/2)) ) && (W < ( mean + (range/2)) ) )
//		{
//			return W;
//		}
//	}
//}

double normal(double mean, double sigma, double range,int stream1, int stream2)
{
	 

		if (sigma == 0 || range == 0)
			return mean;
		else
		{
			double U1, U2, W;
			int flag =0;

			while (1)
			{
				flag=0;
				while (!flag)
				{
					U1 = (double) mrand(stream1);
					U2 = (double) mrand(stream2);
					U1 = 2*U1-1;
					U2 = 2*U2-1;
					W = U1*U1 + U2*U2;
					if (W<=1)
					{
						flag = 1;
						W = sqrt( (-2 * log(W) ) / W );
						W = U1 * W;
						break;
					}
				}

				W = mean + W*sigma;
				if ( ( W > (mean - (range/2)) ) && (W < ( mean + (range/2)) ) )
				{
					return W;
				}
			}
		}

	 
}

 
int factorial (int num)
{

 if (num==1)
   return 1;

 return factorial(num-1)*num; // recursive call
}
 
 
double GammaGenerate (double scale, double shape)
{
	double Storage;
	Storage = 0.0;

	for (int i = 0; i < (int)shape; i++)
		Storage = Storage + log ( mrand (101) );

	return -scale * Storage;
}

 



//save simulation to .sim file. FILE * outsim points to
//a textfile already open in write mode.
//Returns TRUE if succeeds, FALSE if fails.


 

//int saveSim(FILE * outSim,car *cars)
//{
//	if (!outSim)
//		return 0;
//
//	char str1[30];
//	char str2[30];
//	char str3[30];
//	char str4[30];
//	char str5[30];
//	int c;
//
//	fprintf(outSim,"T(timesteps)=%s\tN=%s\t\tL=%s\n",itoa(T,str1,10),itoa(N,str2,10),itoa(L,str3,10));
 
//	fprintf(outSim,"mean_s=%s\t\tstd_s=%s\trange_s=%s\n",convert(mean_s,str1),convert(std_s,str2),convert(range_s,str3));
 
//	fprintf(outSim,"meanacc=%s\t\tstdacc=%s\trangeacc=%s\n",convert(meanacc,str1),convert(stdacc,str2),convert(rangeacc,str3));
//	fprintf(outSim,"meandecc=%s\t\tstddecc=%s\trangedecc=%s\n",convert(meandecc,str1),convert(stddecc,str2),convert(rangedecc,str3));
//	fprintf(outSim,"meanVd_1=%s\t\tstdVd_1=%s\trangeVd_1=%s\n",convert(meanVd_1,str1),convert(stdVd_1,str2),convert(rangeVd_1,str3));
//	fprintf(outSim,"meanVd_2=%s\t\tstdVd_2=%s\trangeVd_2=%s\n",convert(meanVd_2,str1),convert(stdVd_2,str2),convert(rangeVd_2,str3));
//	fprintf(outSim,"percentVd_1=%s\n",convert(percentVd_1,str1));
//	fprintf(outSim,"meanrisk=%s\t\tstdrisk=%s\trangerisk=%s\n",convert(meanrisk,str1),convert(stdrisk,str2),convert(rangerisk,str3));
//	fprintf(outSim,"meanRt=%s\t\tstdRt=%s\trangeRt=%s\n",convert(meanRt,str1),convert(stdRt,str2),convert(rangeRt,str3));
//	fprintf(outSim,"meanLCT=%s\t\tstdLCT=%s\trangeLCT=%s\n",convert(meanLCT,str1),convert(stdLCT,str2),convert(rangeLCT,str3));
//	fprintf(outSim,"meanLCcrash=%s\t\tstdLCcrash=%s\trangeLCcrash=%s\n",convert(meanLCcrash,str1),convert(stdLCcrash,str2),convert(rangeLCcrash,str3));
//	fprintf(outSim,"startV=%s\n",convert(startV,str1));
//	fprintf(outSim,"\n");
//
//	for (c=0; c<N; c++)
//	{
//		fprintf(outSim,"*************************************CAR %i***************************************\n", c+1);
//		fprintf(outSim,"acc=%s\tdecc=%s\tVd=%s\t\tlane=%s\t\ts=%s\n",convert(cars[c].acc,str1),convert(cars[c].decc,str2),convert(cars[c].Vd,str3),itoa(cars[c].lane,str4,10),convert(cars[c].s,str5));
//		fprintf(outSim,"risk=%s\tRt=%s\t\tLCT(timesteps)=%s\tLCcrash=%s\tdepart(timestep)=%s\n\n",convert(cars[c].risk,str1),convert(cars[c].Rt,str2),itoa(cars[c].LCT,str3,10),itoa(cars[c].LCcrash,str4,10),itoa(cars[c].departure,str5,10));
//	}
//
//	return 1;
//
//}

int saveSim(FILE * outSim,car *cars)
{
	if (!outSim)
		return 0;

	char str1[30];
	char str2[30];
	char str3[30];
	char str4[30];
	char str5[30];
	char str6[30];
	int c;

	//Simulation Related Parameters
	fprintf(outSim,"T(timesteps)=%s\tN=%s\t\tL=%s\n",itoa(T,str1,10),itoa(N,str2,10),itoa(L,str3,10));
	fprintf(outSim,"startV=%s\n\n",convert(startV,str1));

	//Vehicle Specific Parameters
	fprintf(outSim,"mean_s=%s\t\tstd_s=%s\trange_s=%s\n",convert(mean_s,str1),convert(std_s,str2),convert(range_s,str3));
	fprintf(outSim,"meanacc=%s\t\tstdacc=%s\trangeacc=%s\n",convert(meanacc,str1),convert(stdacc,str2),convert(rangeacc,str3));
	fprintf(outSim,"meandecc=%s\t\tstddecc=%s\trangedecc=%s\n",convert(meandecc,str1),convert(stddecc,str2),convert(rangedecc,str3));//For Removal
	fprintf(outSim,"meanVd_1=%s\t\tstdVd_1=%s\trangeVd_1=%s\n",convert(meanVd_1,str1),convert(stdVd_1,str2),convert(rangeVd_1,str3));
	fprintf(outSim,"meanVd_2=%s\t\tstdVd_2=%s\trangeVd_2=%s\n",convert(meanVd_2,str1),convert(stdVd_2,str2),convert(rangeVd_2,str3));
	fprintf(outSim,"percentVd_1=%s\n",convert(percentVd_1,str1));
	fprintf(outSim,"meanRt=%s\t\tstdRt=%s\trangeRt=%s\n",convert(meanRt,str1),convert(stdRt,str2),convert(rangeRt,str3));
	fprintf(outSim,"meanLCT=%s\t\tstdLCT=%s\trangeLCT=%s\n\n",convert(meanLCT,str1),convert(stdLCT,str2),convert(rangeLCT,str3));

	//Car-Following Parameters: PT Model
	fprintf(outSim,"mean_Gamma=%s\t\tst_Gamma=%s\t\trange_Gamma=%s\n",convert(mean_Gamma,str1),convert(std_Gamma,str2),convert(range_Gamma,str3));
	fprintf(outSim,"mean_Wm=%s\t\tstd_Wm=%s\t\trange_Wm=%s\n",convert(mean_Wm,str1),convert(std_Wm,str2),convert(range_Wm,str3));
	fprintf(outSim,"mean_Wc=%s\tstd_Wc=%s\trange_Wc=%s\n",convert(mean_Wc,str1),convert(std_Wc,str2),convert(range_Wc,str3));
	fprintf(outSim,"mean_Tmax=%s\t\tstd_Tmax=%s\t\trange_Tmax=%s\n",convert(mean_Tmax,str1),convert(std_Tmax,str2),convert(range_Tmax,str3));
	fprintf(outSim,"mean_Alpha=%s\tstd_Alpha=%s\trange_Alpha=%s\n",convert(mean_Alpha,str1),convert(std_Alpha,str2),convert(range_Alpha,str3));
	fprintf(outSim,"mean_Beta=%s\t\tstd_Beta=%s\t\trange_Beta=%s\n",convert(mean_Beta,str1),convert(std_Beta,str2),convert(range_Beta,str3));
	fprintf(outSim,"mean_Tcorr=%s\tstd_Tcorr=%s\t\trange_Tcorr=%s\n",convert(mean_Tcorr,str1),convert(std_Tcorr,str2),convert(range_Tcorr,str3));
	fprintf(outSim,"mean_So=%s\t\tstd_So=%s\t\trange_So=%s\n\n",convert(mean_So,str1),convert(std_So,str2),convert(range_So,str3));

	//Lane-Changing Model Parameters: Hazard Model
	fprintf(outSim,"Hazard_ParameterA=%s\n",convert(Hazard_ParameterA,str1));
	fprintf(outSim,"Hazard_ParameterB=%s\n",convert(Hazard_ParameterB,str1));
	fprintf(outSim,"R_Heterogeneity=%s\n",convert(R_Heterogeneity,str1));
	fprintf(outSim,"K_Heterogeneity=%s\n",convert(K_Heterogeneity,str1));
	fprintf(outSim,"BetaOne_CF=%s\t\tBetaOne_FF=%s\n",convert(BetaOne_CF,str1),convert(BetaOne_FF,str2));
	fprintf(outSim,"BetaLCL_CF=%s\tBetaLCL_FF=%s\n",convert(BetaLCL_CF,str1),convert(BetaLCL_FF,str2));
	fprintf(outSim,"BetaV_CF=%s\t\tBetaV_FF=%s\n",convert(BetaV_CF,str1),convert(BetaV_FF,str2));
	fprintf(outSim,"BetaDXL1_CF=%s\tBetaDXL1_FF=%s\n",convert(BetaDXL1_CF,str1),convert(BetaDXL1_FF,str2));
	fprintf(outSim,"BetaDVL1_CF=%s\tBetaDVL1_FF=%s\n",convert(BetaDVL1_CF,str1),convert(BetaDVL1_FF,str2));
	fprintf(outSim,"BetaDXL2_CF=%s\tBetaDXL2_FF=%s\n",convert(BetaDXL2_CF,str1),convert(BetaDXL2_FF,str2));
	fprintf(outSim,"BetaDVL2_CF=%s\tBetaDVL2_FF=%s\n",convert(BetaDVL2_CF,str1),convert(BetaDVL2_FF,str2));
	fprintf(outSim,"BetaDXF1_CF=%s\tBetaDXF1_FF=%s\n",convert(BetaDXF1_CF,str1),convert(BetaDXF1_FF,str2));
	fprintf(outSim,"BetaDVF1_CF=%s\tBetaDVF1_FF=%s\n",convert(BetaDVF1_CF,str1),convert(BetaDVF1_FF,str2));
	fprintf(outSim,"BetaDXL1R_CF=%s\tBetaDXL1R_FF=%s\n",convert(BetaDXL1R_CF,str1),convert(BetaDXL1R_FF,str2));
	fprintf(outSim,"BetaDVL1R_CF=%s\tBetaDVL1R_FF=%s\n",convert(BetaDVL1R_CF,str1),convert(BetaDVL1R_FF,str2));
	fprintf(outSim,"BetaDXF1R_CF=%s\tBetaDXF1R_FF=%s\n",convert(BetaDXF1R_CF,str1),convert(BetaDXF1R_FF,str2));
	fprintf(outSim,"BetaDVF1R_CF=%s\tBetaDVF1R_FF=%s\n",convert(BetaDVF1R_CF,str1),convert(BetaDVF1R_FF,str2));
	fprintf(outSim,"BetaDXL1L_CF=%s\tBetaDXL1L_FF=%s\n",convert(BetaDXL1L_CF,str1),convert(BetaDXL1L_FF,str2));
	fprintf(outSim,"BetaDVL1L_CF=%s\tBetaDVL1L_FF=%s\n",convert(BetaDVL1L_CF,str1),convert(BetaDVL1L_FF,str2));
	fprintf(outSim,"BetaDXF1L_CF=%s\tBetaDXF1L_FF=%s\n",convert(BetaDXF1L_CF,str1),convert(BetaDXF1L_FF,str2));
	fprintf(outSim,"BetaDVF1L_CF=%s\tBetaDVF1L_FF=%s\n",convert(BetaDVF1L_CF,str1),convert(BetaDVF1L_FF,str2));
	fprintf(outSim,"BetaKR_CF=%s\t\tBetaKR_FF=%s\n",convert(BetaKR_CF,str1),convert(BetaKR_FF,str2));
	fprintf(outSim,"BetaKL_CF=%s\t\tBetaKL_FF=%s\n",convert(BetaKL_CF,str1),convert(BetaKL_FF,str2));
	fprintf(outSim,"BetaK_CF=%s\t\tBetaK_FF=%s\n\n",convert(BetaK_CF,str1),convert(BetaK_FF,str2));

	//Miscellanious Parameters
	fprintf(outSim,"meanrisk=%s\t\tstdrisk=%s\t\trangerisk=%s\n",convert(meanrisk,str1),convert(stdrisk,str2),convert(rangerisk,str3));
	fprintf(outSim,"meanLCcrash=%s\tstdLCcrash=%s\trangeLCcrash=%s\n",convert(meanLCcrash,str1),convert(stdLCcrash,str2),convert(rangeLCcrash,str3));

	fprintf(outSim,"\n");

	for (c=0; c<N; c++)
	{
		if (cars[c].Connectivity_Status == 0)
		{
			fprintf(outSim,"*************************************CAR %i***************************************\n", c+1);
			fprintf(outSim,"acc=%s\tdecc=%s\tVd=%s\t\tlane=%s\t\ts=%s\t\t\tSo=%s\n",convert(cars[c].acc,str1),convert(cars[c].decc,str2),convert(cars[c].Vd,str3),itoa(cars[c].lane,str4,10),convert(cars[c].s,str5),convert(cars[c].So,str6));
			fprintf(outSim,"risk=%s\tRt=%s\t\tLCT(timesteps)=%s\tLCcrash=%s\tdepart(timestep)=%s\n",convert(cars[c].risk,str1),convert(cars[c].Rt,str2),itoa(cars[c].LCT,str3,10),itoa(cars[c].LCcrash,str4,10),itoa(cars[c].departure,str5,10));
			fprintf(outSim,"Gamma=%s\tAlpha=%s\tBeta=%s\t\tSigma=%s\tTmax=%s\n",convert(cars[c].Gamma,str1),convert(cars[c].Alpha,str2),convert(cars[c].Beta,str3),convert(cars[c].Sigma,str4),convert(cars[c].Tmax,str5));
			fprintf(outSim,"Wm=%s\t\tWc=%s\tWp=%s\t\t\tDeltaW=%s\tTcorr=%s\n\n",convert(cars[c].Wm,str1),convert(cars[c].Wc,str2),convert(cars[c].Wp,str3),convert(cars[c].DeltaW,str4),convert(cars[c].Tcorr,str5));
		}
		else if (cars[c].Connectivity_Status == 1)
		{
			fprintf(outSim,"*************************************CAR %i***************************************\n", c+1);
			fprintf(outSim,"acc=%s\tdecc=%s\tVd=%s\t\tlane=%s\t\ts=%s\t\t\tSo=%s\n",convert(cars[c].acc,str1),convert(cars[c].decc,str2),convert(cars[c].Vd,str3),itoa(cars[c].lane,str4,10),convert(cars[c].s,str5),convert(cars[c].So,str6));
			fprintf(outSim,"risk=%s\tRt=%s\t\tLCT(timesteps)=%s\tLCcrash=%s\tdepart(timestep)=%s\n",convert(cars[c].risk,str1),convert(cars[c].Rt,str2),itoa(cars[c].LCT,str3,10),itoa(cars[c].LCcrash,str4,10),itoa(cars[c].departure,str5,10));
			fprintf(outSim,"Delta=%s\tT=%s\tSo=%s\t\ta=%s\tb=%s\tVo=%s\n",convert(cars[c].Deltan,str1),convert(cars[c].Tn,str2),convert(cars[c].Son,str3),convert(cars[c].an,str4),convert(cars[c].bn,str5), convert(cars[c].Von,str6));			
		}
	}

	return 1;

}
 


//load simulation from .sim file.
//FILE * inSim points to a textfile already open in read mode.
//car ** carPoint is the car list pointer's address.
//car ** dep is departure pointer's address.
//Returns TRUE if succeeds, FALSE if fails.
//Return -1 = fatal error (insufficient memory)


 

//int loadSim2(FILE * inSim, car ** carPoint, car ** dep)
//{
//	int a,b,c;
//	char q;
//	char str[30];
//	char str1[30];
//	char str2[30];
//	char str3[30];
//	car *cars;
//	car *p;
//
//	if (!inSim)
//	return 0;
//	 
//	//for (b=0; b<29; b++)
//	for (b=0; b<32; b++)
//	{
//		for(c=0;(q=getc(inSim))!='=';c++) //find next '=' sign
//			if (c==5000) return 0; //invalid file; '=' not found
//		for(c=0;((q=getc(inSim))!='\t')&&(q!='\n');c++) //get chars until \t or \n
//		{
//			str[c] = q;
//				if (c==29) return 0; //invalid file; '\t' or '\n' not found
//		}
//		str[c]=0; //terminate string
//		switch (b)
//		{
//			/*case 0  : { T = atoi(str); break;}
//			case 1  : { N = atoi(str); break;}
//			case 2  : { L = atof(str); break;}
//			case 3  : { meanacc = atof(str); break;}
//			case 4  : { stdacc = atof(str); break;}
//			case 5  : { rangeacc = atof(str); break;}
//			case 6  : { meandecc = atof(str); break;}
//			case 7  : { stddecc = atof(str); break;}
//			case 8  : { rangedecc = atof(str); break;}
//			case 9  : { meanVd_1 = atof(str); break;}
//			case 10 : { stdVd_1 = atof(str); break;}
//			case 11 : { rangeVd_1 = atof(str); break;}
//			case 12 : { meanVd_2 = atof(str); break;}
//			case 13 : { stdVd_2 = atof(str); break;}
//			case 14 : { rangeVd_2 = atof(str); break;}
//			case 15 : { percentVd_1 = atof(str); percentVd_2=100-percentVd_1; break;}
//			case 16 : { meanrisk = atof(str); break;}
//			case 17 : { stdrisk = atof(str); break;}
//			case 18 : { rangerisk = atof(str); break;}
//			case 19 : { meanRt = atof(str); break;}
//			case 20 : { stdRt = atof(str); break;}
//			case 21 : { rangeRt = atof(str); break;}
//			case 22 : { meanLCT = atof(str); break;}
//			case 23 : { stdLCT = atof(str); break;}
//			case 24 : { rangeLCT = atof(str); break;}
//			case 25 : { meanLCcrash = atof(str); break;}
//			case 26 : { stdLCcrash = atof(str); break;}
//			case 27 : { rangeLCcrash = atof(str); break;}
//			case 28 : { startV = atof(str); break;}*/
//
//            case 0  : { T = atoi(str); break;}
//			case 1  : { N = atoi(str); break;}
//			case 2  : { L = atof(str); break;}
//
//		  //Modified by Samer Hamdar, June 20, 2006
//            case 3  : { mean_s = atof(str); break;}
//			case 4  : { std_s = atof(str); break;}
//			case 5  : { range_s = atof(str); break;}
//
//			case 6  : { meanacc = atof(str); break;}
//			case 7  : { stdacc = atof(str); break;}
//			case 8  : { rangeacc = atof(str); break;}
//			case 9  : { meandecc = atof(str); break;}
//			case 10 : { stddecc = atof(str); break;}
//			case 11 : { rangedecc = atof(str); break;}
//			case 12 : { meanVd_1 = atof(str); break;}
//			case 13 : { stdVd_1 = atof(str); break;}
//			case 14 : { rangeVd_1 = atof(str); break;}
//			case 15 : { meanVd_2 = atof(str); break;}
//			case 16 : { stdVd_2 = atof(str); break;}
//			case 17 : { rangeVd_2 = atof(str); break;}
//			case 18 : { percentVd_1 = atof(str); percentVd_2=100-percentVd_1; break;}
//			case 19 : { meanrisk = atof(str); break;}
//			case 20 : { stdrisk = atof(str); break;}
//			case 21 : { rangerisk = atof(str); break;}
//			case 22 : { meanRt = atof(str); break;}
//			case 23 : { stdRt = atof(str); break;}
//			case 24 : { rangeRt = atof(str); break;}
//			case 25 : { meanLCT = atof(str); break;}
//			case 26 : { stdLCT = atof(str); break;}
//			case 27 : { rangeLCT = atof(str); break;}
//			case 28 : { meanLCcrash = atof(str); break;}
//			case 29 : { stdLCcrash = atof(str); break;}
//			case 30 : { rangeLCcrash = atof(str); break;}
//			case 31 : { startV = atof(str); break;}
//		}
//	}
//
//	NUM_AGG_TIMEINTERVALS = T/AGG_TIMEINTERVAL;
//
//	NUM_SEGMENTS = L/AGG_SEGMENT_LENGTH; 
//	NUM_AGG_TIMEINTERVALS_SEGMENT = T/AGG_TIMEINTERVAL_SEGMENT;
//
//
//	*carPoint = (car *) malloc(sizeof(car) * N);
//	cars = *carPoint;
//
//	if (cars == NULL) //memory allocation problem
//	{
//		cprintf("Insufficient memory or memory allocation problem.");
//		cprintf("Program terminating. Press any key...");
//		getch();
//		return -1;
//	}
//
//	AggCars agg_car;
//	for (a=0; a<N; a++)
//	{
//		for (b=0; b<10; b++)
//		{
//			for(c=0;(q=getc(inSim))!='=';c++) //find next '=' sign
//				if (c==5000) return 0; //invalid file; '=' not found
//			for(c=0;((q=getc(inSim))!='\t')&&(q!='\n');c++) //get chars until \t or \n
//			{
//				str[c] = q;
//					if (c==29) return 0; //invalid file; '\t' or '\n' not found
//			}
//			str[c]=0; //terminate string
//			switch (b)
//			{
//				case 0  : { cars[a].acc = atof(str); break;}
//				case 1  : { cars[a].decc = atof(str); break;}
//				case 2  : { cars[a].Vd = atof(str); break;}
//				case 3  : { cars[a].lane = atoi(str); break;}
//				case 4  : { cars[a].s = atof(str); break;}
//				case 5  : { cars[a].risk = atof(str); break;}
//				case 6  : { cars[a].Rt = atof(str); break;}
//				case 7  : { cars[a].LCT = atoi(str); break;}
//				case 8  : { cars[a].LCcrash = atoi(str); break;}
//				case 9  : { cars[a].departure = atoi(str); break;}
//			}
//		}
//
//
//
//		cars[a].x = 0.0;
//		cars[a].v = startV;
//		cars[a].l = 0;
//		cars[a].carID = a+1;
//		cars[a].crash = 0;
//		cars[a].previous = 0;
//		cars[a].next = 0;
//		cars[a].headway = -9999.0;
//		cars[a].react = (int) (cars[a].Rt * 10.0); //ten times the reaction time in seconds
//
//		 
//		cars[a].dt_prev = cars[a].departure;
//		cars[a].x_prev = 0.0;
//		cars[a].crash_prev = 0;
//		cars[a].lane_prev = cars[a].lane;
//		cars[a].v_prev = 0.0;
//		cars[a].decc_original = cars[a].decc;
//		cars[a].risk_original = cars[a].risk;
//		cars[a].Vd_original = cars[a].Vd;
//
//		agg_car.CarID = cars[a].carID;
//		AggCarsData.push_back(agg_car);
//
//		if (a==0)
//		{
//			*dep = &cars[a];
//		}
//		else
//		{
//			p = *dep;
//
//			if (cars[a].departure < p->departure) //car has earliest departure time so far
//			{                     //=> insert at beginning of list
//				cars[a].next = *dep;
//				(*dep)->previous = &cars[a];
//				*dep = &cars[a];
//			}
//			else
//			{
//				//find place to insert car in departure list
//				while (  (p->next!=NULL) && ((p->next->departure) < cars[a].departure)  )
//					p = p->next;
//
//				//insert car
//				cars[a].previous = p;
//				cars[a].next = p->next;
//				if (p->next!=NULL)
//					p->next->previous = &cars[a];
//				p->next = &cars[a];
//			}
//		}
//
//	}
//	
//	SegmentAggregateData segment_aggregate_data;
//	MacroAggregateSegmentLane lane_data;
//	lane_data.DensityAtLane = 0;
//	lane_data.FlowRateAtLane = 0;
//	lane_data.NumOfAccidentsAtLane = 0;
//	lane_data.NumOfCarsPassAtLane = 0;
//	lane_data.NumOfCarsUseAtLane = 0;
//	lane_data.SpeedAverageAtLane = 0;
//	lane_data.TravelTimeAtLane = 0;
//	lane_data.HeadwayAtLane = 0;
//	lane_data.NumOfCarsForHeadway = 0;
//	lane_data.NumOfCarsForSpeed = 0;
//	lane_data.NumOfLaneChanges = 0;
//	
//	for (int i=0;i<NUM_SEGMENTS;i++)
//	{
//		segment_aggregate_data.SegmentID = i+1;
//		segment_aggregate_data.NumOfLanesAtSegment = NUM_LANES;
//		segment_aggregate_data.DensityAtSegment = 0;
//		segment_aggregate_data.FlowRateAtSegment = 0;
//		segment_aggregate_data.NumOfLaneChangesAtSegment = 0;
//		segment_aggregate_data.NumOfLanesAtSegment = 0;
//		segment_aggregate_data.SpeedAverageAtSegment = 0;
//		segment_aggregate_data.TravelTimeAtSegment = 0;
//
//		AggregateSegments.push_back(segment_aggregate_data);
//
//		for (int j=0;j<NUM_AGG_TIMEINTERVALS_SEGMENT;j++)
//		{
//			AggregateSegments[i].Lane0.push_back(lane_data);
//			AggregateSegments[i].Lane1.push_back(lane_data);
//		}
//	}
//	return 1;
//}


 
int loadSim2(FILE * inSim, car ** carPoint, car ** dep)
{
	int a,b,c;
	char q;
	char str[30];	

	if (!inSim)
	return 0;

	for (b=0; b<100; b++)
	{
		for(c=0;(q=getc(inSim))!='=';c++) //find next '=' sign
			if (c==5000) return 0; //invalid file; '=' not found
		for(c=0;((q=getc(inSim))!='\t')&&(q!='\n');c++) //get chars until \t or \n
		{
			str[c] = q;
				if (c==29) return 0; //invalid file; '\t' or '\n' not found
		}
		str[c]=0; //terminate string
		switch (b)
		{
			//Simulation Related Parameters
            case 0  : { T = atoi(str); break;}
			case 1  : { N = atoi(str); break;}
			case 2  : { L = atof(str); break;}

			case 3 :  { startV = atof(str); break;}

			//Vehicle/Driver Specific Parameters
            case 4  : { mean_s = atof(str); break;}
			case 5  : { std_s = atof(str); break;}
			case 6  : { range_s = atof(str); break;}

			case 7  : { meanacc = atof(str); break;}
			case 8  : { stdacc = atof(str); break;}
			case 9  : { rangeacc = atof(str); break;}

			case 10  : { meandecc = atof(str); break;}
			case 11 : { stddecc = atof(str); break;}
			case 12 : { rangedecc = atof(str); break;}

			case 13 : { meanVd_1 = atof(str); break;}
			case 14 : { stdVd_1 = atof(str); break;}
			case 15 : { rangeVd_1 = atof(str); break;}

			case 16 : { meanVd_2 = atof(str); break;}
			case 17 : { stdVd_2 = atof(str); break;}
			case 18 : { rangeVd_2 = atof(str); break;}

			case 19 : { percentVd_1 = atof(str); percentVd_2=100-percentVd_1; break;}			

			case 20 : { meanRt = atof(str); break;}
			case 21 : { stdRt = atof(str); break;}
			case 22 : { rangeRt = atof(str); break;}

			case 23 : { meanLCT = atof(str); break;}
			case 24 : { stdLCT = atof(str); break;}
			case 25 : { rangeLCT = atof(str); break;}

			//PT Model Parameters
		  	case 26 :  { mean_Gamma = atof(str); break;}
			case 27  : { std_Gamma = atof(str); break;}
			case 28  : { range_Gamma = atof(str); break;}

		   	case 29 :  { mean_Wm = atof(str); break;}
			case 30  : { std_Wm = atof(str); break;}
			case 31  : { range_Wm = atof(str); break;}

		   	case 32 :  { mean_Wc = atof(str); break;}
			case 33  : { std_Wc = atof(str); break;}
			case 34  : { range_Wc = atof(str); break;}

		   	case 35 :  { mean_Tmax = atof(str); break;}
			case 36  : { std_Tmax = atof(str); break;}
			case 37  : { range_Tmax = atof(str); break;}

			case 38 :  { mean_Alpha = atof(str); break;}
			case 39  : { std_Alpha = atof(str); break;}
			case 40  : { range_Alpha = atof(str); break;}

		   	case 41 :  { mean_Beta = atof(str); break;}
			case 42  : { std_Beta = atof(str); break;}
			case 43  : { range_Beta = atof(str); break;}

			case 44 :  { mean_Tcorr = atof(str); break;}
			case 45  : { std_Tcorr = atof(str); break;}
			case 46  : { range_Tcorr = atof(str); break;}

		   	case 47 :  { mean_So = atof(str); break;}
			case 48  : { std_So = atof(str); break;}
			case 49  : { range_So = atof(str); break;}

			//Hazard-Based Model Parameters
		   	case 50 :  { Hazard_ParameterA = atof(str); break;}

			case 51  : { Hazard_ParameterB = atof(str); break;}

			case 52  : { R_Heterogeneity = atof(str); break;}

			case 53 :  { K_Heterogeneity = atof(str); break;}

			case 54  : { BetaOne_CF = atof(str); break;}
			case 55  : { BetaOne_FF = atof(str); break;}
			
			case 56  : { BetaLCL_CF = atof(str); break;}
			case 57  : { BetaLCL_FF = atof(str); break;}

		   	case 58  : { BetaV_CF = atof(str); break;}
			case 59  : { BetaV_FF = atof(str); break;}

		   	case 60  : { BetaDXL1_CF = atof(str); break;}
		    case 61  : { BetaDXL1_FF = atof(str); break;}

			case 62  : { BetaDVL1_CF = atof(str); break;}
		    case 63  : { BetaDVL1_FF = atof(str); break;}

		   	case 64  : { BetaDXL2_CF = atof(str); break;}
		    case 65  : { BetaDXL2_FF = atof(str); break;}

			case 66  : { BetaDVL2_CF = atof(str); break;}
		    case 67  : { BetaDVL2_FF = atof(str); break;}

			case 68  : { BetaDXF1_CF = atof(str); break;}
		    case 69  : { BetaDXF1_FF = atof(str); break;}

			case 70  : { BetaDVF1_CF = atof(str); break;}
		    case 71  : { BetaDVF1_FF = atof(str); break;}

		   	case 72  : { BetaDXL1R_CF = atof(str); break;}
            case 73  : { BetaDXL1R_FF = atof(str); break;}

			case 74  : { BetaDVL1R_CF = atof(str); break;}
			case 75  : { BetaDVL1R_FF = atof(str); break;}

		   	case 76  : { BetaDXF1R_CF = atof(str); break;}
		    case 77  : { BetaDXF1R_FF = atof(str); break;}

			case 78  : { BetaDVF1R_CF = atof(str); break;}
		    case 79  : { BetaDVF1R_FF = atof(str); break;}

		   	case 80  : { BetaDXL1L_CF = atof(str); break;}
		    case 81  : { BetaDXL1L_FF = atof(str); break;}

			case 82  : { BetaDVL1L_CF = atof(str); break;}
            case 83  : { BetaDVL1L_FF = atof(str); break;}

		   	case 84  : { BetaDXF1L_CF = atof(str); break;}
			case 85  : { BetaDXF1L_FF = atof(str); break;}

			case 86  : { BetaDVF1L_CF = atof(str); break;}
		    case 87  : { BetaDVF1L_FF = atof(str); break;}

		   	case 88  : { BetaKR_CF = atof(str); break;}
		    case 89  : { BetaKR_FF = atof(str); break;}

			case 90  : { BetaKL_CF = atof(str); break;}
		    case 91  : { BetaKL_FF = atof(str); break;}

		   	case 92  : { BetaK_CF = atof(str); break;}
			case 93  : { BetaK_FF = atof(str); break;}

			//Miscellanious Parameters
			case 94 : { meanrisk = atof(str); break;}
			case 95 : { stdrisk = atof(str); break;}
			case 96 : { rangerisk = atof(str); break;}

			case 97 : { meanLCcrash = atof(str); break;}
			case 98 : { stdLCcrash = atof(str); break;}
			case 99 : { rangeLCcrash = atof(str); break;}
		}
	}


	for (a=0; a<35; a++)
	{
		for (b=0; b<14; b++)
		{
			for(c=0;(q=getc(inSim))!='=';c++) //find next '=' sign
				if (c==5000) return 0; //invalid file; '=' not found
			for(c=0;((q=getc(inSim))!='\t')&&(q!='\n');c++) //get chars until \t or \n
			{
				str[c] = q;
					if (c==29) return 0; //invalid file; '\t' or '\n' not found
			}
			str[c]=0; //terminate string
			switch (b)
			{
				case 0  : { tempDriver.Rt = atof(str); break;}
				case 1  : { tempDriver.Gamma = atof(str); break;}
				case 2  : { tempDriver.Alpha = atof(str); break;}
				case 3  : { tempDriver.Beta = atoi(str); break;}
				case 4  : { tempDriver.Tmax = atof(str); break;}
				case 5  : { tempDriver.Wm = atof(str); break;}
				case 6  : { tempDriver.Wc = atof(str); break;}
				case 7  : { tempDriver.Tcorr = atof(str); break;}
				case 8  : { tempDriver.Deltan = atof(str); break;}
				case 9  : { tempDriver.Tn = atof(str); break;}
				case 10  : { tempDriver.Son = atof(str); break;}
				case 11  : { tempDriver.an = atof(str); break;}
				case 12  : { tempDriver.bn = atof(str); break;}
				case 13  : { tempDriver.Von = atof(str); break;}
			}
		}
		Drivers.push_back(tempDriver);
	}
	return 1;
}
 


int loadSim1(FILE * inSim, car ** carPoint, car ** dep)
{
	int b,c;
	char q;
	char str[30];
	car *cars;	

	if (!inSim)
	return 0;
	 
	//for (b=0; b<29; b++)
	for (b=0; b<100; b++)
	{
		for(c=0;(q=getc(inSim))!='=';c++) //find next '=' sign
			if (c==5000) return 0; //invalid file; '=' not found
		for(c=0;((q=getc(inSim))!='\t')&&(q!='\n');c++) //get chars until \t or \n
		{
			str[c] = q;
				if (c==29) return 0; //invalid file; '\t' or '\n' not found
		}
		str[c]=0; //terminate string
		switch (b)
		{
			//Simulation Related Parameters
            case 0  : { T = atoi(str); break;}
			case 1  : { N = atoi(str); break;}
			case 2  : { L = atof(str); break;}

			case 3 :  { startV = atof(str); break;}

			//Vehicle/Driver Specific Parameters
            case 4  : { mean_s = atof(str); break;}
			case 5  : { std_s = atof(str); break;}
			case 6  : { range_s = atof(str); break;}

			case 7  : { meanacc = atof(str); break;}
			case 8  : { stdacc = atof(str); break;}
			case 9  : { rangeacc = atof(str); break;}

			case 10  : { meandecc = atof(str); break;}
			case 11 : { stddecc = atof(str); break;}
			case 12 : { rangedecc = atof(str); break;}

			case 13 : { meanVd_1 = atof(str); break;}
			case 14 : { stdVd_1 = atof(str); break;}
			case 15 : { rangeVd_1 = atof(str); break;}

			case 16 : { meanVd_2 = atof(str); break;}
			case 17 : { stdVd_2 = atof(str); break;}
			case 18 : { rangeVd_2 = atof(str); break;}

			case 19 : { percentVd_1 = atof(str); percentVd_2=100-percentVd_1; break;}			

			case 20 : { meanRt = atof(str); break;}
			case 21 : { stdRt = atof(str); break;}
			case 22 : { rangeRt = atof(str); break;}

			case 23 : { meanLCT = atof(str); break;}
			case 24 : { stdLCT = atof(str); break;}
			case 25 : { rangeLCT = atof(str); break;}

			//PT Model Parameters
		  	case 26 :  { mean_Gamma = atof(str); break;}
			case 27  : { std_Gamma = atof(str); break;}
			case 28  : { range_Gamma = atof(str); break;}

		   	case 29 :  { mean_Wm = atof(str); break;}
			case 30  : { std_Wm = atof(str); break;}
			case 31  : { range_Wm = atof(str); break;}

		   	case 32 :  { mean_Wc = atof(str); break;}
			case 33  : { std_Wc = atof(str); break;}
			case 34  : { range_Wc = atof(str); break;}

		   	case 35 :  { mean_Tmax = atof(str); break;}
			case 36  : { std_Tmax = atof(str); break;}
			case 37  : { range_Tmax = atof(str); break;}

			case 38 :  { mean_Alpha = atof(str); break;}
			case 39  : { std_Alpha = atof(str); break;}
			case 40  : { range_Alpha = atof(str); break;}

		   	case 41 :  { mean_Beta = atof(str); break;}
			case 42  : { std_Beta = atof(str); break;}
			case 43  : { range_Beta = atof(str); break;}

			case 44 :  { mean_Tcorr = atof(str); break;}
			case 45  : { std_Tcorr = atof(str); break;}
			case 46  : { range_Tcorr = atof(str); break;}

		   	case 47 :  { mean_So = atof(str); break;}
			case 48  : { std_So = atof(str); break;}
			case 49  : { range_So = atof(str); break;}

			//Hazard-Based Model Parameters
		   	case 50 :  { Hazard_ParameterA = atof(str); break;}

			case 51  : { Hazard_ParameterB = atof(str); break;}

			case 52  : { R_Heterogeneity = atof(str); break;}

			case 53 :  { K_Heterogeneity = atof(str); break;}

			case 54  : { BetaOne_CF = atof(str); break;}
			case 55  : { BetaOne_FF = atof(str); break;}
			
			case 56  : { BetaLCL_CF = atof(str); break;}
			case 57  : { BetaLCL_FF = atof(str); break;}

		   	case 58  : { BetaV_CF = atof(str); break;}
			case 59  : { BetaV_FF = atof(str); break;}

		   	case 60  : { BetaDXL1_CF = atof(str); break;}
		    case 61  : { BetaDXL1_FF = atof(str); break;}

			case 62  : { BetaDVL1_CF = atof(str); break;}
		    case 63  : { BetaDVL1_FF = atof(str); break;}

		   	case 64  : { BetaDXL2_CF = atof(str); break;}
		    case 65  : { BetaDXL2_FF = atof(str); break;}

			case 66  : { BetaDVL2_CF = atof(str); break;}
		    case 67  : { BetaDVL2_FF = atof(str); break;}

			case 68  : { BetaDXF1_CF = atof(str); break;}
		    case 69  : { BetaDXF1_FF = atof(str); break;}

			case 70  : { BetaDVF1_CF = atof(str); break;}
		    case 71  : { BetaDVF1_FF = atof(str); break;}

		   	case 72  : { BetaDXL1R_CF = atof(str); break;}
            case 73  : { BetaDXL1R_FF = atof(str); break;}

			case 74  : { BetaDVL1R_CF = atof(str); break;}
			case 75  : { BetaDVL1R_FF = atof(str); break;}

		   	case 76  : { BetaDXF1R_CF = atof(str); break;}
		    case 77  : { BetaDXF1R_FF = atof(str); break;}

			case 78  : { BetaDVF1R_CF = atof(str); break;}
		    case 79  : { BetaDVF1R_FF = atof(str); break;}

		   	case 80  : { BetaDXL1L_CF = atof(str); break;}
		    case 81  : { BetaDXL1L_FF = atof(str); break;}

			case 82  : { BetaDVL1L_CF = atof(str); break;}
            case 83  : { BetaDVL1L_FF = atof(str); break;}

		   	case 84  : { BetaDXF1L_CF = atof(str); break;}
			case 85  : { BetaDXF1L_FF = atof(str); break;}

			case 86  : { BetaDVF1L_CF = atof(str); break;}
		    case 87  : { BetaDVF1L_FF = atof(str); break;}

		   	case 88  : { BetaKR_CF = atof(str); break;}
		    case 89  : { BetaKR_FF = atof(str); break;}

			case 90  : { BetaKL_CF = atof(str); break;}
		    case 91  : { BetaKL_FF = atof(str); break;}

		   	case 92  : { BetaK_CF = atof(str); break;}
			case 93  : { BetaK_FF = atof(str); break;}

			//Miscellanious Parameters
			case 94 : { meanrisk = atof(str); break;}
			case 95 : { stdrisk = atof(str); break;}
			case 96 : { rangerisk = atof(str); break;}

			case 97 : { meanLCcrash = atof(str); break;}
			case 98 : { stdLCcrash = atof(str); break;}
			case 99 : { rangeLCcrash = atof(str); break;}
		}
	}

	*carPoint = (car *) malloc(sizeof(car) * N);
	cars = *carPoint;

	if (cars == NULL) //memory allocation problem
	{
		return -1;
	}

	return 1;
}

 

//bubblesort algorithm on cars to sort by departure times
//cars are not physically moved, but indexes are created in carDeparting[]
//to index cars in order of departure time.

void arrangeCars(car * cars, int * carDeparting)
{
	int c,l,r;
	for (c=0; c<N; c++)
		carDeparting[c] = c;

	for (r=0; r<N; r++)
	{
//		for (l=N-1; l>=r; l--)
		for (l=N-2; l>=r; l--)
		{
			if (cars[carDeparting[l+1]].departure < cars[carDeparting[l]].departure)
			{
				//switch
				c = carDeparting[l];
				carDeparting[l] = carDeparting[l+1];
				carDeparting[l+1] = c;
			}
		}
	}
}

void traverseLane(car * L)
{
	int count=0;
//	while (L!=NULL && count<50)
	while (L!=NULL)
	{
		cout<<"|ID:"<<L->carID<<" X:"<<L->x<<" V:"<<L->v<<" L:"<<L->lane<<"|->";
		L=L->next;
		count++;
		if (count%2 == 0)
			cout<<endl;
	}
	if (count == 50)
		cout<<"OVERFLOW!!"<<endl;
	cout<<"press any key..."<<endl;
	getch();
}


 
struct car* CopyStream (car *r1)
{
	car *r_temp;
	car *r_temp2;
	r_temp = NULL;
	r_temp2 = NULL;
	
	while (r1 != NULL)
	{
		if (r_temp == NULL)
		{
			r_temp= (car*)malloc(sizeof(car));

			// copy data
			r_temp->Connectivity_Status = r1->Connectivity_Status;
			r_temp->Deltan = r1->Deltan;
			r_temp->Tn = r1->Tn;
			r_temp->Son = r1->Son;
			r_temp->an = r1->an;
			r_temp->bn = r1->bn;
			r_temp->Von = r1->Von;
			r_temp->VSLFalg = r1->VSLFalg;
			r_temp->acc = r1->acc; 
			r_temp->decc = r1->decc; 
			r_temp->Vd = r1->Vd; 
			r_temp->lane = r1->lane;
			r_temp->l = r1->l; 
			r_temp->a_update = r1->a_update;
			r_temp->s = r1->s; 
			r_temp->x = r1->x; 
			r_temp->v = r1->v; 
			r_temp->Deltax = r1->Deltax;
			r_temp->headway = r1->headway;
			r_temp->DV = r1->DV;
			r_temp->Spacing = r1->Spacing;
			r_temp->risk = r1->risk;
			r_temp->Rt = r1->Rt;
			r_temp->react = r1->react;
			r_temp->LCT = r1->LCT; 
			r_temp->LCcrash = r1->LCcrash; 
			r_temp->departure = r1->departure;
			r_temp->carID = r1->carID;
			r_temp->crash = r1->crash;
			r_temp->crashtimecounter_ramp = r1->crashtimecounter_ramp;
			r_temp->crashtimecounter = r1->crashtimecounter;
			r_temp->x_prev = r1->x_prev;
			r_temp->dt_prev = r1->dt_prev;
			r_temp->crash_prev = r1->crash_prev;
			r_temp->lane_prev = r1->lane_prev;
			r_temp->v_prev = r1->v_prev;
			//r_temp->risk_original = r1->risk_original;
			//r_temp->decc_original = r1->decc_original;
			//r_temp->Vd_original = r1->Vd_original;
			r_temp->Yt = r1->Yt;
			r_temp->Yt_previous = r1->Yt_previous;
			r_temp->Leader_Previous = r1->Leader_Previous;	
			r_temp->Gamma = r1->Gamma;
			r_temp->Wm = r1->Wm;
			r_temp->Wc = r1->Wc;
			r_temp->Tmax = r1->Tmax;
			r_temp->Alpha = r1->Alpha;
			r_temp->Beta = r1->Beta;
			r_temp->Tcorr = r1->Tcorr;
			//r_temp->Amax = r1->Amax;
			//r_temp->Vo = r1->Vo;
			r_temp->So = r1->So;
			//r_temp->Ao = r1->Ao;
			r_temp->Wp = r1->Wp;
			r_temp->DeltaW = r1->DeltaW;
			r_temp->Sigma = r1->Sigma;
			r_temp->Dummy_Shockwave = r1->Dummy_Shockwave;
			r_temp->LCL = r1->LCL;
			//r_temp->DXL1 = r1->DXL1;
			//r_temp->DVL1 = r1->DVL1;
			//r_temp->DXL2 = r1->DXL2;
			//r_temp->DVL2 = r1->DVL2;
			//r_temp->DXF1 = r1->DXF1;
			//r_temp->DVF1 = r1->DVF1;
			//r_temp->DXL1R = r1->DXL1R;
			//r_temp->DVL1R = r1->DVL1R;
			//r_temp->DXF1R = r1->DXF1R;
			//r_temp->DVF1R = r1->DVF1R;
			//r_temp->DXL1L = r1->DXL1L;
			//r_temp->DVL1L = r1->DVL1L;
			//r_temp->DXF1L = r1->DXF1L;
			//r_temp->DVF1L = r1->DVF1L;
			//r_temp->KR = r1->KR;
			//r_temp->KL = r1->KL;
			//r_temp->K = r1->K;
			r_temp->Duration = r1->Duration;
			r_temp->Mode = r1->Mode;
			r_temp->PreviousLeaderID = r1->PreviousLeaderID;
			r_temp->GammaHeterogeneity = r1->GammaHeterogeneity;	
			r_temp->RampDummy = r1->RampDummy;
			r_temp->LeftLaneDummy = r1->LeftLaneDummy;
			r_temp->RightLaneDummy = r1->RightLaneDummy;
			r_temp->DistanceToRamp = r1->DistanceToRamp;
			r_temp->Hazard = r1->Hazard;
			r_temp->DummyDXL1 = r1->DummyDXL1;
			r_temp->DummyDVL1 = r1->DummyDVL1;
			r_temp->lanechangingflag = r1->lanechangingflag;
			r_temp->basehazard = r1->basehazard;
			r_temp->ComplianceValue = r1->ComplianceValue;
			r_temp->Compliance = r1->Compliance;
			r_temp->overspeedingrate = r1->overspeedingrate; 
			r_temp->Crash_Probability = r1->Crash_Probability;
			r_temp->changelane_direction = r1->changelane_direction;
			r_temp->targetrampside = r1->targetrampside;
			r_temp->destlane = r1->destlane;
			r_temp->changelaneinstance = r1->changelaneinstance;			
			r_temp->Connectivity_Status = r1->Connectivity_Status;
			r_temp->Deltan = r1->Deltan;
			r_temp->Tn = r1->Tn;
			r_temp->Son = r1->Son;
			r_temp->an = r1->an;
			r_temp->bn = r1->bn;
			r_temp->Von = r1->Von;
			r_temp->K = r1->K;
			r_temp->KPrime = r1->KPrime;
			r_temp->Ka = r1->Ka;
			r_temp->Kv = r1->Kv;
			r_temp->Kd = r1->Kd;
			// end of copy data
			
			r_temp2 = r_temp;
			r_temp->previous = NULL;
			r_temp->next = NULL;	

		}
		else
		{
			r_temp->next = (car*)malloc(sizeof(car));
			r_temp = r_temp->next;
			r_temp->previous=NULL;
			r_temp->previous = r_temp2;
		
			// copy data
			r_temp->Connectivity_Status = r1->Connectivity_Status;
			r_temp->Deltan = r1->Deltan;
			r_temp->Tn = r1->Tn;
			r_temp->Son = r1->Son;
			r_temp->an = r1->an;
			r_temp->bn = r1->bn;
			r_temp->Von = r1->Von;
			r_temp->VSLFalg = r1->VSLFalg;
			r_temp->acc = r1->acc; 
			r_temp->decc = r1->decc; 
			r_temp->Vd = r1->Vd; 
			r_temp->lane = r1->lane;
			r_temp->l = r1->l; 
			r_temp->a_update = r1->a_update;
			r_temp->s = r1->s; 
			r_temp->x = r1->x; 
			r_temp->v = r1->v; 
			r_temp->Deltax = r1->Deltax;
			r_temp->headway = r1->headway;
			r_temp->DV = r1->DV;
			r_temp->Spacing = r1->Spacing;
			r_temp->risk = r1->risk;
			r_temp->Rt = r1->Rt;
			r_temp->react = r1->react;
			r_temp->LCT = r1->LCT; 
			r_temp->LCcrash = r1->LCcrash; 
			r_temp->departure = r1->departure;
			r_temp->carID = r1->carID;
			r_temp->crash = r1->crash;
			r_temp->crashtimecounter_ramp = r1->crashtimecounter_ramp;
			r_temp->crashtimecounter = r1->crashtimecounter;
			r_temp->x_prev = r1->x_prev;
			r_temp->dt_prev = r1->dt_prev;
			r_temp->crash_prev = r1->crash_prev;
			r_temp->lane_prev = r1->lane_prev;
			r_temp->v_prev = r1->v_prev;
			//r_temp->risk_original = r1->risk_original;
			//r_temp->decc_original = r1->decc_original;
			//r_temp->Vd_original = r1->Vd_original;
			r_temp->Yt = r1->Yt;
			r_temp->Yt_previous = r1->Yt_previous;
			r_temp->Leader_Previous = r1->Leader_Previous;	
			r_temp->Gamma = r1->Gamma;
			r_temp->Wm = r1->Wm;
			r_temp->Wc = r1->Wc;
			r_temp->Tmax = r1->Tmax;
			r_temp->Alpha = r1->Alpha;
			r_temp->Beta = r1->Beta;
			r_temp->Tcorr = r1->Tcorr;
			//r_temp->Amax = r1->Amax;
			//r_temp->Vo = r1->Vo;
			r_temp->So = r1->So;
			//r_temp->Ao = r1->Ao;
			r_temp->Wp = r1->Wp;
			r_temp->DeltaW = r1->DeltaW;
			r_temp->Sigma = r1->Sigma;
			r_temp->Dummy_Shockwave = r1->Dummy_Shockwave;
			r_temp->LCL = r1->LCL;
			//r_temp->DXL1 = r1->DXL1;
			//r_temp->DVL1 = r1->DVL1;
			//r_temp->DXL2 = r1->DXL2;
			//r_temp->DVL2 = r1->DVL2;
			//r_temp->DXF1 = r1->DXF1;
			//r_temp->DVF1 = r1->DVF1;
			//r_temp->DXL1R = r1->DXL1R;
			//r_temp->DVL1R = r1->DVL1R;
			//r_temp->DXF1R = r1->DXF1R;
			//r_temp->DVF1R = r1->DVF1R;
			//r_temp->DXL1L = r1->DXL1L;
			//r_temp->DVL1L = r1->DVL1L;
			//r_temp->DXF1L = r1->DXF1L;
			//r_temp->DVF1L = r1->DVF1L;
			//r_temp->KR = r1->KR;
			//r_temp->KL = r1->KL;
			//r_temp->K = r1->K;
			r_temp->Duration = r1->Duration;
			r_temp->Mode = r1->Mode;
			r_temp->PreviousLeaderID = r1->PreviousLeaderID;
			r_temp->GammaHeterogeneity = r1->GammaHeterogeneity;	
			r_temp->RampDummy = r1->RampDummy;
			r_temp->LeftLaneDummy = r1->LeftLaneDummy;
			r_temp->RightLaneDummy = r1->RightLaneDummy;
			r_temp->DistanceToRamp = r1->DistanceToRamp;
			r_temp->Hazard = r1->Hazard;
			r_temp->DummyDXL1 = r1->DummyDXL1;
			r_temp->DummyDVL1 = r1->DummyDVL1;
			r_temp->lanechangingflag = r1->lanechangingflag;
			r_temp->basehazard = r1->basehazard;
			r_temp->ComplianceValue = r1->ComplianceValue;
			r_temp->Compliance = r1->Compliance;
			r_temp->overspeedingrate = r1->overspeedingrate; 
			r_temp->Crash_Probability = r1->Crash_Probability;
			r_temp->changelane_direction = r1->changelane_direction;
			r_temp->targetrampside = r1->targetrampside;
			r_temp->destlane = r1->destlane;
			r_temp->changelaneinstance = r1->changelaneinstance;
			r_temp->Connectivity_Status = r1->Connectivity_Status;
			r_temp->Deltan = r1->Deltan;
			r_temp->Tn = r1->Tn;
			r_temp->Son = r1->Son;
			r_temp->an = r1->an;
			r_temp->bn = r1->bn;
			r_temp->Von = r1->Von;
			r_temp->K = r1->K;
			r_temp->KPrime = r1->KPrime;
			r_temp->Ka = r1->Ka;
			r_temp->Kv = r1->Kv;
			r_temp->Kd = r1->Kd;
			// end of copy data
			//r_temp->previous = r1->previous;
			r_temp2 = r_temp;
			r_temp->next = NULL;
			
		};
		r1 = r1->next;	
	};

	while (r_temp->previous != NULL)
	{
		r_temp = r_temp->previous;
	};


	
	/*while (r2->next->next != NULL)
	{
		r2 = r2->next;
	};

	while (r2 != NULL)
	{
			r_temp->previous= (car*)malloc(sizeof(car));
			r_temp = r_temp->previous;
		
			// copy data
			r_temp->acc = r2->acc;
			r_temp->decc = r2->decc;
			r_temp->Vd = r2->Vd; 
			r_temp->lane = r2->lane;
			r_temp->l = r2->l;
			r_temp->a_update = r2->a_update;
			r_temp->s = r2->s;
			r_temp->x = r2->x;
			r_temp->v = r2->v;
			r_temp->headway = r2->headway;
			r_temp->DV = r2->DV;
			r_temp->Spacing = r2->Spacing;
			r_temp->risk = r2->risk;
			r_temp->Rt = r2->Rt;
			r_temp->react = r2->react;
			r_temp->LCT = r2->LCT;
			r_temp->LCcrash = r2->LCcrash;
			r_temp->departure = r2->departure;
			r_temp->carID = r2->carID;
			r_temp->crash = r2->crash;
			r_temp->crashtimecounter_ramp = r2->crashtimecounter_ramp;
			r_temp->x_prev = r2->x_prev;
			r_temp->dt_prev = r2->dt_prev;
			r_temp->crash_prev = r2->crash_prev;
			r_temp->lane_prev = r2->lane_prev;
			r_temp->v_prev = r2->v_prev;
			r_temp->risk_original = r2->risk_original;
			r_temp->decc_original = r2->decc_original;
			r_temp->Vd_original = r2->Vd_original;
			r_temp->Yt = r2->Yt;
			r_temp->Yt_previous = r2->Yt_previous;
			r_temp->Leader_Previous = r2->Leader_Previous;
			r_temp->Gamma = r2->Gamma;
			r_temp->Wm = r2->Wm;
			r_temp->Wc = r2->Wc;
			r_temp->Tmax = r2->Tmax;
			r_temp->Alpha = r2->Alpha;
			r_temp->Beta = r2->Beta;
			r_temp->Tcorr = r2->Tcorr;
			r_temp->Amax = r2->Amax;
			r_temp->Vo = r2->Vo;
			r_temp->So = r2->So;
			r_temp->Ao = r2->Ao;
			r_temp->Wp = r2->Wp;
			r_temp->DeltaW =r2->DeltaW;
			r_temp->Sigma = r2->Sigma;
			r_temp->Dummy_Shockwave = r2->Dummy_Shockwave;
			r_temp->LCL = r2->LCL;
			r_temp->DXL1 = r2->DXL1;
			r_temp->DVL1 = r2->DVL1;
			r_temp->DXL2 = r2->DXL2;
			r_temp->DVL2 = r2->DVL2;
			r_temp->DXF1 = r2->DXF1;
			r_temp->DVF1 = r2->DVF1;
			r_temp->DXL1R = r2->DXL1R;
			r_temp->DVL1R = r2->DVL1R;
			r_temp->DXF1R = r2->DXF1R;
			r_temp->DVF1R = r2->DVF1R;
			r_temp->DXL1L = r2->DXL1L;
			r_temp->DVL1L = r2->DVL1L;
			r_temp->DXF1L = r2->DXF1L;
			r_temp->DVF1L = r2->DVF1L;
			r_temp->KR = r2->KR;
			r_temp->KL = r2->KL;
			r_temp->K = r2->K;
			r_temp->Duration = r2->Duration;
			r_temp->Mode = r2->Mode; 
			r_temp->PreviousLeaderID = r2->PreviousLeaderID;
			r_temp->GammaHeterogeneity = r2->GammaHeterogeneity;
			r_temp->RampDummy = r2->RampDummy;
			r_temp->LeftLaneDummy = r2->LeftLaneDummy;
			r_temp->RightLaneDummy = r2->RightLaneDummy;
			r_temp->DistanceToRamp = r2->DistanceToRamp; 
			r_temp->Hazard = r2->Hazard;
			r_temp->DummyDXL1 = r2->DummyDXL1;
			r_temp->DummyDVL1 = r2->DummyDVL1;
			// end of copy data
			r_temp->previous=NULL;
			
		
		r2 = r2->previous;	
	};*/


	

	return (r_temp);
	
}
 
int CompareLinkedLists (car *LinkedList1, car * LinkedList2)
{

	if ((LinkedList1->acc == LinkedList2->acc) && (LinkedList1->decc == LinkedList2->decc) && (LinkedList1->Vd == LinkedList2->Vd) && (LinkedList1->lane == LinkedList2->lane)
		&& (LinkedList1->l == LinkedList2->l) && (LinkedList1->a_update == LinkedList2->a_update) && (LinkedList1->s == LinkedList2->s) && (LinkedList1->x == LinkedList2->x)
		&& (LinkedList1->v == LinkedList2->v) && (LinkedList1->headway == LinkedList2->headway) && (LinkedList1->DV == LinkedList2->DV) && (LinkedList1->Spacing == LinkedList2->Spacing)
		&& (LinkedList1->risk == LinkedList2->risk) && (LinkedList1->Rt == LinkedList2->Rt) && (LinkedList1->react == LinkedList2->react) && (LinkedList1->LCT == LinkedList2->LCT)
		&& (LinkedList1->LCcrash == LinkedList2->LCcrash) && (LinkedList1->departure == LinkedList2->departure) && (LinkedList1->carID == LinkedList2->carID) 
		&& (LinkedList1->crash == LinkedList2->crash))/* && (LinkedList1->crashtimecounter_ramp == LinkedList2->crashtimecounter_ramp)
		&& (LinkedList1->x_prev == LinkedList2->x_prev) && (LinkedList1->dt_prev == LinkedList2->dt_prev) && (LinkedList1->crash_prev == LinkedList2->crash_prev)
		&& (LinkedList1->lane_prev == LinkedList2->lane_prev) && (LinkedList1->v_prev == LinkedList2->v_prev) && (LinkedList1->risk_original == LinkedList2->risk_original)
		&& (LinkedList1->decc_original == LinkedList2->decc_original) && (LinkedList1->Vd_original == LinkedList2->Vd_original) && (LinkedList1->Yt == LinkedList2->Yt)
		&& (LinkedList1->Yt_previous == LinkedList2->Yt_previous) && (LinkedList1->Leader_Previous == LinkedList2->Leader_Previous) && (LinkedList1->Gamma == LinkedList2->Gamma)
		&& (LinkedList1->Wm == LinkedList2->Wm) && (LinkedList1->Wc == LinkedList2->Wc) && (LinkedList1->Tmax == LinkedList2->Tmax) && (LinkedList1->Alpha == LinkedList2->Alpha) 
		&& (LinkedList1->Beta == LinkedList2->Beta) && (LinkedList1->Tcorr == LinkedList2->Tcorr) && (LinkedList1->Amax == LinkedList2->Amax) && (LinkedList1->Vo == LinkedList2->Vo)
		&& (LinkedList1->So == LinkedList2->So) && (LinkedList1->Ao == LinkedList2->Ao) && (LinkedList1->Wp == LinkedList2->Wp) && (LinkedList1->DeltaW ==LinkedList2->DeltaW)
		&& (LinkedList1->Sigma == LinkedList2->Sigma) && (LinkedList1->Dummy_Shockwave == LinkedList2->Dummy_Shockwave) && (LinkedList1->LCL == LinkedList2->LCL)
		&& (LinkedList1->DXL1 == LinkedList2->DXL1) && (LinkedList1->DVL1 == LinkedList2->DVL1) && (LinkedList1->DXL2 == LinkedList2->DXL2) && (LinkedList1->DVL2 == LinkedList2->DVL2)
		&& (LinkedList1->DXF1 == LinkedList2->DXF1) && (LinkedList1->DVF1 == LinkedList2->DVF1) && (LinkedList1->DXL1R == LinkedList2->DXL1R) && (LinkedList1->DVL1R == LinkedList2->DVL1R)
		&& (LinkedList1->DXF1R == LinkedList2->DXF1R) && (LinkedList1->DVF1R == LinkedList2->DVF1R) && (LinkedList1->DXL1L == LinkedList2->DXL1L) && (LinkedList1->DVL1L == LinkedList2->DVL1L)
		&& (LinkedList1->DXF1L == LinkedList2->DXF1L) && (LinkedList1->DVF1L == LinkedList2->DVF1L) && (LinkedList1->KR == LinkedList2->KR) && (LinkedList1->KL == LinkedList2->KL)
		&& (LinkedList1->K == LinkedList2->K) && (LinkedList1->Duration == LinkedList2->Duration) && (LinkedList1->Mode == LinkedList2->Mode) 
		&& (LinkedList1->PreviousLeaderID == LinkedList2->PreviousLeaderID) && (LinkedList1->GammaHeterogeneity == LinkedList2->GammaHeterogeneity)
		&& (LinkedList1->RampDummy == LinkedList2->RampDummy) && (LinkedList1->LeftLaneDummy == LinkedList2->LeftLaneDummy) 
		&& (LinkedList1->RightLaneDummy == LinkedList2->RightLaneDummy) && (LinkedList1->DistanceToRamp == LinkedList2->DistanceToRamp)
 		&& (LinkedList1->Hazard == LinkedList2->Hazard) && (LinkedList1->DummyDXL1 == LinkedList2->DummyDXL1) && (LinkedList1->DummyDVL1 == LinkedList2->DummyDVL1) && (LinkedList1->DummyDXL1 == LinkedList2->DummyDXL1))*/

	{
		return (1);
	}
	else
	{
		return (-1);
	};
}
 

/*int PreviousLeaderIDgenerator(car *p1)
{
	if (p1->Mode != -1)
	{
					
		if (p1->next == NULL)
			p1->PreviousLeaderID = -1;
		else
			p1->PreviousLeaderID = p->next->carID;
	}				
	else
	{
		if (p->next == NULL)
		{
			if (p->PreviousLeaderID != -1)
			{
				p->LCL = p->LCL + 1;
				p->PreviousLeaderID = -1;
			}
		}
		else
		{
			if (p->PreviousLeaderID != p->next->carID)
			{
				p->LCL = p->LCL + 1;
				p->PreviousLeaderID = p->next->carID;
							
			}	
		}
	}
}*/

vector<int> wavelet_pushback_counter;

struct Wavelet_Information
{
	double Location;
	long Time;
	double Energy_value;
	int car_ID;
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

double Energy1,Energy2,Energy3,Energy4,Energy5;
int Evaluate;
int Car_Wavelet_History[10000];
//int Car_Wavelet_Segment_Counter[10000];
int Shockwave_counter [10000];

Wavelet_Information wavelet (vector<data> vehicle_info, long time_counter, int level)
{
	//double Energy [100000];
	//vector<double> Energy;
	double Energy;
	//Energy.clear();
	double Energy_temp = 0;
	double a_max = 64;//1;
	double T_wavelet = 0;
	long b_max, b_min;
	//Wavelet_Information *temp_info;
	Wavelet_Information temp_info;
	//temp_info = NULL;
	//temp_info = (Wavelet_Information*)malloc(sizeof(temp_info));

	b_max = vehicle_info.size() - 1;
	
	if (level == 1)
		b_min = b_max - 40;//100;
	if (level == 2)
		b_min = b_max - 30;//90;
	if (level == 3)
		b_min = b_max - 20;//80;
	if (level == 4)
		b_min = b_max - 10;//70;
	if (level == 5)
		b_min = b_max - 0;// - 60;

	if(b_min < 0)
		b_min = 0;

	for (long b = b_min; b < b_min + 1; b++)
	{
		//for (float a = 0.05; a < a_max; a+=0.05)
		for (float a = 1; a < a_max; a+=1)
		{
			for (long i = 0; i < vehicle_info.size(); i++)
			{				
				T_wavelet = T_wavelet + (vehicle_info[i].v) * (1 - pow(((vehicle_info[i].t - vehicle_info[b].t) /(a)),2)) * exp((-1) * pow((((vehicle_info[i].t - vehicle_info[b].t) / (2 * a))),2));
			}
			T_wavelet = (1 / sqrt(a)) * T_wavelet;
			Energy_temp = Energy_temp + (T_wavelet * T_wavelet);
		}
		Energy = ((1 / a_max) * Energy_temp);
	}

	Energy = Energy/(1000000000);
	//temp_info->Energy_value = Energy[b_min];
	//temp_info->Location = vehicle_info[b_min].x;
	//temp_info->Time = time_counter;
	temp_info.Energy_value = Energy;
	temp_info.Location = vehicle_info[b_min].x;
	temp_info.Time = time_counter;

	return (temp_info);
}

int Evaluation (double E1, double E2, double E3, double E4, double E5)
{
	int Evaluation_temp = 0;
	double First_Derivative = 0;
	double Second_Derivative = 0;

	First_Derivative  = (-E5 + 8 * E4 - 8 * E2 + E1)/(12 * 10);
	Second_Derivative = (-E5 + 16 * E4 - 30 * E3 + 16 * E2 - E1)/(12 * 10 * 10);

	if (Second_Derivative < 0)
		if ( sqrt(First_Derivative * First_Derivative) < 0.05)
			Evaluation_temp = 1;
	return(Evaluation_temp);
}

#endif
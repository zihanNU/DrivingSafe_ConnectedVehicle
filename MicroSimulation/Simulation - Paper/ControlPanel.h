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
#include "Detector.h"

using namespace std;



int TimetoClear = 50;

double DesiredSpeed = 26.7;
double InitialSpeed = 26.7;
float CV_Market_Penetration_Rate = 0;//0-99
int InitializationSeed = 9;//13;//34;//90;//34;
float Autonomous_Market_Penetration_Rate = 0;//0-99
double rand_seed_platoon = InitializationSeed; // Seed for the vehicle type
 
// It is changed in control panel again

/*
mile/hr		mps
15			6.7
35			15.6
45			20
60			26.67
65			25
*/
//double safeD = 70.0; //safety margin for car to enter simulation
//double safeLC1 = 60.0; //safety margin on either side for car to prevent 3.0
//double safeLC = 100.0; //lane change (if within safety margin, the lane change
							//crash percentage is used to decide lane change or not.

double safeD = 50.0; //safety margin for car to enter simulation
double safeLC1 = 10.0; //safety margin on either side for car to prevent 3.0
double safeLC = 10.0; //lane change (if within safety margin, the lane change
							//crash percentage is used to decide lane change or not.


float Sensor_Range = 43.5;

float TargetFlow = 1500;
float TargetDensity = 15;

int Compliance = 100.00;

float VSLStartLocation_Fix = 1000;
float VSLEndLocation_Fix = 10000;
float VSLStartLocation = 4000;
float VSLEndLocation = 6000;

float NonComplianceRate = 0.00;//0.10;
float OverspeedingRate = 0.00;//0.15;

int VSLActivation = 0;//if 1 then VSL active, 0 otherwise.
int RampMeteringActivation = 0;


float detectorlocation = 5950;
float detectorlength = 2;

float LaneDropLocation = 100000;

float DriversSpeedLimit(float SpeedLimit, int Compliancetype)
{
	float Z_Value;
	float OverSpeed;
	float Mean_Compliance;
	float Variance_Compliance;
	float Range_Compliance;

	if (Compliancetype<=4)
	{
		if (Compliancetype==1)
			Z_Value = 2.33;
		if (Compliancetype==2)
			Z_Value = 1.555;
		if (Compliancetype==3)
			Z_Value = 0.68;
		if (Compliancetype==4)
			Z_Value = 3.09;

		if (Compliancetype==1)
			OverSpeed = 1.19;
		if (Compliancetype==2)
			OverSpeed  = 1.19;
		if (Compliancetype==3)
			OverSpeed  = 1.08;
		if (Compliancetype==4)
			OverSpeed  = 0.95;


		Mean_Compliance = OverSpeed * SpeedLimit;
		Variance_Compliance = (Mean_Compliance - SpeedLimit) / (Z_Value);//2.33 is the normal value for the 99% level.
		Range_Compliance = 2 * (Mean_Compliance - SpeedLimit) + 5;
	
		return (float) ( normal(Mean_Compliance, Variance_Compliance, Range_Compliance,80,90));
	}
	else
		return (float) (SpeedLimit);
}

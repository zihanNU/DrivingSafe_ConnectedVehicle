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


int section_output_final (int i, vector<data2> AggDataHazardD_Li, vector<data3> AggDataHazardS_Li, vector<data3> AggDataHazardE_Li, vector<data4> AggDataHazardA_Li, vector <float> SelectedSpeedLimit_Li)
{
	string fileaddress;
	FILE *stHazardL0;
	char filename[9999];
	sprintf(filename, "..\\I-O Files_%d\\Simulated-Density-Hazard-L%d.txt",InitializationSeed, i);
	// cout << filename;
	stHazardL0 = fopen(filename, "wb");

	
	double AGGcounter = 0;
	int NonZeroCounterSection1 = 0;
	int NonZeroCounterSection2 = 0;
	int NonZeroCounterSection3 = 0;
	int NonZeroCounterSection4 = 0;
	int NonZeroCounterSection5 = 0;
	int NonZeroCounterSection6 = 0;
	int NonZeroCounterSection7 = 0;
	int NonZeroCounterSection8 = 0;
	int NonZeroCounterSection9 = 0;
	int NonZeroCounterSection10 = 0;

	if(stHazardL0 !=NULL)
	{
		for (int i=1; i<AggDataHazardD_Li.size(); i++)
		{
			if (i % 300 == 0)
			{
				 

				AGGcounter = 0;
				NonZeroCounterSection1 = 1;
				NonZeroCounterSection2 = 1;
				NonZeroCounterSection3 = 1;
				NonZeroCounterSection4 = 1;
				NonZeroCounterSection5 = 1;
				NonZeroCounterSection6 = 1;
				NonZeroCounterSection7 = 1;
				NonZeroCounterSection8 = 1;
				NonZeroCounterSection9 = 1;
				NonZeroCounterSection10 = 1;

				AGGcounter = i-299;
				while (AGGcounter<=i-1)
				{
					AGGcounter = AGGcounter + 1;	

					 
					if (AggDataHazardD_Li[AGGcounter].Section1 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section1=AggDataHazardS_Li[AGGcounter].Section1/AggDataHazardD_Li[AGGcounter].Section1;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section1 = 0;
					}
	
					if (AggDataHazardD_Li[AGGcounter].Section2 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section2=AggDataHazardS_Li[AGGcounter].Section2/AggDataHazardD_Li[AGGcounter].Section2;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section2 = 0;
					}
	
					if (AggDataHazardD_Li[AGGcounter].Section3 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section3=AggDataHazardS_Li[AGGcounter].Section3/AggDataHazardD_Li[AGGcounter].Section3;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section3 = 0;
					}
	
					if (AggDataHazardD_Li[AGGcounter].Section4 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section4=AggDataHazardS_Li[AGGcounter].Section4/AggDataHazardD_Li[AGGcounter].Section4;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section4 = 0;
					}

					if (AggDataHazardD_Li[AGGcounter].Section5 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section5=AggDataHazardS_Li[AGGcounter].Section5/AggDataHazardD_Li[AGGcounter].Section5;
					}
					else	
					{
						AggDataHazardS_Li[AGGcounter].Section5 = 0;
					}
	
					if (AggDataHazardD_Li[AGGcounter].Section6 != 0)
					{						
						AggDataHazardS_Li[AGGcounter].Section6=AggDataHazardS_Li[AGGcounter].Section6/AggDataHazardD_Li[AGGcounter].Section6;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section6 = 0;
					}
	
					if (AggDataHazardD_Li[AGGcounter].Section7 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section7=AggDataHazardS_Li[AGGcounter].Section7/AggDataHazardD_Li[AGGcounter].Section7;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section7 = 0;
					}
		
					if (AggDataHazardD_Li[AGGcounter].Section8 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section8=AggDataHazardS_Li[AGGcounter].Section8/AggDataHazardD_Li[AGGcounter].Section8;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section8 = 0;
					}
	
					if (AggDataHazardD_Li[AGGcounter].Section9 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section9=AggDataHazardS_Li[AGGcounter].Section9/AggDataHazardD_Li[AGGcounter].Section9;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section9 = 0;
					}
	
					if (AggDataHazardD_Li[AGGcounter].Section10 != 0)
					{
						AggDataHazardS_Li[AGGcounter].Section10=AggDataHazardS_Li[AGGcounter].Section10/AggDataHazardD_Li[AGGcounter].Section10;
					}
					else
					{
						AggDataHazardS_Li[AGGcounter].Section10 = 0;
					}
					 
				}

				
				//********************* Pre Assignment for counting the i-th one***********************
				if (AggDataHazardD_Li[i].Section1 == 0)
				{
					NonZeroCounterSection1 = 0;
					AggDataHazardS_Li[i].Section1 = 0;
					AggDataHazardD_Li[i].Hazard1 = 0;
					AggDataHazardD_Li[i].Hazardcounter1 = 0;
					AggDataHazardE_Li[i].Section1 = 0;
					AggDataHazardA_Li[i].Section1 = 0;
				}
				if (AggDataHazardD_Li[i].Section2 == 0)
				{
					NonZeroCounterSection2 = 0;
					AggDataHazardS_Li[i].Section2 = 0;
					AggDataHazardD_Li[i].Hazard2 = 0;
					AggDataHazardD_Li[i].Hazardcounter2 = 0;
					AggDataHazardE_Li[i].Section2 = 0;
					AggDataHazardA_Li[i].Section2 = 0;
				}
				if (AggDataHazardD_Li[i].Section3 == 0)
				{
					NonZeroCounterSection3 = 0;
					AggDataHazardS_Li[i].Section3 = 0;
					AggDataHazardD_Li[i].Hazard3 = 0;
					AggDataHazardD_Li[i].Hazardcounter3 = 0;
					AggDataHazardE_Li[i].Section3 = 0;
					AggDataHazardA_Li[i].Section3 = 0;
				}
				if (AggDataHazardD_Li[i].Section4 == 0)
				{
					NonZeroCounterSection4 = 0;
					AggDataHazardS_Li[i].Section4 = 0;
					AggDataHazardD_Li[i].Hazard4 = 0;
					AggDataHazardD_Li[i].Hazardcounter4 = 0;
					AggDataHazardE_Li[i].Section4 = 0;
					AggDataHazardA_Li[i].Section4 = 0;
				}
				if (AggDataHazardD_Li[i].Section5 == 0)
				{
					NonZeroCounterSection5 = 0;
					AggDataHazardS_Li[i].Section5 = 0;
					AggDataHazardD_Li[i].Hazard5 = 0;
					AggDataHazardD_Li[i].Hazardcounter5 = 0;
					AggDataHazardE_Li[i].Section5 = 0;
					AggDataHazardA_Li[i].Section5 = 0;
				}
				if (AggDataHazardD_Li[i].Section6 == 0)
				{
					NonZeroCounterSection6 = 0;
					AggDataHazardS_Li[i].Section6 = 0;
					AggDataHazardD_Li[i].Hazard6 = 0;
					AggDataHazardD_Li[i].Hazardcounter6 = 0;
					AggDataHazardE_Li[i].Section6 = 0;
					AggDataHazardA_Li[i].Section6 = 0;
				}
				if (AggDataHazardD_Li[i].Section7 == 0)
				{
					NonZeroCounterSection7 = 0;
					AggDataHazardS_Li[i].Section7 = 0;
					AggDataHazardD_Li[i].Hazard7 = 0;
					AggDataHazardD_Li[i].Hazardcounter7 = 0;
					AggDataHazardE_Li[i].Section7 = 0;
					AggDataHazardA_Li[i].Section7 = 0;
				}
				if (AggDataHazardD_Li[i].Section8 == 0)
				{
					NonZeroCounterSection8 = 0;
					AggDataHazardS_Li[i].Section8 = 0;
					AggDataHazardD_Li[i].Hazard8 = 0;
					AggDataHazardD_Li[i].Hazardcounter8 = 0;
					AggDataHazardE_Li[i].Section8 = 0;
					AggDataHazardA_Li[i].Section8 = 0;
				}
				if (AggDataHazardD_Li[i].Section9 == 0)
				{
					NonZeroCounterSection9 = 0;
					AggDataHazardS_Li[i].Section9 = 0;
					AggDataHazardD_Li[i].Hazard9 = 0;
					AggDataHazardD_Li[i].Hazardcounter9 = 0;
					AggDataHazardE_Li[i].Section9 = 0;
					AggDataHazardA_Li[i].Section9 = 0;
				}
				if (AggDataHazardD_Li[i].Section10 == 0)
				{
					NonZeroCounterSection10 = 0;
					AggDataHazardS_Li[i].Section10 = 0;
					AggDataHazardD_Li[i].Hazard10 = 0;
					AggDataHazardD_Li[i].Hazardcounter10 = 0;
					AggDataHazardE_Li[i].Section10 = 0;
					AggDataHazardA_Li[i].Section10 = 0;
				}
				//********************* End of Pre Assignment for counting the i-th one***********************

				
				AGGcounter = i-299;
				while (AGGcounter<i-1)
				{
					AGGcounter = AGGcounter + 1;

					if (AggDataHazardD_Li[AGGcounter].Section1 != 0)
					{
						AggDataHazardD_Li[i].Section1 = AggDataHazardD_Li[i].Section1 + AggDataHazardD_Li[AGGcounter].Section1;
						AggDataHazardD_Li[i].Hazard1 = AggDataHazardD_Li[i].Hazard1 + AggDataHazardD_Li[AGGcounter].Hazard1;
						AggDataHazardS_Li[i].Section1 = AggDataHazardS_Li[i].Section1 + AggDataHazardS_Li[AGGcounter].Section1;
						AggDataHazardD_Li[i].Hazardcounter1 = AggDataHazardD_Li[i].Hazardcounter1 + AggDataHazardD_Li[AGGcounter].Hazardcounter1;
						AggDataHazardE_Li[i].Section1 = AggDataHazardE_Li[i].Section1 + AggDataHazardE_Li[AGGcounter].Section1;
						AggDataHazardA_Li[i].Section1 = AggDataHazardA_Li[i].Section1 + AggDataHazardA_Li[AGGcounter].Section1;
						NonZeroCounterSection1 = NonZeroCounterSection1 +1;
					}
					
					if (AggDataHazardD_Li[AGGcounter].Section2 != 0)
					{
						AggDataHazardD_Li[i].Section2 = AggDataHazardD_Li[i].Section2 + AggDataHazardD_Li[AGGcounter].Section2;
						AggDataHazardD_Li[i].Hazard2 = AggDataHazardD_Li[i].Hazard2 + AggDataHazardD_Li[AGGcounter].Hazard2;
						AggDataHazardS_Li[i].Section2 = AggDataHazardS_Li[i].Section2 + AggDataHazardS_Li[AGGcounter].Section2;
						AggDataHazardD_Li[i].Hazardcounter2 = AggDataHazardD_Li[i].Hazardcounter2 + AggDataHazardD_Li[AGGcounter].Hazardcounter2;
						AggDataHazardE_Li[i].Section2 = AggDataHazardE_Li[i].Section2 + AggDataHazardE_Li[AGGcounter].Section2;
						AggDataHazardA_Li[i].Section2 = AggDataHazardA_Li[i].Section2 + AggDataHazardA_Li[AGGcounter].Section2;
						NonZeroCounterSection2 = NonZeroCounterSection2 +1;
					}

					if (AggDataHazardD_Li[AGGcounter].Section3 != 0)
					{
						AggDataHazardD_Li[i].Section3 = AggDataHazardD_Li[i].Section3 + AggDataHazardD_Li[AGGcounter].Section3;
						AggDataHazardD_Li[i].Hazard3 = AggDataHazardD_Li[i].Hazard3 + AggDataHazardD_Li[AGGcounter].Hazard3;
						AggDataHazardS_Li[i].Section3 = AggDataHazardS_Li[i].Section3 + AggDataHazardS_Li[AGGcounter].Section3;
						AggDataHazardD_Li[i].Hazardcounter3 = AggDataHazardD_Li[i].Hazardcounter3 + AggDataHazardD_Li[AGGcounter].Hazardcounter3;
						AggDataHazardE_Li[i].Section3 = AggDataHazardE_Li[i].Section3 + AggDataHazardE_Li[AGGcounter].Section3;
						AggDataHazardA_Li[i].Section3 = AggDataHazardA_Li[i].Section3 + AggDataHazardA_Li[AGGcounter].Section3;
						NonZeroCounterSection3 = NonZeroCounterSection3 +1;
					}

					if (AggDataHazardD_Li[AGGcounter].Section4 != 0)
					{
						AggDataHazardD_Li[i].Section4 = AggDataHazardD_Li[i].Section4 + AggDataHazardD_Li[AGGcounter].Section4;
						AggDataHazardD_Li[i].Hazard4 = AggDataHazardD_Li[i].Hazard4 + AggDataHazardD_Li[AGGcounter].Hazard4;
						AggDataHazardS_Li[i].Section4 = AggDataHazardS_Li[i].Section4 + AggDataHazardS_Li[AGGcounter].Section4;
						AggDataHazardD_Li[i].Hazardcounter4 = AggDataHazardD_Li[i].Hazardcounter4 + AggDataHazardD_Li[AGGcounter].Hazardcounter4;
						AggDataHazardE_Li[i].Section4 = AggDataHazardE_Li[i].Section4 + AggDataHazardE_Li[AGGcounter].Section4;
						AggDataHazardA_Li[i].Section4 = AggDataHazardA_Li[i].Section4 + AggDataHazardA_Li[AGGcounter].Section4;
						NonZeroCounterSection4 = NonZeroCounterSection4 +1;
					}

					if (AggDataHazardD_Li[AGGcounter].Section5 != 0)
					{
						AggDataHazardD_Li[i].Section5 = AggDataHazardD_Li[i].Section5 + AggDataHazardD_Li[AGGcounter].Section5;
						AggDataHazardD_Li[i].Hazard5 = AggDataHazardD_Li[i].Hazard5 + AggDataHazardD_Li[AGGcounter].Hazard5;
						AggDataHazardS_Li[i].Section5 = AggDataHazardS_Li[i].Section5 + AggDataHazardS_Li[AGGcounter].Section5;
						AggDataHazardD_Li[i].Hazardcounter5 = AggDataHazardD_Li[i].Hazardcounter5 + AggDataHazardD_Li[AGGcounter].Hazardcounter5;
						AggDataHazardE_Li[i].Section5 = AggDataHazardE_Li[i].Section5 + AggDataHazardE_Li[AGGcounter].Section5;
						AggDataHazardA_Li[i].Section5 = AggDataHazardA_Li[i].Section5 + AggDataHazardA_Li[AGGcounter].Section5;
						NonZeroCounterSection5 = NonZeroCounterSection5 +1;
					}

					if (AggDataHazardD_Li[AGGcounter].Section6 != 0)
					{
						AggDataHazardD_Li[i].Section6 = AggDataHazardD_Li[i].Section6 + AggDataHazardD_Li[AGGcounter].Section6;
						AggDataHazardD_Li[i].Hazard6 = AggDataHazardD_Li[i].Hazard6 + AggDataHazardD_Li[AGGcounter].Hazard6;
						AggDataHazardS_Li[i].Section6 = AggDataHazardS_Li[i].Section6 + AggDataHazardS_Li[AGGcounter].Section6;
						AggDataHazardD_Li[i].Hazardcounter6 = AggDataHazardD_Li[i].Hazardcounter6 + AggDataHazardD_Li[AGGcounter].Hazardcounter6;
						AggDataHazardE_Li[i].Section6 = AggDataHazardE_Li[i].Section6 + AggDataHazardE_Li[AGGcounter].Section6;
						AggDataHazardA_Li[i].Section6 = AggDataHazardA_Li[i].Section6 + AggDataHazardA_Li[AGGcounter].Section6;
						NonZeroCounterSection6 = NonZeroCounterSection6 +1;
					}

					if (AggDataHazardD_Li[AGGcounter].Section7 != 0)
					{
						AggDataHazardD_Li[i].Section7 = AggDataHazardD_Li[i].Section7 + AggDataHazardD_Li[AGGcounter].Section7;
						AggDataHazardD_Li[i].Hazard7 = AggDataHazardD_Li[i].Hazard7 + AggDataHazardD_Li[AGGcounter].Hazard7;
						AggDataHazardS_Li[i].Section7 = AggDataHazardS_Li[i].Section7 + AggDataHazardS_Li[AGGcounter].Section7;
						AggDataHazardD_Li[i].Hazardcounter7 = AggDataHazardD_Li[i].Hazardcounter7 + AggDataHazardD_Li[AGGcounter].Hazardcounter7;
						AggDataHazardE_Li[i].Section7 = AggDataHazardE_Li[i].Section7 + AggDataHazardE_Li[AGGcounter].Section7;
						AggDataHazardA_Li[i].Section7 = AggDataHazardA_Li[i].Section7 + AggDataHazardA_Li[AGGcounter].Section7;
						NonZeroCounterSection7 = NonZeroCounterSection7 +1;
					}

					if (AggDataHazardD_Li[AGGcounter].Section8 != 0)
					{
						AggDataHazardD_Li[i].Section8 = AggDataHazardD_Li[i].Section8 + AggDataHazardD_Li[AGGcounter].Section8;
						AggDataHazardD_Li[i].Hazard8 = AggDataHazardD_Li[i].Hazard8 + AggDataHazardD_Li[AGGcounter].Hazard8;
						AggDataHazardS_Li[i].Section8 = AggDataHazardS_Li[i].Section8 + AggDataHazardS_Li[AGGcounter].Section8;
						AggDataHazardD_Li[i].Hazardcounter8 = AggDataHazardD_Li[i].Hazardcounter8 + AggDataHazardD_Li[AGGcounter].Hazardcounter8;
						AggDataHazardE_Li[i].Section8 = AggDataHazardE_Li[i].Section8 + AggDataHazardE_Li[AGGcounter].Section8;
						AggDataHazardA_Li[i].Section8 = AggDataHazardA_Li[i].Section8 + AggDataHazardA_Li[AGGcounter].Section8;
						NonZeroCounterSection8 = NonZeroCounterSection8 +1;
					}

					if (AggDataHazardD_Li[AGGcounter].Section9 != 0)
					{
						AggDataHazardD_Li[i].Section9 = AggDataHazardD_Li[i].Section9 + AggDataHazardD_Li[AGGcounter].Section9;
						AggDataHazardD_Li[i].Hazard9 = AggDataHazardD_Li[i].Hazard9 + AggDataHazardD_Li[AGGcounter].Hazard9;
						AggDataHazardS_Li[i].Section9 = AggDataHazardS_Li[i].Section9 + AggDataHazardS_Li[AGGcounter].Section9;
						AggDataHazardD_Li[i].Hazardcounter9 = AggDataHazardD_Li[i].Hazardcounter9 + AggDataHazardD_Li[AGGcounter].Hazardcounter9;
						AggDataHazardE_Li[i].Section9 = AggDataHazardE_Li[i].Section9 + AggDataHazardE_Li[AGGcounter].Section9;
						AggDataHazardA_Li[i].Section9 = AggDataHazardA_Li[i].Section9 + AggDataHazardA_Li[AGGcounter].Section9;
						NonZeroCounterSection9 = NonZeroCounterSection9 +1;
					}

					if (AggDataHazardD_Li[AGGcounter].Section10 != 0)
					{
						AggDataHazardD_Li[i].Section10 = AggDataHazardD_Li[i].Section10 + AggDataHazardD_Li[AGGcounter].Section10;
						AggDataHazardD_Li[i].Hazard10 = AggDataHazardD_Li[i].Hazard10 + AggDataHazardD_Li[AGGcounter].Hazard10;
						AggDataHazardS_Li[i].Section10 = AggDataHazardS_Li[i].Section10 + AggDataHazardS_Li[AGGcounter].Section10;
						AggDataHazardD_Li[i].Hazardcounter10 = AggDataHazardD_Li[i].Hazardcounter10 + AggDataHazardD_Li[AGGcounter].Hazardcounter10;
						AggDataHazardE_Li[i].Section10 = AggDataHazardE_Li[i].Section10 + AggDataHazardE_Li[AGGcounter].Section10;
						AggDataHazardA_Li[i].Section10 = AggDataHazardA_Li[i].Section10 + AggDataHazardA_Li[AGGcounter].Section10;
						NonZeroCounterSection10 = NonZeroCounterSection10 +1;
					}
					
				}
				
				if (NonZeroCounterSection1 != 0)
				{
					AggDataHazardD_Li[i].Section1 = AggDataHazardD_Li[i].Section1 / NonZeroCounterSection1;
					AggDataHazardD_Li[i].Hazard1 = AggDataHazardD_Li[i].Hazard1 / NonZeroCounterSection1;					
					AggDataHazardS_Li[i].Section1 = AggDataHazardS_Li[i].Section1 / NonZeroCounterSection1;					
					AggDataHazardD_Li[i].Hazardcounter1 = AggDataHazardD_Li[i].Hazardcounter1 / NonZeroCounterSection1;
					AggDataHazardE_Li[i].Section1 = AggDataHazardE_Li[i].Section1 / NonZeroCounterSection1;
					AggDataHazardA_Li[i].Section1 = AggDataHazardA_Li[i].Section1 / NonZeroCounterSection1;
				}
				else
				{
					AggDataHazardD_Li[i].Section1 = 0;
					AggDataHazardD_Li[i].Hazard1 = 0;
					AggDataHazardS_Li[i].Section1 = 0;
					AggDataHazardD_Li[i].Hazardcounter1 = 0;
					AggDataHazardE_Li[i].Section1 = 0;
					AggDataHazardA_Li[i].Section1 = 0;
				}

				if (NonZeroCounterSection2 != 0)
				{
					AggDataHazardD_Li[i].Section2 = AggDataHazardD_Li[i].Section2 / NonZeroCounterSection2;
					AggDataHazardD_Li[i].Hazard2 = AggDataHazardD_Li[i].Hazard2 / NonZeroCounterSection2;
					AggDataHazardS_Li[i].Section2 = AggDataHazardS_Li[i].Section2 / NonZeroCounterSection2;
					AggDataHazardD_Li[i].Hazardcounter2 = AggDataHazardD_Li[i].Hazardcounter2 / NonZeroCounterSection2;
					AggDataHazardE_Li[i].Section2 = AggDataHazardE_Li[i].Section2 / NonZeroCounterSection2;
					AggDataHazardA_Li[i].Section2 = AggDataHazardA_Li[i].Section2 / NonZeroCounterSection2;
				}
				else
				{
					AggDataHazardD_Li[i].Section2 = 0;
					AggDataHazardD_Li[i].Hazard2 = 0;
					AggDataHazardS_Li[i].Section2 = 0;
					AggDataHazardD_Li[i].Hazardcounter2 = 0;
					AggDataHazardE_Li[i].Section2 = 0;
					AggDataHazardA_Li[i].Section2 = 0;
				}

				if (NonZeroCounterSection3 != 0)
				{
					AggDataHazardD_Li[i].Section3 = AggDataHazardD_Li[i].Section3 / NonZeroCounterSection3;
					AggDataHazardD_Li[i].Hazard3 = AggDataHazardD_Li[i].Hazard3 / NonZeroCounterSection3;
					AggDataHazardS_Li[i].Section3 = AggDataHazardS_Li[i].Section3 / NonZeroCounterSection3;
					AggDataHazardD_Li[i].Hazardcounter3 = AggDataHazardD_Li[i].Hazardcounter3 / NonZeroCounterSection3;
					AggDataHazardE_Li[i].Section3 = AggDataHazardE_Li[i].Section3 / NonZeroCounterSection3;
					AggDataHazardA_Li[i].Section3 = AggDataHazardA_Li[i].Section3 / NonZeroCounterSection3;
				}
				else
				{
					AggDataHazardD_Li[i].Section3 = 0;
					AggDataHazardD_Li[i].Hazard3 = 0;
					AggDataHazardS_Li[i].Section3 = 0;
					AggDataHazardD_Li[i].Hazardcounter3 = 0;
					AggDataHazardE_Li[i].Section3 = 0;
					AggDataHazardA_Li[i].Section3 = 0;
				}

				if (NonZeroCounterSection4 != 0)
				{
					AggDataHazardD_Li[i].Section4 = AggDataHazardD_Li[i].Section4 / NonZeroCounterSection4;
					AggDataHazardD_Li[i].Hazard4 = AggDataHazardD_Li[i].Hazard4 / NonZeroCounterSection4;
					AggDataHazardS_Li[i].Section4 = AggDataHazardS_Li[i].Section4 / NonZeroCounterSection4;
					AggDataHazardD_Li[i].Hazardcounter4 = AggDataHazardD_Li[i].Hazardcounter4 / NonZeroCounterSection4;
					AggDataHazardE_Li[i].Section4 = AggDataHazardE_Li[i].Section4 / NonZeroCounterSection4;
					AggDataHazardA_Li[i].Section4 = AggDataHazardA_Li[i].Section4 / NonZeroCounterSection4;
				}
				else
				{
					AggDataHazardD_Li[i].Section4 = 0;
					AggDataHazardD_Li[i].Hazard4 = 0;
					AggDataHazardS_Li[i].Section4 = 0;
					AggDataHazardD_Li[i].Hazardcounter4 = 0;
					AggDataHazardE_Li[i].Section4 = 0;
					AggDataHazardA_Li[i].Section4 = 0;
				}

				if (NonZeroCounterSection5 != 0)
				{
					AggDataHazardD_Li[i].Section5 = AggDataHazardD_Li[i].Section5 / NonZeroCounterSection5;
					AggDataHazardD_Li[i].Hazard5 = AggDataHazardD_Li[i].Hazard5 / NonZeroCounterSection5;
					AggDataHazardS_Li[i].Section5 = AggDataHazardS_Li[i].Section5 / NonZeroCounterSection5;
					AggDataHazardD_Li[i].Hazardcounter5 = AggDataHazardD_Li[i].Hazardcounter5 / NonZeroCounterSection5;
					AggDataHazardE_Li[i].Section5 = AggDataHazardE_Li[i].Section5 / NonZeroCounterSection5;
					AggDataHazardA_Li[i].Section5 = AggDataHazardA_Li[i].Section5 / NonZeroCounterSection5;
				}
				else
				{
					AggDataHazardD_Li[i].Section5 = 0;
					AggDataHazardD_Li[i].Hazard5 = 0;
					AggDataHazardS_Li[i].Section5 = 0;
					AggDataHazardD_Li[i].Hazardcounter5 = 0;
					AggDataHazardE_Li[i].Section5 = 0;
					AggDataHazardA_Li[i].Section5 = 0;
				}

				if (NonZeroCounterSection6 != 0)
				{
					AggDataHazardD_Li[i].Section6 = AggDataHazardD_Li[i].Section6 / NonZeroCounterSection6;
					AggDataHazardD_Li[i].Hazard6 = AggDataHazardD_Li[i].Hazard6 / NonZeroCounterSection6;
					AggDataHazardS_Li[i].Section6 = AggDataHazardS_Li[i].Section6 / NonZeroCounterSection6;
					AggDataHazardD_Li[i].Hazardcounter6 = AggDataHazardD_Li[i].Hazardcounter6 / NonZeroCounterSection6;
					AggDataHazardE_Li[i].Section6 = AggDataHazardE_Li[i].Section6 / NonZeroCounterSection6;
					AggDataHazardA_Li[i].Section6 = AggDataHazardA_Li[i].Section6 / NonZeroCounterSection6;
				}
				else
				{
					AggDataHazardD_Li[i].Section6 = 0;
					AggDataHazardD_Li[i].Hazard6 = 0;
					AggDataHazardS_Li[i].Section6 = 0;
					AggDataHazardD_Li[i].Hazardcounter6 = 0;
					AggDataHazardE_Li[i].Section6 = 0;
					AggDataHazardA_Li[i].Section6 = 0;
				}

				if (NonZeroCounterSection7 != 0)
				{
					AggDataHazardD_Li[i].Section7 = AggDataHazardD_Li[i].Section7 / NonZeroCounterSection7;
					AggDataHazardD_Li[i].Hazard7 = AggDataHazardD_Li[i].Hazard7 / NonZeroCounterSection7;
					AggDataHazardS_Li[i].Section7 = AggDataHazardS_Li[i].Section7 / NonZeroCounterSection7;
					AggDataHazardD_Li[i].Hazardcounter7 = AggDataHazardD_Li[i].Hazardcounter7 / NonZeroCounterSection7;
					AggDataHazardE_Li[i].Section7 = AggDataHazardE_Li[i].Section7 / NonZeroCounterSection7;
					AggDataHazardA_Li[i].Section7 = AggDataHazardA_Li[i].Section7 / NonZeroCounterSection7;
				}
				else
				{
					AggDataHazardD_Li[i].Section7 = 0;
					AggDataHazardD_Li[i].Hazard7 = 0;
					AggDataHazardS_Li[i].Section7 = 0;
					AggDataHazardD_Li[i].Hazardcounter7 = 0;
					AggDataHazardE_Li[i].Section7 = 0;
					AggDataHazardA_Li[i].Section7 = 0;
				}

				if (NonZeroCounterSection8 != 0)
				{
					AggDataHazardD_Li[i].Section8 = AggDataHazardD_Li[i].Section8 / NonZeroCounterSection8;
					AggDataHazardD_Li[i].Hazard8 = AggDataHazardD_Li[i].Hazard8 / NonZeroCounterSection8;
					AggDataHazardS_Li[i].Section8 = AggDataHazardS_Li[i].Section8 / NonZeroCounterSection8;
					AggDataHazardD_Li[i].Hazardcounter8 = AggDataHazardD_Li[i].Hazardcounter8 / NonZeroCounterSection8;
					AggDataHazardE_Li[i].Section8 = AggDataHazardE_Li[i].Section8 / NonZeroCounterSection8;
					AggDataHazardA_Li[i].Section8 = AggDataHazardA_Li[i].Section8 / NonZeroCounterSection8;
				}
				else
				{
					AggDataHazardD_Li[i].Section8 = 0;
					AggDataHazardD_Li[i].Hazard8 = 0;
					AggDataHazardS_Li[i].Section8 = 0;
					AggDataHazardD_Li[i].Hazardcounter8 = 0;
					AggDataHazardE_Li[i].Section8 = 0;
				}

				if (NonZeroCounterSection9 != 0)
				{
					AggDataHazardD_Li[i].Section9 = AggDataHazardD_Li[i].Section9 / NonZeroCounterSection9;
					AggDataHazardD_Li[i].Hazard9 = AggDataHazardD_Li[i].Hazard9 / NonZeroCounterSection9;
					AggDataHazardS_Li[i].Section9 = AggDataHazardS_Li[i].Section9 / NonZeroCounterSection9;
					AggDataHazardD_Li[i].Hazardcounter9 = AggDataHazardD_Li[i].Hazardcounter9 / NonZeroCounterSection9;
					AggDataHazardE_Li[i].Section9 = AggDataHazardE_Li[i].Section9 / NonZeroCounterSection9;
					AggDataHazardA_Li[i].Section9 = AggDataHazardA_Li[i].Section9 / NonZeroCounterSection9;
				}
				else
				{
					AggDataHazardD_Li[i].Section9 = 0;
					AggDataHazardD_Li[i].Hazard9 = 0;
					AggDataHazardS_Li[i].Section9 = 0;
					AggDataHazardD_Li[i].Hazardcounter9 = 0;
					AggDataHazardE_Li[i].Section9 = 0;
					AggDataHazardA_Li[i].Section9 = 0;
				}

				if (NonZeroCounterSection10 != 0)
				{
					AggDataHazardD_Li[i].Section10 = AggDataHazardD_Li[i].Section10 / NonZeroCounterSection10;
					AggDataHazardD_Li[i].Hazard10 = AggDataHazardD_Li[i].Hazard10 / NonZeroCounterSection10;
					AggDataHazardS_Li[i].Section10 = AggDataHazardS_Li[i].Section10 / NonZeroCounterSection10;
					AggDataHazardD_Li[i].Hazardcounter10 = AggDataHazardD_Li[i].Hazardcounter10 / NonZeroCounterSection10;
					AggDataHazardE_Li[i].Section10 = AggDataHazardE_Li[i].Section10 / NonZeroCounterSection10;
					AggDataHazardA_Li[i].Section10 = AggDataHazardA_Li[i].Section10 / NonZeroCounterSection10;
				}
				else
				{
					AggDataHazardD_Li[i].Section10 = 0;
					AggDataHazardD_Li[i].Hazard10 = 0;
					AggDataHazardS_Li[i].Section10 = 0;
					AggDataHazardD_Li[i].Hazardcounter10 = 0;
					AggDataHazardE_Li[i].Section10 = 0;
					AggDataHazardA_Li[i].Section10 = 0;
				}


			 
				
				fprintf(stHazardL0, "%d	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f\n",
						i,
						AggDataHazardD_Li[i].Section1,
						AggDataHazardD_Li[i].Section2,
						AggDataHazardD_Li[i].Section3,
						AggDataHazardD_Li[i].Section4,
						AggDataHazardD_Li[i].Section5,
						AggDataHazardD_Li[i].Section6,
						AggDataHazardD_Li[i].Section7,
						AggDataHazardD_Li[i].Section8,
						AggDataHazardD_Li[i].Section9,
						AggDataHazardD_Li[i].Section10,
						AggDataHazardD_Li[i].Hazard1,//AggDataHazardD_Li[i].Hazardcounter1),
						AggDataHazardD_Li[i].Hazard2,//AggDataHazardD_Li[i].Hazardcounter2),
						AggDataHazardD_Li[i].Hazard3,//AggDataHazardD_Li[i].Hazardcounter3),
						AggDataHazardD_Li[i].Hazard4,//AggDataHazardD_Li[i].Hazardcounter4),
						AggDataHazardD_Li[i].Hazard5,//AggDataHazardD_Li[i].Hazardcounter5),
						AggDataHazardD_Li[i].Hazard6,//AggDataHazardD_Li[i].Hazardcounter6),
						AggDataHazardD_Li[i].Hazard7,//AggDataHazardD_Li[i].Hazardcounter7),
						AggDataHazardD_Li[i].Hazard8,//AggDataHazardD_Li[i].Hazardcounter8),
						AggDataHazardD_Li[i].Hazard9,//AggDataHazardD_Li[i].Hazardcounter9),
						AggDataHazardD_Li[i].Hazard10,//AggDataHazardD_Li[i].Hazardcounter10),
						AggDataHazardS_Li[i].Section1,//AggDataHazardD_Li[i].Hazardcounter1,
						AggDataHazardS_Li[i].Section2,//AggDataHazardD_Li[i].Hazardcounter2,
						AggDataHazardS_Li[i].Section3,//AggDataHazardD_Li[i].Hazardcounter3,
						AggDataHazardS_Li[i].Section4,//AggDataHazardD_Li[i].Hazardcounter4,
						AggDataHazardS_Li[i].Section5,//AggDataHazardD_Li[i].Hazardcounter5,
						AggDataHazardS_Li[i].Section6,//AggDataHazardD_Li[i].Hazardcounter6,
						AggDataHazardS_Li[i].Section7,//AggDataHazardD_Li[i].Hazardcounter7,
						AggDataHazardS_Li[i].Section8,//AggDataHazardD_Li[i].Hazardcounter8,
						AggDataHazardS_Li[i].Section9,//AggDataHazardD_Li[i].Hazardcounter9,
						AggDataHazardS_Li[i].Section10,//AggDataHazardD_Li[i].Hazardcounter10,
						AggDataHazardD_Li[i].Hazardcounter1,
						AggDataHazardD_Li[i].Hazardcounter2,
						AggDataHazardD_Li[i].Hazardcounter3,
						AggDataHazardD_Li[i].Hazardcounter4,
						AggDataHazardD_Li[i].Hazardcounter5,
						AggDataHazardD_Li[i].Hazardcounter6,
						AggDataHazardD_Li[i].Hazardcounter7,
						AggDataHazardD_Li[i].Hazardcounter8,
						AggDataHazardD_Li[i].Hazardcounter9,
						AggDataHazardD_Li[i].Hazardcounter10,
						SelectedSpeedLimit_Li[i],
						AggDataHazardE_Li[i].Section1,
						AggDataHazardE_Li[i].Section2,
						AggDataHazardE_Li[i].Section3,
						AggDataHazardE_Li[i].Section4,
						AggDataHazardE_Li[i].Section5,
						AggDataHazardE_Li[i].Section6,
						AggDataHazardE_Li[i].Section7,
						AggDataHazardE_Li[i].Section8,
						AggDataHazardE_Li[i].Section9,
						AggDataHazardE_Li[i].Section10,
						AggDataHazardA_Li[i].Section1,
						AggDataHazardA_Li[i].Section2,
						AggDataHazardA_Li[i].Section3,
						AggDataHazardA_Li[i].Section4,
						AggDataHazardA_Li[i].Section5,
						AggDataHazardA_Li[i].Section6,
						AggDataHazardA_Li[i].Section7,
						AggDataHazardA_Li[i].Section8,
						AggDataHazardA_Li[i].Section9,
						AggDataHazardA_Li[i].Section10);
			}			
		}
	}
	fclose (stHazardL0);
	return 0;
}

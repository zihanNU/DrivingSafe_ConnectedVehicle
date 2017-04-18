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
#include "VehiclePath.h"
//#include "mrand.h"

using namespace std;
//char workingfolder[_MAX_PATH];

double AGGcount = 0;
int NZCounterSection1 = 0;
int NZCounterSection2 = 0;
int NZCounterSection3 = 0;
int NZCounterSection4 = 0;
int NZCounterSection5 = 0;
int NZCounterSection6 = 0;
int NZCounterSection7 = 0;
int NZCounterSection8 = 0;
int NZCounterSection9 = 0;
int NZCounterSection10 = 0;


struct SFDValues
{
	double SDFSpeed;
	double SDFFlow;
	double SDFDensity;
};

struct SFDValues *SFDL0 (vector <data2> ADHDL0, vector <data3> ADHSL0, int Timer)
{
			 

	SFDValues *SFDResutls;
	SFDResutls = NULL;
	SFDResutls = (SFDValues*)malloc(sizeof(SFDValues));

				AGGcount = 0;
				NZCounterSection1 = 1;
				NZCounterSection2 = 1;
				NZCounterSection3 = 1;
				NZCounterSection4 = 1;
				NZCounterSection5 = 1;
				NZCounterSection6 = 1;
				NZCounterSection7 = 1;
				NZCounterSection8 = 1;
				NZCounterSection9 = 1;
				NZCounterSection10 = 1;

				AGGcount = Timer-299;
				if (AGGcount < 0)
					AGGcount  = 0;
				while (AGGcount<=Timer-1)
				{
					AGGcount = AGGcount + 1;	

					 
					if (ADHDL0[AGGcount].Section1 != 0)
					{
						ADHSL0[AGGcount].Section1=ADHSL0[AGGcount].Section1/ADHDL0[AGGcount].Section1;
					}
					else
					{
						ADHSL0[AGGcount].Section1 = 0;
					}
	
					if (ADHDL0[AGGcount].Section2 != 0)
					{
						ADHSL0[AGGcount].Section2=ADHSL0[AGGcount].Section2/ADHDL0[AGGcount].Section2;
					}
					else
					{
						ADHSL0[AGGcount].Section2 = 0;
					}
	
					if (ADHDL0[AGGcount].Section3 != 0)
					{
						ADHSL0[AGGcount].Section3=ADHSL0[AGGcount].Section3/ADHDL0[AGGcount].Section3;
					}
					else
					{
						ADHSL0[AGGcount].Section3 = 0;
					}
	
					if (ADHDL0[AGGcount].Section4 != 0)
					{
						ADHSL0[AGGcount].Section4=ADHSL0[AGGcount].Section4/ADHDL0[AGGcount].Section4;
					}
					else
					{
						ADHSL0[AGGcount].Section4 = 0;
					}

					if (ADHDL0[AGGcount].Section5 != 0)
					{
						ADHSL0[AGGcount].Section5=ADHSL0[AGGcount].Section5/ADHDL0[AGGcount].Section5;
					}
					else	
					{
						ADHSL0[AGGcount].Section5 = 0;
					}
	
					if (ADHDL0[AGGcount].Section6 != 0)
					{						
						ADHSL0[AGGcount].Section6=ADHSL0[AGGcount].Section6/ADHDL0[AGGcount].Section6;
					}
					else
					{
						ADHSL0[AGGcount].Section6 = 0;
					}
	
					if (ADHDL0[AGGcount].Section7 != 0)
					{
						ADHSL0[AGGcount].Section7=ADHSL0[AGGcount].Section7/ADHDL0[AGGcount].Section7;
					}
					else
					{
						ADHSL0[AGGcount].Section7 = 0;
					}
		
					if (ADHDL0[AGGcount].Section8 != 0)
					{
						ADHSL0[AGGcount].Section8=ADHSL0[AGGcount].Section8/ADHDL0[AGGcount].Section8;
					}
					else
					{
						ADHSL0[AGGcount].Section8 = 0;
					}
	
					if (ADHDL0[AGGcount].Section9 != 0)
					{
						ADHSL0[AGGcount].Section9=ADHSL0[AGGcount].Section9/ADHDL0[AGGcount].Section9;
					}
					else
					{
						ADHSL0[AGGcount].Section9 = 0;
					}
	
					if (ADHDL0[AGGcount].Section10 != 0)
					{
						ADHSL0[AGGcount].Section10=ADHSL0[AGGcount].Section10/ADHDL0[AGGcount].Section10;
					}
					else
					{
						ADHSL0[AGGcount].Section10 = 0;
					}
					 
				}

				
				//********************* Pre Assignment for counting the i-th one***********************
				if (ADHDL0[Timer].Section1 == 0)
				{
					NZCounterSection1 = 0;
					ADHSL0[Timer].Section1 = 0;
					ADHDL0[Timer].Hazard1 = 0;
					ADHDL0[Timer].Hazardcounter1 = 0;
				}
				if (ADHDL0[Timer].Section2 == 0)
				{
					NZCounterSection2 = 0;
					ADHSL0[Timer].Section2 = 0;
					ADHDL0[Timer].Hazard2 = 0;
					ADHDL0[Timer].Hazardcounter2 = 0;
				}
				if (ADHDL0[Timer].Section3 == 0)
				{
					NZCounterSection3 = 0;
					ADHSL0[Timer].Section3 = 0;
					ADHDL0[Timer].Hazard3 = 0;
					ADHDL0[Timer].Hazardcounter3 = 0;
				}
				if (ADHDL0[Timer].Section4 == 0)
				{
					NZCounterSection4 = 0;
					ADHSL0[Timer].Section4 = 0;
					ADHDL0[Timer].Hazard4 = 0;
					ADHDL0[Timer].Hazardcounter4 = 0;
				}
				if (ADHDL0[Timer].Section5 == 0)
				{
					NZCounterSection5 = 0;
					ADHSL0[Timer].Section5 = 0;
					ADHDL0[Timer].Hazard5 = 0;
					ADHDL0[Timer].Hazardcounter5 = 0;
				}
				if (ADHDL0[Timer].Section6 == 0)
				{
					NZCounterSection6 = 0;
					ADHSL0[Timer].Section6 = 0;
					ADHDL0[Timer].Hazard6 = 0;
					ADHDL0[Timer].Hazardcounter6 = 0;
				}
				if (ADHDL0[Timer].Section7 == 0)
				{
					NZCounterSection7 = 0;
					ADHSL0[Timer].Section7 = 0;
					ADHDL0[Timer].Hazard7 = 0;
					ADHDL0[Timer].Hazardcounter7 = 0;
				}
				if (ADHDL0[Timer].Section8 == 0)
				{
					NZCounterSection8 = 0;
					ADHSL0[Timer].Section8 = 0;
					ADHDL0[Timer].Hazard8 = 0;
					ADHDL0[Timer].Hazardcounter8 = 0;
				}
				if (ADHDL0[Timer].Section9 == 0)
				{
					NZCounterSection9 = 0;
					ADHSL0[Timer].Section9 = 0;
					ADHDL0[Timer].Hazard9 = 0;
					ADHDL0[Timer].Hazardcounter9 = 0;
				}
				if (ADHDL0[Timer].Section10 == 0)
				{
					NZCounterSection10 = 0;
					ADHSL0[Timer].Section10 = 0;
					ADHDL0[Timer].Hazard10 = 0;
					ADHDL0[Timer].Hazardcounter10 = 0;
				}
				//********************* End of Pre Assignment for counting the i-th one***********************

				
				AGGcount = Timer-299;
				if (AGGcount < 0)
					AGGcount  = 0;
				while (AGGcount<Timer-1)
				{
					AGGcount = AGGcount + 1;

					if (ADHDL0[AGGcount].Section1 != 0)
					{
						ADHDL0[Timer].Section1 = ADHDL0[Timer].Section1 + ADHDL0[AGGcount].Section1;
						ADHDL0[Timer].Hazard1 = ADHDL0[Timer].Hazard1 + ADHDL0[AGGcount].Hazard1;
						ADHSL0[Timer].Section1 = ADHSL0[Timer].Section1 + ADHSL0[AGGcount].Section1;
						ADHDL0[Timer].Hazardcounter1 = ADHDL0[Timer].Hazardcounter1 + ADHDL0[AGGcount].Hazardcounter1;
						NZCounterSection1 = NZCounterSection1 +1;
					}
					
					if (ADHDL0[AGGcount].Section2 != 0)
					{
						ADHDL0[Timer].Section2 = ADHDL0[Timer].Section2 + ADHDL0[AGGcount].Section2;
						ADHDL0[Timer].Hazard2 = ADHDL0[Timer].Hazard2 + ADHDL0[AGGcount].Hazard2;
						ADHSL0[Timer].Section2 = ADHSL0[Timer].Section2 + ADHSL0[AGGcount].Section2;
						ADHDL0[Timer].Hazardcounter2 = ADHDL0[Timer].Hazardcounter2 + ADHDL0[AGGcount].Hazardcounter2;
						NZCounterSection2 = NZCounterSection2 +1;
					}

					if (ADHDL0[AGGcount].Section3 != 0)
					{
						ADHDL0[Timer].Section3 = ADHDL0[Timer].Section3 + ADHDL0[AGGcount].Section3;
						ADHDL0[Timer].Hazard3 = ADHDL0[Timer].Hazard3 + ADHDL0[AGGcount].Hazard3;
						ADHSL0[Timer].Section3 = ADHSL0[Timer].Section3 + ADHSL0[AGGcount].Section3;
						ADHDL0[Timer].Hazardcounter3 = ADHDL0[Timer].Hazardcounter3 + ADHDL0[AGGcount].Hazardcounter3;
						NZCounterSection3 = NZCounterSection3 +1;
					}

					if (ADHDL0[AGGcount].Section4 != 0)
					{
						ADHDL0[Timer].Section4 = ADHDL0[Timer].Section4 + ADHDL0[AGGcount].Section4;
						ADHDL0[Timer].Hazard4 = ADHDL0[Timer].Hazard4 + ADHDL0[AGGcount].Hazard4;
						ADHSL0[Timer].Section4 = ADHSL0[Timer].Section4 + ADHSL0[AGGcount].Section4;
						ADHDL0[Timer].Hazardcounter4 = ADHDL0[Timer].Hazardcounter4 + ADHDL0[AGGcount].Hazardcounter4;
						NZCounterSection4 = NZCounterSection4 +1;
					}

					if (ADHDL0[AGGcount].Section5 != 0)
					{
						ADHDL0[Timer].Section5 = ADHDL0[Timer].Section5 + ADHDL0[AGGcount].Section5;
						ADHDL0[Timer].Hazard5 = ADHDL0[Timer].Hazard5 + ADHDL0[AGGcount].Hazard5;
						ADHSL0[Timer].Section5 = ADHSL0[Timer].Section5 + ADHSL0[AGGcount].Section5;
						ADHDL0[Timer].Hazardcounter5 = ADHDL0[Timer].Hazardcounter5 + ADHDL0[AGGcount].Hazardcounter5;
						NZCounterSection5 = NZCounterSection5 +1;
					}

					if (ADHDL0[AGGcount].Section6 != 0)
					{
						ADHDL0[Timer].Section6 = ADHDL0[Timer].Section6 + ADHDL0[AGGcount].Section6;
						ADHDL0[Timer].Hazard6 = ADHDL0[Timer].Hazard6 + ADHDL0[AGGcount].Hazard6;
						ADHSL0[Timer].Section6 = ADHSL0[Timer].Section6 + ADHSL0[AGGcount].Section6;
						ADHDL0[Timer].Hazardcounter6 = ADHDL0[Timer].Hazardcounter6 + ADHDL0[AGGcount].Hazardcounter6;
						NZCounterSection6 = NZCounterSection6 +1;
					}

					if (ADHDL0[AGGcount].Section7 != 0)
					{
						ADHDL0[Timer].Section7 = ADHDL0[Timer].Section7 + ADHDL0[AGGcount].Section7;
						ADHDL0[Timer].Hazard7 = ADHDL0[Timer].Hazard7 + ADHDL0[AGGcount].Hazard7;
						ADHSL0[Timer].Section7 = ADHSL0[Timer].Section7 + ADHSL0[AGGcount].Section7;
						ADHDL0[Timer].Hazardcounter7 = ADHDL0[Timer].Hazardcounter7 + ADHDL0[AGGcount].Hazardcounter7;
						NZCounterSection7 = NZCounterSection7 +1;
					}

					if (ADHDL0[AGGcount].Section8 != 0)
					{
						ADHDL0[Timer].Section8 = ADHDL0[Timer].Section8 + ADHDL0[AGGcount].Section8;
						ADHDL0[Timer].Hazard8 = ADHDL0[Timer].Hazard8 + ADHDL0[AGGcount].Hazard8;
						ADHSL0[Timer].Section8 = ADHSL0[Timer].Section8 + ADHSL0[AGGcount].Section8;
						ADHDL0[Timer].Hazardcounter8 = ADHDL0[Timer].Hazardcounter8 + ADHDL0[AGGcount].Hazardcounter8;
						NZCounterSection8 = NZCounterSection8 +1;
					}

					if (ADHDL0[AGGcount].Section9 != 0)
					{
						ADHDL0[Timer].Section9 = ADHDL0[Timer].Section9 + ADHDL0[AGGcount].Section9;
						ADHDL0[Timer].Hazard9 = ADHDL0[Timer].Hazard9 + ADHDL0[AGGcount].Hazard9;
						ADHSL0[Timer].Section9 = ADHSL0[Timer].Section9 + ADHSL0[AGGcount].Section9;
						ADHDL0[Timer].Hazardcounter9 = ADHDL0[Timer].Hazardcounter9 + ADHDL0[AGGcount].Hazardcounter9;
						NZCounterSection9 = NZCounterSection9 +1;
					}

					if (ADHDL0[AGGcount].Section10 != 0)
					{
						ADHDL0[Timer].Section10 = ADHDL0[Timer].Section10 + ADHDL0[AGGcount].Section10;
						ADHDL0[Timer].Hazard10 = ADHDL0[Timer].Hazard10 + ADHDL0[AGGcount].Hazard10;
						ADHSL0[Timer].Section10 = ADHSL0[Timer].Section10 + ADHSL0[AGGcount].Section10;
						ADHDL0[Timer].Hazardcounter10 = ADHDL0[Timer].Hazardcounter10 + ADHDL0[AGGcount].Hazardcounter10;
						NZCounterSection10 = NZCounterSection10 +1;
					}
					
				}
				
				if (NZCounterSection1 != 0)
				{
					ADHDL0[Timer].Section1 = ADHDL0[Timer].Section1 / NZCounterSection1;
					ADHDL0[Timer].Hazard1 = ADHDL0[Timer].Hazard1 / NZCounterSection1;					
					ADHSL0[Timer].Section1 = ADHSL0[Timer].Section1 / NZCounterSection1;					
					ADHDL0[Timer].Hazardcounter1 = ADHDL0[Timer].Hazardcounter1 / NZCounterSection1;					
				}
				else
				{
					ADHDL0[Timer].Section1 = 0;
					ADHDL0[Timer].Hazard1 = 0;
					ADHSL0[Timer].Section1 = 0;
					ADHDL0[Timer].Hazardcounter1 = 0;
				}

				if (NZCounterSection2 != 0)
				{
					ADHDL0[Timer].Section2 = ADHDL0[Timer].Section2 / NZCounterSection2;
					ADHDL0[Timer].Hazard2 = ADHDL0[Timer].Hazard2 / NZCounterSection2;
					ADHSL0[Timer].Section2 = ADHSL0[Timer].Section2 / NZCounterSection2;
					ADHDL0[Timer].Hazardcounter2 = ADHDL0[Timer].Hazardcounter2 / NZCounterSection2;
				}
				else
				{
					ADHDL0[Timer].Section2 = 0;
					ADHDL0[Timer].Hazard2 = 0;
					ADHSL0[Timer].Section2 = 0;
					ADHDL0[Timer].Hazardcounter2 = 0;
				}

				if (NZCounterSection3 != 0)
				{
					ADHDL0[Timer].Section3 = ADHDL0[Timer].Section3 / NZCounterSection3;
					ADHDL0[Timer].Hazard3 = ADHDL0[Timer].Hazard3 / NZCounterSection3;
					ADHSL0[Timer].Section3 = ADHSL0[Timer].Section3 / NZCounterSection3;
					ADHDL0[Timer].Hazardcounter3 = ADHDL0[Timer].Hazardcounter3 / NZCounterSection3;
				}
				else
				{
					ADHDL0[Timer].Section3 = 0;
					ADHDL0[Timer].Hazard3 = 0;
					ADHSL0[Timer].Section3 = 0;
					ADHDL0[Timer].Hazardcounter3 = 0;
				}

				if (NZCounterSection4 != 0)
				{
					ADHDL0[Timer].Section4 = ADHDL0[Timer].Section4 / NZCounterSection4;
					ADHDL0[Timer].Hazard4 = ADHDL0[Timer].Hazard4 / NZCounterSection4;
					ADHSL0[Timer].Section4 = ADHSL0[Timer].Section4 / NZCounterSection4;
					ADHDL0[Timer].Hazardcounter4 = ADHDL0[Timer].Hazardcounter4 / NZCounterSection4;
				}
				else
				{
					ADHDL0[Timer].Section4 = 0;
					ADHDL0[Timer].Hazard4 = 0;
					ADHSL0[Timer].Section4 = 0;
					ADHDL0[Timer].Hazardcounter4 = 0;
				}

				if (NZCounterSection5 != 0)
				{
					ADHDL0[Timer].Section5 = ADHDL0[Timer].Section5 / NZCounterSection5;
					ADHDL0[Timer].Hazard5 = ADHDL0[Timer].Hazard5 / NZCounterSection5;
					ADHSL0[Timer].Section5 = ADHSL0[Timer].Section5 / NZCounterSection5;
					ADHDL0[Timer].Hazardcounter5 = ADHDL0[Timer].Hazardcounter5 / NZCounterSection5;
				}
				else
				{
					ADHDL0[Timer].Section5 = 0;
					ADHDL0[Timer].Hazard5 = 0;
					ADHSL0[Timer].Section5 = 0;
					ADHDL0[Timer].Hazardcounter5 = 0;
				}

				if (NZCounterSection6 != 0)
				{
					ADHDL0[Timer].Section6 = ADHDL0[Timer].Section6 / NZCounterSection6;
					ADHDL0[Timer].Hazard6 = ADHDL0[Timer].Hazard6 / NZCounterSection6;
					ADHSL0[Timer].Section6 = ADHSL0[Timer].Section6 / NZCounterSection6;
					ADHDL0[Timer].Hazardcounter6 = ADHDL0[Timer].Hazardcounter6 / NZCounterSection6;
				}
				else
				{
					ADHDL0[Timer].Section6 = 0;
					ADHDL0[Timer].Hazard6 = 0;
					ADHSL0[Timer].Section6 = 0;
					ADHDL0[Timer].Hazardcounter6 = 0;
				}

				if (NZCounterSection7 != 0)
				{
					ADHDL0[Timer].Section7 = ADHDL0[Timer].Section7 / NZCounterSection7;
					ADHDL0[Timer].Hazard7 = ADHDL0[Timer].Hazard7 / NZCounterSection7;
					ADHSL0[Timer].Section7 = ADHSL0[Timer].Section7 / NZCounterSection7;
					ADHDL0[Timer].Hazardcounter7 = ADHDL0[Timer].Hazardcounter7 / NZCounterSection7;
				}
				else
				{
					ADHDL0[Timer].Section7 = 0;
					ADHDL0[Timer].Hazard7 = 0;
					ADHSL0[Timer].Section7 = 0;
					ADHDL0[Timer].Hazardcounter7 = 0;
				}

				if (NZCounterSection8 != 0)
				{
					ADHDL0[Timer].Section8 = ADHDL0[Timer].Section8 / NZCounterSection8;
					ADHDL0[Timer].Hazard8 = ADHDL0[Timer].Hazard8 / NZCounterSection8;
					ADHSL0[Timer].Section8 = ADHSL0[Timer].Section8 / NZCounterSection8;
					ADHDL0[Timer].Hazardcounter8 = ADHDL0[Timer].Hazardcounter8 / NZCounterSection8;
				}
				else
				{
					ADHDL0[Timer].Section8 = 0;
					ADHDL0[Timer].Hazard8 = 0;
					ADHSL0[Timer].Section8 = 0;
					ADHDL0[Timer].Hazardcounter8 = 0;
				}

				if (NZCounterSection9 != 0)
				{
					ADHDL0[Timer].Section9 = ADHDL0[Timer].Section9 / NZCounterSection9;
					ADHDL0[Timer].Hazard9 = ADHDL0[Timer].Hazard9 / NZCounterSection9;
					ADHSL0[Timer].Section9 = ADHSL0[Timer].Section9 / NZCounterSection9;
					ADHDL0[Timer].Hazardcounter9 = ADHDL0[Timer].Hazardcounter9 / NZCounterSection9;
				}
				else
				{
					ADHDL0[Timer].Section9 = 0;
					ADHDL0[Timer].Hazard9 = 0;
					ADHSL0[Timer].Section9 = 0;
					ADHDL0[Timer].Hazardcounter9 = 0;
				}

				if (NZCounterSection10 != 0)
				{
					ADHDL0[Timer].Section10 = ADHDL0[Timer].Section10 / NZCounterSection10;
					ADHDL0[Timer].Hazard10 = ADHDL0[Timer].Hazard10 / NZCounterSection10;
					ADHSL0[Timer].Section10 = ADHSL0[Timer].Section10 / NZCounterSection10;
					ADHDL0[Timer].Hazardcounter10 = ADHDL0[Timer].Hazardcounter10 / NZCounterSection10;
				}
				else
				{
					ADHDL0[Timer].Section10 = 0;
					ADHDL0[Timer].Hazard10 = 0;
					ADHSL0[Timer].Section10 = 0;
					ADHDL0[Timer].Hazardcounter10 = 0;
				}


			 
				SFDResutls->SDFSpeed = ADHSL0[Timer].Section6;
				SFDResutls->SDFDensity = ADHDL0[Timer].Section6;
				SFDResutls->SDFFlow = SFDResutls->SDFSpeed * SFDResutls->SDFDensity;
					
				return(SFDResutls);
	}
	
	
				

	//---------------------------------------------------------------------

	
struct SFDValues *SFDL1 (vector <data2> ADHDL1,vector <data3> ADHSL1, int Timer)
{
				 
SFDValues *SFDResutls;
SFDResutls = NULL;
SFDResutls = (SFDValues*)malloc(sizeof(SFDValues));
				
				AGGcount = 0;
				NZCounterSection1 = 1;
				NZCounterSection2 = 1;
				NZCounterSection3 = 1;
				NZCounterSection4 = 1;
				NZCounterSection5 = 1;
				NZCounterSection6 = 1;
				NZCounterSection7 = 1;
				NZCounterSection8 = 1;
				NZCounterSection9 = 1;
				NZCounterSection10 = 1;

				AGGcount = Timer-299;
				if (AGGcount < 0)
					AGGcount  = 0;
				while (AGGcount<=Timer-1)
				{
					AGGcount = AGGcount + 1;	

				 
					if (ADHDL1[AGGcount].Section1 != 0)
					{
						ADHSL1[AGGcount].Section1=ADHSL1[AGGcount].Section1/ADHDL1[AGGcount].Section1;
					}
					else
					{
						ADHSL1[AGGcount].Section1 = 0;
					}
	
					if (ADHDL1[AGGcount].Section2 != 0)
					{
						ADHSL1[AGGcount].Section2=ADHSL1[AGGcount].Section2/ADHDL1[AGGcount].Section2;
					}
					else
					{
						ADHSL1[AGGcount].Section2 = 0;
					}
	
					if (ADHDL1[AGGcount].Section3 != 0)
					{
						ADHSL1[AGGcount].Section3=ADHSL1[AGGcount].Section3/ADHDL1[AGGcount].Section3;
					}
					else
					{
						ADHSL1[AGGcount].Section3 = 0;
					}
	
					if (ADHDL1[AGGcount].Section4 != 0)
					{
						ADHSL1[AGGcount].Section4=ADHSL1[AGGcount].Section4/ADHDL1[AGGcount].Section4;
					}
					else
					{
						ADHSL1[AGGcount].Section4 = 0;
					}

					if (ADHDL1[AGGcount].Section5 != 0)
					{
						ADHSL1[AGGcount].Section5=ADHSL1[AGGcount].Section5/ADHDL1[AGGcount].Section5;
					}
					else	
					{
						ADHSL1[AGGcount].Section5 = 0;
					}
	
					if (ADHDL1[AGGcount].Section6 != 0)
					{						
						ADHSL1[AGGcount].Section6=ADHSL1[AGGcount].Section6/ADHDL1[AGGcount].Section6;
					}
					else
					{
						ADHSL1[AGGcount].Section6 = 0;
					}
	
					if (ADHDL1[AGGcount].Section7 != 0)
					{
						ADHSL1[AGGcount].Section7=ADHSL1[AGGcount].Section7/ADHDL1[AGGcount].Section7;
					}
					else
					{
						ADHSL1[AGGcount].Section7 = 0;
					}
		
					if (ADHDL1[AGGcount].Section8 != 0)
					{
						ADHSL1[AGGcount].Section8=ADHSL1[AGGcount].Section8/ADHDL1[AGGcount].Section8;
					}
					else
					{
						ADHSL1[AGGcount].Section8 = 0;
					}
	
					if (ADHDL1[AGGcount].Section9 != 0)
					{
						ADHSL1[AGGcount].Section9=ADHSL1[AGGcount].Section9/ADHDL1[AGGcount].Section9;
					}
					else
					{
						ADHSL1[AGGcount].Section9 = 0;
					}
	
					if (ADHDL1[AGGcount].Section10 != 0)
					{
						ADHSL1[AGGcount].Section10=ADHSL1[AGGcount].Section10/ADHDL1[AGGcount].Section10;
					}
					else
					{
						ADHSL1[AGGcount].Section10 = 0;
					}
					 
				}

				
				//********************* Pre Assignment for counting the i-th one***********************
				if (ADHDL1[Timer].Section1 == 0)
				{
					NZCounterSection1 = 0;
					ADHSL1[Timer].Section1 = 0;
					ADHDL1[Timer].Hazard1 = 0;
					ADHDL1[Timer].Hazardcounter1 = 0;
				}
				if (ADHDL1[Timer].Section2 == 0)
				{
					NZCounterSection2 = 0;
					ADHSL1[Timer].Section2 = 0;
					ADHDL1[Timer].Hazard2 = 0;
					ADHDL1[Timer].Hazardcounter2 = 0;
				}
				if (ADHDL1[Timer].Section3 == 0)
				{
					NZCounterSection3 = 0;
					ADHSL1[Timer].Section3 = 0;
					ADHDL1[Timer].Hazard3 = 0;
					ADHDL1[Timer].Hazardcounter3 = 0;
				}
				if (ADHDL1[Timer].Section4 == 0)
				{
					NZCounterSection4 = 0;
					ADHSL1[Timer].Section4 = 0;
					ADHDL1[Timer].Hazard4 = 0;
					ADHDL1[Timer].Hazardcounter4 = 0;
				}
				if (ADHDL1[Timer].Section5 == 0)
				{
					NZCounterSection5 = 0;
					ADHSL1[Timer].Section5 = 0;
					ADHDL1[Timer].Hazard5 = 0;
					ADHDL1[Timer].Hazardcounter5 = 0;
				}
				if (ADHDL1[Timer].Section6 == 0)
				{
					NZCounterSection6 = 0;
					ADHSL1[Timer].Section6 = 0;
					ADHDL1[Timer].Hazard6 = 0;
					ADHDL1[Timer].Hazardcounter6 = 0;
				}
				if (ADHDL1[Timer].Section7 == 0)
				{
					NZCounterSection7 = 0;
					ADHSL1[Timer].Section7 = 0;
					ADHDL1[Timer].Hazard7 = 0;
					ADHDL1[Timer].Hazardcounter7 = 0;
				}
				if (ADHDL1[Timer].Section8 == 0)
				{
					NZCounterSection8 = 0;
					ADHSL1[Timer].Section8 = 0;
					ADHDL1[Timer].Hazard8 = 0;
					ADHDL1[Timer].Hazardcounter8 = 0;
				}
				if (ADHDL1[Timer].Section9 == 0)
				{
					NZCounterSection9 = 0;
					ADHSL1[Timer].Section9 = 0;
					ADHDL1[Timer].Hazard9 = 0;
					ADHDL1[Timer].Hazardcounter9 = 0;
				}
				if (ADHDL1[Timer].Section10 == 0)
				{
					NZCounterSection10 = 0;
					ADHSL1[Timer].Section10 = 0;
					ADHDL1[Timer].Hazard10 = 0;
					ADHDL1[Timer].Hazardcounter10 = 0;
				}
				//********************* End of Pre Assignment for counting the i-th one***********************

				
				AGGcount = Timer-299;
				if (AGGcount < 0)
					AGGcount  = 0;
				while (AGGcount<Timer-1)
				{
					AGGcount = AGGcount + 1;

					if (ADHDL1[AGGcount].Section1 != 0)
					{
						ADHDL1[Timer].Section1 = ADHDL1[Timer].Section1 + ADHDL1[AGGcount].Section1;
						ADHDL1[Timer].Hazard1 = ADHDL1[Timer].Hazard1 + ADHDL1[AGGcount].Hazard1;
						ADHSL1[Timer].Section1 = ADHSL1[Timer].Section1 + ADHSL1[AGGcount].Section1;
						ADHDL1[Timer].Hazardcounter1 = ADHDL1[Timer].Hazardcounter1 + ADHDL1[AGGcount].Hazardcounter1;
						NZCounterSection1 = NZCounterSection1 +1;
					}
					
					if (ADHDL1[AGGcount].Section2 != 0)
					{
						ADHDL1[Timer].Section2 = ADHDL1[Timer].Section2 + ADHDL1[AGGcount].Section2;
						ADHDL1[Timer].Hazard2 = ADHDL1[Timer].Hazard2 + ADHDL1[AGGcount].Hazard2;
						ADHSL1[Timer].Section2 = ADHSL1[Timer].Section2 + ADHSL1[AGGcount].Section2;
						ADHDL1[Timer].Hazardcounter2 = ADHDL1[Timer].Hazardcounter2 + ADHDL1[AGGcount].Hazardcounter2;
						NZCounterSection2 = NZCounterSection2 +1;
					}

					if (ADHDL1[AGGcount].Section3 != 0)
					{
						ADHDL1[Timer].Section3 = ADHDL1[Timer].Section3 + ADHDL1[AGGcount].Section3;
						ADHDL1[Timer].Hazard3 = ADHDL1[Timer].Hazard3 + ADHDL1[AGGcount].Hazard3;
						ADHSL1[Timer].Section3 = ADHSL1[Timer].Section3 + ADHSL1[AGGcount].Section3;
						ADHDL1[Timer].Hazardcounter3 = ADHDL1[Timer].Hazardcounter3 + ADHDL1[AGGcount].Hazardcounter3;
						NZCounterSection3 = NZCounterSection3 +1;
					}

					if (ADHDL1[AGGcount].Section4 != 0)
					{
						ADHDL1[Timer].Section4 = ADHDL1[Timer].Section4 + ADHDL1[AGGcount].Section4;
						ADHDL1[Timer].Hazard4 = ADHDL1[Timer].Hazard4 + ADHDL1[AGGcount].Hazard4;
						ADHSL1[Timer].Section4 = ADHSL1[Timer].Section4 + ADHSL1[AGGcount].Section4;
						ADHDL1[Timer].Hazardcounter4 = ADHDL1[Timer].Hazardcounter4 + ADHDL1[AGGcount].Hazardcounter4;
						NZCounterSection4 = NZCounterSection4 +1;
					}

					if (ADHDL1[AGGcount].Section5 != 0)
					{
						ADHDL1[Timer].Section5 = ADHDL1[Timer].Section5 + ADHDL1[AGGcount].Section5;
						ADHDL1[Timer].Hazard5 = ADHDL1[Timer].Hazard5 + ADHDL1[AGGcount].Hazard5;
						ADHSL1[Timer].Section5 = ADHSL1[Timer].Section5 + ADHSL1[AGGcount].Section5;
						ADHDL1[Timer].Hazardcounter5 = ADHDL1[Timer].Hazardcounter5 + ADHDL1[AGGcount].Hazardcounter5;
						NZCounterSection5 = NZCounterSection5 +1;
					}

					if (ADHDL1[AGGcount].Section6 != 0)
					{
						ADHDL1[Timer].Section6 = ADHDL1[Timer].Section6 + ADHDL1[AGGcount].Section6;
						ADHDL1[Timer].Hazard6 = ADHDL1[Timer].Hazard6 + ADHDL1[AGGcount].Hazard6;
						ADHSL1[Timer].Section6 = ADHSL1[Timer].Section6 + ADHSL1[AGGcount].Section6;
						ADHDL1[Timer].Hazardcounter6 = ADHDL1[Timer].Hazardcounter6 + ADHDL1[AGGcount].Hazardcounter6;
						NZCounterSection6 = NZCounterSection6 +1;
					}

					if (ADHDL1[AGGcount].Section7 != 0)
					{
						ADHDL1[Timer].Section7 = ADHDL1[Timer].Section7 + ADHDL1[AGGcount].Section7;
						ADHDL1[Timer].Hazard7 = ADHDL1[Timer].Hazard7 + ADHDL1[AGGcount].Hazard7;
						ADHSL1[Timer].Section7 = ADHSL1[Timer].Section7 + ADHSL1[AGGcount].Section7;
						ADHDL1[Timer].Hazardcounter7 = ADHDL1[Timer].Hazardcounter7 + ADHDL1[AGGcount].Hazardcounter7;
						NZCounterSection7 = NZCounterSection7 +1;
					}

					if (ADHDL1[AGGcount].Section8 != 0)
					{
						ADHDL1[Timer].Section8 = ADHDL1[Timer].Section8 + ADHDL1[AGGcount].Section8;
						ADHDL1[Timer].Hazard8 = ADHDL1[Timer].Hazard8 + ADHDL1[AGGcount].Hazard8;
						ADHSL1[Timer].Section8 = ADHSL1[Timer].Section8 + ADHSL1[AGGcount].Section8;
						ADHDL1[Timer].Hazardcounter8 = ADHDL1[Timer].Hazardcounter8 + ADHDL1[AGGcount].Hazardcounter8;
						NZCounterSection8 = NZCounterSection8 +1;
					}

					if (ADHDL1[AGGcount].Section9 != 0)
					{
						ADHDL1[Timer].Section9 = ADHDL1[Timer].Section9 + ADHDL1[AGGcount].Section9;
						ADHDL1[Timer].Hazard9 = ADHDL1[Timer].Hazard9 + ADHDL1[AGGcount].Hazard9;
						ADHSL1[Timer].Section9 = ADHSL1[Timer].Section9 + ADHSL1[AGGcount].Section9;
						ADHDL1[Timer].Hazardcounter9 = ADHDL1[Timer].Hazardcounter9 + ADHDL1[AGGcount].Hazardcounter9;
						NZCounterSection9 = NZCounterSection9 +1;
					}

					if (ADHDL1[AGGcount].Section10 != 0)
					{
						ADHDL1[Timer].Section10 = ADHDL1[Timer].Section10 + ADHDL1[AGGcount].Section10;
						ADHDL1[Timer].Hazard10 = ADHDL1[Timer].Hazard10 + ADHDL1[AGGcount].Hazard10;
						ADHSL1[Timer].Section10 = ADHSL1[Timer].Section10 + ADHSL1[AGGcount].Section10;
						ADHDL1[Timer].Hazardcounter10 = ADHDL1[Timer].Hazardcounter10 + ADHDL1[AGGcount].Hazardcounter10;
						NZCounterSection10 = NZCounterSection10 +1;
					}
					
				}
				
				if (NZCounterSection1 != 0)
				{
					ADHDL1[Timer].Section1 = ADHDL1[Timer].Section1 / NZCounterSection1;
					ADHDL1[Timer].Hazard1 = ADHDL1[Timer].Hazard1 / NZCounterSection1;					
					ADHSL1[Timer].Section1 = ADHSL1[Timer].Section1 / NZCounterSection1;					
					ADHDL1[Timer].Hazardcounter1 = ADHDL1[Timer].Hazardcounter1 / NZCounterSection1;					
				}
				else
				{
					ADHDL1[Timer].Section1 = 0;
					ADHDL1[Timer].Hazard1 = 0;
					ADHSL1[Timer].Section1 = 0;
					ADHDL1[Timer].Hazardcounter1 = 0;
				}

				if (NZCounterSection2 != 0)
				{
					ADHDL1[Timer].Section2 = ADHDL1[Timer].Section2 / NZCounterSection2;
					ADHDL1[Timer].Hazard2 = ADHDL1[Timer].Hazard2 / NZCounterSection2;
					ADHSL1[Timer].Section2 = ADHSL1[Timer].Section2 / NZCounterSection2;
					ADHDL1[Timer].Hazardcounter2 = ADHDL1[Timer].Hazardcounter2 / NZCounterSection2;
				}
				else
				{
					ADHDL1[Timer].Section2 = 0;
					ADHDL1[Timer].Hazard2 = 0;
					ADHSL1[Timer].Section2 = 0;
					ADHDL1[Timer].Hazardcounter2 = 0;
				}

				if (NZCounterSection3 != 0)
				{
					ADHDL1[Timer].Section3 = ADHDL1[Timer].Section3 / NZCounterSection3;
					ADHDL1[Timer].Hazard3 = ADHDL1[Timer].Hazard3 / NZCounterSection3;
					ADHSL1[Timer].Section3 = ADHSL1[Timer].Section3 / NZCounterSection3;
					ADHDL1[Timer].Hazardcounter3 = ADHDL1[Timer].Hazardcounter3 / NZCounterSection3;
				}
				else
				{
					ADHDL1[Timer].Section3 = 0;
					ADHDL1[Timer].Hazard3 = 0;
					ADHSL1[Timer].Section3 = 0;
					ADHDL1[Timer].Hazardcounter3 = 0;
				}

				if (NZCounterSection4 != 0)
				{
					ADHDL1[Timer].Section4 = ADHDL1[Timer].Section4 / NZCounterSection4;
					ADHDL1[Timer].Hazard4 = ADHDL1[Timer].Hazard4 / NZCounterSection4;
					ADHSL1[Timer].Section4 = ADHSL1[Timer].Section4 / NZCounterSection4;
					ADHDL1[Timer].Hazardcounter4 = ADHDL1[Timer].Hazardcounter4 / NZCounterSection4;
				}
				else
				{
					ADHDL1[Timer].Section4 = 0;
					ADHDL1[Timer].Hazard4 = 0;
					ADHSL1[Timer].Section4 = 0;
					ADHDL1[Timer].Hazardcounter4 = 0;
				}

				if (NZCounterSection5 != 0)
				{
					ADHDL1[Timer].Section5 = ADHDL1[Timer].Section5 / NZCounterSection5;
					ADHDL1[Timer].Hazard5 = ADHDL1[Timer].Hazard5 / NZCounterSection5;
					ADHSL1[Timer].Section5 = ADHSL1[Timer].Section5 / NZCounterSection5;
					ADHDL1[Timer].Hazardcounter5 = ADHDL1[Timer].Hazardcounter5 / NZCounterSection5;
				}
				else
				{
					ADHDL1[Timer].Section5 = 0;
					ADHDL1[Timer].Hazard5 = 0;
					ADHSL1[Timer].Section5 = 0;
					ADHDL1[Timer].Hazardcounter5 = 0;
				}

				if (NZCounterSection6 != 0)
				{
					ADHDL1[Timer].Section6 = ADHDL1[Timer].Section6 / NZCounterSection6;
					ADHDL1[Timer].Hazard6 = ADHDL1[Timer].Hazard6 / NZCounterSection6;
					ADHSL1[Timer].Section6 = ADHSL1[Timer].Section6 / NZCounterSection6;
					ADHDL1[Timer].Hazardcounter6 = ADHDL1[Timer].Hazardcounter6 / NZCounterSection6;
				}
				else
				{
					ADHDL1[Timer].Section6 = 0;
					ADHDL1[Timer].Hazard6 = 0;
					ADHSL1[Timer].Section6 = 0;
					ADHDL1[Timer].Hazardcounter6 = 0;
				}

				if (NZCounterSection7 != 0)
				{
					ADHDL1[Timer].Section7 = ADHDL1[Timer].Section7 / NZCounterSection7;
					ADHDL1[Timer].Hazard7 = ADHDL1[Timer].Hazard7 / NZCounterSection7;
					ADHSL1[Timer].Section7 = ADHSL1[Timer].Section7 / NZCounterSection7;
					ADHDL1[Timer].Hazardcounter7 = ADHDL1[Timer].Hazardcounter7 / NZCounterSection7;
				}
				else
				{
					ADHDL1[Timer].Section7 = 0;
					ADHDL1[Timer].Hazard7 = 0;
					ADHSL1[Timer].Section7 = 0;
					ADHDL1[Timer].Hazardcounter7 = 0;
				}

				if (NZCounterSection8 != 0)
				{
					ADHDL1[Timer].Section8 = ADHDL1[Timer].Section8 / NZCounterSection8;
					ADHDL1[Timer].Hazard8 = ADHDL1[Timer].Hazard8 / NZCounterSection8;
					ADHSL1[Timer].Section8 = ADHSL1[Timer].Section8 / NZCounterSection8;
					ADHDL1[Timer].Hazardcounter8 = ADHDL1[Timer].Hazardcounter8 / NZCounterSection8;
				}
				else
				{
					ADHDL1[Timer].Section8 = 0;
					ADHDL1[Timer].Hazard8 = 0;
					ADHSL1[Timer].Section8 = 0;
					ADHDL1[Timer].Hazardcounter8 = 0;
				}

				if (NZCounterSection9 != 0)
				{
					ADHDL1[Timer].Section9 = ADHDL1[Timer].Section9 / NZCounterSection9;
					ADHDL1[Timer].Hazard9 = ADHDL1[Timer].Hazard9 / NZCounterSection9;
					ADHSL1[Timer].Section9 = ADHSL1[Timer].Section9 / NZCounterSection9;
					ADHDL1[Timer].Hazardcounter9 = ADHDL1[Timer].Hazardcounter9 / NZCounterSection9;
				}
				else
				{
					ADHDL1[Timer].Section9 = 0;
					ADHDL1[Timer].Hazard9 = 0;
					ADHSL1[Timer].Section9 = 0;
					ADHDL1[Timer].Hazardcounter9 = 0;
				}

				if (NZCounterSection10 != 0)
				{
					ADHDL1[Timer].Section10 = ADHDL1[Timer].Section10 / NZCounterSection10;
					ADHDL1[Timer].Hazard10 = ADHDL1[Timer].Hazard10 / NZCounterSection10;
					ADHSL1[Timer].Section10 = ADHSL1[Timer].Section10 / NZCounterSection10;
					ADHDL1[Timer].Hazardcounter10 = ADHDL1[Timer].Hazardcounter10 / NZCounterSection10;
				}
				else
				{
					ADHDL1[Timer].Section10 = 0;
					ADHDL1[Timer].Hazard10 = 0;
					ADHSL1[Timer].Section10 = 0;
					ADHDL1[Timer].Hazardcounter10 = 0;
				}

				SFDResutls->SDFSpeed = ADHSL1[Timer].Section6;
				SFDResutls->SDFDensity = ADHDL1[Timer].Section6;
				SFDResutls->SDFFlow = SFDResutls->SDFSpeed * SFDResutls->SDFDensity;
					
				return(SFDResutls);

}


struct SFDValues *SFDLi_Wavelet (vector <Mesh_Data> ADHDLi, int Timer, int Start_Segment)
{
			
	 

	SFDValues *SFDResutls;
	SFDResutls = NULL;
	SFDResutls = (SFDValues*)malloc(sizeof(SFDValues));

	AGGcount = 0;
	int NZCounterSection = 1;
				
	//********************* Pre Assignment for counting the i-th one***********************
	if ((ADHDLi[Timer].Density[Start_Segment] == 0) && (ADHDLi[Timer].Density[Start_Segment + 1] == 0) && (ADHDLi[Timer].Density[Start_Segment + 2] == 0) && (ADHDLi[Timer].Density[Start_Segment + 3] == 0) && (ADHDLi[Timer].Density[Start_Segment + 4] == 0))
	{
		NZCounterSection = 0;
		ADHDLi[Timer].Speed[Start_Segment] = 0;
		ADHDLi[Timer].Hazard[Start_Segment] = 0;
		ADHDLi[Timer].Hazardcounter[Start_Segment] = 0;
	}
					
	//********************* End of Pre Assignment for counting the i-th one***********************

	ADHDLi[Timer].Density[Start_Segment] = ADHDLi[Timer].Density[Start_Segment] + ADHDLi[Timer].Density[Start_Segment + 1] + ADHDLi[Timer].Density[Start_Segment + 2] + ADHDLi[Timer].Density[Start_Segment + 3] + ADHDLi[Timer].Density[Start_Segment + 4];
	ADHDLi[Timer].Speed[Start_Segment] = ADHDLi[Timer].Speed[Start_Segment] + ADHDLi[Timer].Speed[Start_Segment + 1] + ADHDLi[Timer].Speed[Start_Segment + 2] + ADHDLi[Timer].Speed[Start_Segment + 3] + ADHDLi[Timer].Speed[Start_Segment + 4];
	ADHDLi[Timer].Hazard[Start_Segment] = ADHDLi[Timer].Hazard[Start_Segment] + ADHDLi[Timer].Hazard[Start_Segment + 1] + ADHDLi[Timer].Hazard[Start_Segment + 2] + ADHDLi[Timer].Hazard[Start_Segment + 3] + ADHDLi[Timer].Hazard[Start_Segment + 4];
	ADHDLi[Timer].Hazardcounter[Start_Segment] = ADHDLi[Timer].Hazardcounter[Start_Segment] + ADHDLi[Timer].Hazardcounter[Start_Segment + 1] + ADHDLi[Timer].Hazardcounter[Start_Segment + 2] + ADHDLi[Timer].Hazardcounter[Start_Segment + 3] + ADHDLi[Timer].Hazardcounter[Start_Segment + 4];
	
	AGGcount = Timer-299;
	if (AGGcount < 0)
		AGGcount  = 0;
	double aaa;
	while (AGGcount<Timer-1)
	{
		AGGcount = AGGcount + 1;					
		
		if ((ADHDLi[AGGcount].Density[Start_Segment] != 0) || (ADHDLi[AGGcount].Density[Start_Segment + 1] != 0) || (ADHDLi[AGGcount].Density[Start_Segment + 2] != 0) || (ADHDLi[AGGcount].Density[Start_Segment + 3] != 0) || (ADHDLi[AGGcount].Density[Start_Segment + 4] != 0))
		{
			ADHDLi[Timer].Density[Start_Segment] = ADHDLi[Timer].Density[Start_Segment] + ADHDLi[AGGcount].Density[Start_Segment] + ADHDLi[AGGcount].Density[Start_Segment + 1] + ADHDLi[AGGcount].Density[Start_Segment + 2] + ADHDLi[AGGcount].Density[Start_Segment + 3] + ADHDLi[AGGcount].Density[Start_Segment + 4];
			aaa = ADHDLi[AGGcount].Density[Start_Segment];
			ADHDLi[Timer].Speed[Start_Segment] = ADHDLi[Timer].Speed[Start_Segment] + ADHDLi[AGGcount].Speed[Start_Segment] + ADHDLi[AGGcount].Speed[Start_Segment + 1] + ADHDLi[AGGcount].Speed[Start_Segment + 2] + ADHDLi[AGGcount].Speed[Start_Segment + 3] + ADHDLi[AGGcount].Speed[Start_Segment + 4];
			ADHDLi[Timer].Hazard[Start_Segment] = ADHDLi[Timer].Hazard[Start_Segment] + ADHDLi[AGGcount].Hazard[Start_Segment] + ADHDLi[AGGcount].Hazard[Start_Segment + 1] + ADHDLi[AGGcount].Hazard[Start_Segment + 2] + ADHDLi[AGGcount].Hazard[Start_Segment + 3] + ADHDLi[AGGcount].Hazard[Start_Segment + 4];
			ADHDLi[Timer].Hazardcounter[Start_Segment] = ADHDLi[Timer].Hazardcounter[Start_Segment] + ADHDLi[AGGcount].Hazardcounter[Start_Segment] + ADHDLi[AGGcount].Hazardcounter[Start_Segment + 1] + ADHDLi[AGGcount].Hazardcounter[Start_Segment + 2] + ADHDLi[AGGcount].Hazardcounter[Start_Segment + 3] + ADHDLi[AGGcount].Hazardcounter[Start_Segment + 4];
			NZCounterSection = NZCounterSection + 1;
		}									
	}
	
	if (NZCounterSection != 0)
	{
		ADHDLi[Timer].Speed[Start_Segment] = ADHDLi[Timer].Speed[Start_Segment] / ADHDLi[Timer].Density[Start_Segment];
		ADHDLi[Timer].Hazard[Start_Segment] = ADHDLi[Timer].Hazard[Start_Segment] / ADHDLi[Timer].Density[Start_Segment];
		ADHDLi[Timer].Density[Start_Segment] = ADHDLi[Timer].Density[Start_Segment] / NZCounterSection;
		ADHDLi[Timer].Hazardcounter[Start_Segment] = ADHDLi[Timer].Hazardcounter[Start_Segment] / NZCounterSection1;
	}
	else
	{
		ADHDLi[Timer].Speed[Start_Segment] = 0;
		ADHDLi[Timer].Hazard[Start_Segment] = 0;
		ADHDLi[Timer].Density[Start_Segment] = 0;
		ADHDLi[Timer].Hazardcounter[Start_Segment] = 0;
	}
	 

	SFDResutls->SDFSpeed = ADHDLi[Timer].Speed[Start_Segment];
	SFDResutls->SDFDensity = ADHDLi[Timer].Density[Start_Segment] * (1000 / 100);
	SFDResutls->SDFFlow = SFDResutls->SDFSpeed * SFDResutls->SDFDensity;
		
	return(SFDResutls);
};


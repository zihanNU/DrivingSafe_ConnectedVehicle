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
#include "LaneChanging.h"

struct car * SpeedUpdate (car * r, float VSLStartLocation_temp, float VSLEndLocatio_temp, short VSLActivationFlag_temp, float DesiredSpeedLimit_temp)
{
	car * p = NULL;

	p = r;
	while (p != NULL)
	{
		if (p->x >= VSLStartLocation_temp && p->x < VSLEndLocatio_temp)
		{			
			if (VSLActivationFlag_temp == 1)
				p->Compliance = p->ComplianceValue;
			else
				p->Compliance = 0;
			
			p->Vd = (p->Compliance * p->overspeedingrate * DesiredSpeedLimit_temp)+ DesiredSpeedLimit_temp;//(p->overspeedingrate * DesiredSpeedLimit) + DesiredSpeedLimit;//DriversSpeedLimit(DesiredSpeedLimit, Compliance);
		}
		else
		{
			p->Compliance = 0;
			p->Vd = (p->Compliance * p->overspeedingrate * DesiredSpeed)+ DesiredSpeed;//(p->overspeedingrate * DesiredSpeedLimit) + DesiredSpeedLimit;//DriversSpeedLimit(DesiredSpeedLimit, Compliance);
		}
	
		p = p->next;
	}

	return (r);
};

struct car * SpeedUpdate_Ramp (car * r)
{
	car * p = NULL;

	p = r;
	while (p != NULL)
	{
		p->Compliance = 0;
		p->Vd = (p->Compliance * p->overspeedingrate * DesiredSpeed)+ DesiredSpeed;//(p->overspeedingrate * DesiredSpeedLimit) + DesiredSpeedLimit;//DriversSpeedLimit(DesiredSpeedLimit, Compliance);
		p = p->next;
	}


	return (r);
};
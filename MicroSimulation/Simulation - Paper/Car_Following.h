#include "stdafx.h"
#include <windows.h>
#include <iostream>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <float.h>
#include <dos.h>
#include <vector>
#include <string.h>
#include "Input.h"


//-----------------------------------------Car-Following-----------------------------------------
struct car* car_following (car *r1_head)
{
	//=============================================Variable Definition================================================
	car * p = NULL;
	car * r = NULL;
	car * s = NULL;
	//PT Model Parameters: Update Purposes Taken from "car" Structs
	double Gamma_D;
	double Wm_D;
	double Wc_D;
	double Tmax_D;
	double Alpha_D;
	double Beta_D;
	double Tcorr_D;

	double Deltan_D;
	double Tn_D;
	double Son_D;
	double an_D;
	double bn_D;
	double Von_D;

	double Ao_D;
	double So_D;
	double Vo_D;
	double Amax_D;

	double Wp_D;
	double DeltaW_D;
	double Sigma_D;

	double RT_D;

	//Parameters for Computing the PT Logic

	//Initializing Variables for Wallsten Model
	double Tau;
	double Seff;
	double X;

	double Z;
	double Zprime;
	double Zdoubleprime;
	double Zstar;
	double fn;
	double Azero;

	double G;
	double Gprime;
	double Gdoubleprime;
	double Uptprime;
	double Uptdoubleprime;

	double F;//Same as Uprime
	double Fprime;//Same as Udoubleprime

	double Aone;
	double Atwo;
	double Astar;//Same as Athree

	double Var;

	double At_Update;
	double Vt_Update;
	double Xt_Update;
	double Xt_Update_Previous;
	double DeltaV_Update;
	double DeltaX_Update;
	double S_Update;
	double Yt_Update;
	double Yt_previous_Update;

	double DeltaT = 0.1;
	double Random_Wiener;

	short counter_path;
	short counter_ramp;
	short ramp_check;

	int Simulation_Model;
	double rref_D;
	double S_D;
	double Vmax_D;

	float regime_choice;
	float regime_calculation;
	car * D_Total = NULL;
	int NumberOfVehicles_Total;
	int DistanceDensityBehind = 150;
	int DistanceDensityAhead = 150;
	double spacing_temp;
 
	//==========================================End of Variable Definition============================================
		
	
	p = r1_head;
	while (p!=NULL) //go through the whole list
	{
		Average_Starting_Speed[p->lane] = Average_Starting_Speed[p->lane] + p->v;
		Starting_Speed_Counter[p->lane] = Starting_Speed_Counter[p->lane] + 1;
		ramp_check = -1;
		if (p->lane >= 10) //Not in the Main Stream
		{
			counter_path = 0;
			while (Path[p->carID][counter_path] != p->lane)
				counter_path++;

			counter_ramp = 0;
			while (Path[p->carID][counter_path] != ramp_information[counter_ramp].Ramp_Number)
				counter_ramp++;
						
			if (ramp_information[counter_ramp].Ramp_Type == 0)
				ramp_check = 0;
			if (ramp_information[counter_ramp].Ramp_Type == 1)
				ramp_check = 1;
			if (ramp_information[counter_ramp].Ramp_Type == 2)
				ramp_check = 2;
		}

		
		p->Crash_Probability = 0;
		if (p->carID == -1)//61)//&& p->next == NULL)// && TIME > 885)// || p->carID == 7 )//&& p->x > 4900)//26)
		{
			cout<<"0"<<"	"<<TIME<<"	"<<p->carID<<"	"<<p->v<<"	"<<p->x<<"	"<<p->crash<<"	"<<p->lane<<"	"<<p->Duration<<"	"<<p->l<<"	"<<p->Vd<<endl;//"	"<<p->crash<<"	"<<p->next->v<<"	"<<p->next->carID<<"	"<<p->next->crash<<endl;
			if (p->next != NULL)
			{
				cout<<"1"<<"	"<<TIME<<"	"<<p->next->carID<<"	"<<p->next->v<<"	"<<p->next->x<<"	"<<p->next->crash<<"	"<<p->next->lane<<"	"<<p->next->Duration<<"	"<<p->next->l<<endl;//"	"<<p->crash<<"	"<<p->next->v<<"	"<<p->next->carID<<"	"<<p->next->crash<<endl;
				if (p->next->next != NULL)
					cout<<"2"<<"	"<<TIME<<"	"<<p->next->next->carID<<"	"<<p->next->next->v<<"	"<<p->next->next->x<<"	"<<p->next->next->crash<<"	"<<p->next->next->lane<<"	"<<p->next->next->Duration<<"	"<<p->next->next->l<<endl;//"	"<<p->crash<<"	"<<p->next->v<<"	"<<p->next->carID<<"	"<<p->next->crash<<endl;
			}
			if (p->previous != NULL)
				cout<<"-1"<<"	"<<TIME<<"	"<<p->previous->carID<<"	"<<p->previous->v<<"	"<<p->previous->x<<"	"<<p->previous->crash<<"	"<<p->previous->lane<<"	"<<p->previous->Duration<<"	"<<p->previous->l<<endl;//"	"<<p->crash<<"	"<<p->previous->v<<"	"<<p->previous->carID<<"	"<<p->previous->crash<<endl;
			
			cout<<endl;
			getch();
		}
 
		p->LeftLaneDummy = 1;
		p->RightLaneDummy = 0;
		p->RampDummy = 0;
 

 		//if (p->x > 1000 && p->x < VSLStartLocation)
		//	p->Vd = (p->Compliance * p->overspeedingrate * DesiredSpeed) + DesiredSpeed;				
		
		//if (p->x >= VSLEndLocation)//&& VSLFlag == 0)
		//	p->Vd = (p->Compliance * p->overspeedingrate * DesiredSpeed) + DesiredSpeed;				

		// end of added by ART 07/27/11 for breakdown formation

		/*if (p->Vd != 20)
		{
			cout<<p->carID<<"	"<<TIME<<"	"<<p->Vd<<"		"<<p->x<<"		"<<p->v<<endl;
			getch();
		}*/
		
		//if (TIME == 434 && p->carID == 58)
		//{
		//	cout<<"yes"<<endl;
		//	getch();
		//	getch();
		//}

		
		if (p->crash) //first car in crash
		{
			if (p->crashtimecounter < TimetoClear)
			{
				p->crashtimecounter = p->crashtimecounter + 1;
				p->v -= 0.6;//3.4;//0.6;

 				p->a_update = -0.6;//p->a_update = -3.4;//-0.6;
 
				if (p->v <= 0) //eddited by ART 07/09/11 ==> was < 0
				{
					p->v = 0.001; //eddited by ART 07/09/11 ==> was = 0

 					p->a_update = 0.0;
 				}
				p->x += p->v / 10.0;

				if (ramp_check == 0)
				{
					if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
					{
						p->x = ramp_information[counter_ramp].Ramp_EndPoint;
						p->v = 0.001;
					}
				}
				if (ramp_check == 2)
				{
					if (counter_path < Path[p->carID].size() - 1)
						if (Path[p->carID][counter_path + 1] == 1)
							if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
									Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
				}

 				p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

 				if (p->Mode != -1)
				{
					p->Mode = -1;
					p->Duration = 0.0;
					p->LCL = 0;
					
					if (p->next != NULL)
					{
						p->PreviousLeaderID = p->next->carID;
					}
					else
					{
						p->PreviousLeaderID = -1;
					}
				}
				else
				{
					p->Duration = p->Duration + 0.1;

					if (p->next != NULL)
					{
						if (p->PreviousLeaderID != p->next->carID)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = p->next->carID;
						
						}
					}
					else
					{
						if (p->PreviousLeaderID != -1)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = -1;
						}
					}
				}
 
				//Compute the Rest of Microscopic Measures of Effectiveness
				if (p->next == NULL) //no car ahead
				{
					p->headway = HEADWAY_FIRST_CAR;
					p->Spacing = HEADWAY_FIRST_CAR;
					p->DV = DV_FIRST_CAR;

					 
					//p->DXL2 = 0.0;//Possibility 9999.0
					//p->DVL2 = 0.0;//Possibility 999.0
					
					//if (p->previous == NULL)
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
					//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
					//}
 
				}
				else
				{
					p->headway = p->next->x - p->x;
					p->Spacing = p->headway - p->next->s;
					p->DV = p->v - p->next->v;

					//Added by Samer Hamdar: 14 September 2008
					//if (p->next->next == NULL)
					//{	
					//	p->DXL2 = 0.0;//Possibility 9999.9
					//	p->DVL2 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXL2 = p->next->next->x - p->x;//issue of being double the vehicles lengths ahead if crash
					//	p->DVL2 = p->v - p->next->next->v;//issue of being zero if crash
					//}

					//if (p->previous == NULL)
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
					//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
					//}
					 
				}
				 

				while ( (p->previous != NULL) && (p->previous->crash) && ( (p->x - p->s - 1.0) < p->previous->x) ) //proceed until end of 'pileup'
				{
					p->previous->crashtimecounter = p->previous->crashtimecounter + 1;
					p->previous->x = p->x - p->s;
					p->previous->v = p->v;
					 
					p->previous->a_update = 0.0;
					p->previous->headway = p->s;
					p->previous->Spacing = 0.0;
					p->previous->DV = 0.0;
					 

					 
					p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

 
					
					//if (p->next == NULL)//to make sure p->next->next defined
					//{
					//	p->DXL2 = 0.0;//Possibility 9999.9
					//	p->DVL2 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	if (p->next->next == NULL)
					//	{	
					//		p->DXL2 = 0.0;//Possibility 9999.9
					//		p->DVL2 = 0.0;//Possibility 999.0
					//	}
					//	else
					//	{
					//		p->DXL2 = p->next->next->x - p->x;//maybe needed here the length of the two vehicles ahead
					//		p->DVL2 = p->v - p->next->next->v;//maybe needed here zero
					//	}
					//}


					//if (p->previous == NULL)//: maybe not needed since already in the while loop
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//maybe needed here the vehicle length
					//	p->DVF1 = p->v - p->previous->v;//maybe needed here zero
					//}
 
					if (p->Mode != -1)
				{
					p->Mode = -1;
					p->Duration = 0.0;
					p->LCL = 0;
					
					if (p->next != NULL)
					{
						p->PreviousLeaderID = p->next->carID;
					}
					else
					{
						p->PreviousLeaderID = -1;
					}
				}
				else
				{
					p->Duration = p->Duration + 0.1;

					if (p->next != NULL)
					{
						if (p->PreviousLeaderID != p->next->carID)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = p->next->carID;
						
						}
					}
					else
					{
						if (p->PreviousLeaderID != -1)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = -1;
						}
					}
				}
 
					if (p->crashtimecounter >= TimetoClear)
					{
						if ((p->previous != NULL) && (p->next != NULL))
						{
							p->previous->next = p->next;
							p->next->previous = p->previous;
							
							//car *templist = NULL;					
							//templist = p->next;
							//(p->previous->next) = templist;
							//templist = NULL;
							//templist = p->previous;
							//(p->next->previous) = templist;				
						}
						if ((p->previous == NULL) && (p->next != NULL))
						{
							p->next->previous = NULL;
							//L0 = p->next;
						}
						if ((p->previous == NULL) && (p->next == NULL))
						{
							//L0 = L0_head = NULL;
							p = NULL;
						}
					}


					if (p->previous == NULL)
						s = p;
					p = p->previous;

				}
				if (p->previous == NULL)
						s = p;
				p=p->previous;
				continue;
			}
			else
			{
				if ((p->previous != NULL) && (p->next != NULL))
				{
					p->previous->next = p->next;
					p->next->previous = p->previous;
					
					//car *templist = NULL;					
					//templist = p->next;
					//(p->previous->next) = templist;
					//templist = NULL;
					//templist = p->previous;
					//(p->next->previous) = templist;				
				}
				if ((p->previous == NULL) && (p->next != NULL))
				{
					p->next->previous = NULL;
					//L0 = p->next;
				}
				if ((p->previous == NULL) && (p->next == NULL))
				{
					//L0 = L0_head = NULL;
					p = NULL;
				}						
			}
		}

		if (p == NULL)
		{
			break;
		}

		if (TIME > (p->react + p->departure))
			r = CopyStream(Vehicle_History[p->carID].front());
		else
			r = CopyStream(p);

		Simulation_Model = -1;
		if (r->Connectivity_Status == 0)
			Simulation_Model = 0;
		if (r->Connectivity_Status == 1)
			//if (r->next != NULL)
				//if (r->next->Connectivity_Status == 1)
					Simulation_Model = 1;
				//else
				//	Simulation_Model = 0;
			//else
			//	Simulation_Model = 0;
		if (r->Connectivity_Status == 2)
			Simulation_Model = 2;

		
 
		if (r->next == NULL && ramp_check != 0)
		{
			if (Simulation_Model == 0)
			{
 
			
				//Update Wallsten Parameters
				Gamma_D = r->Gamma;
				Wm_D = r->Wm;
				Wc_D = r->Wc;
				Tmax_D = r->Tmax;
				Alpha_D = r->Alpha;
				Beta_D = r->Beta;
				Tcorr_D = r->Tcorr;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r->So;
				Vo_D = r->Vd;//Transformed from an Earlier Model
				Amax_D = r->acc;//Transformed from an Earlier Model

				Wp_D = r->Wp;
				DeltaW_D = r->DeltaW;
				Sigma_D = r->Sigma;

				RT_D = r->Rt;
			 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r->a_update;
				Vt_Update = r->v;
				Xt_Update = r->x;
				DeltaX_Update = r->headway;
				S_Update = r->Spacing;
				DeltaV_Update = r->DV;
				Yt_Update = r->Yt;
				Yt_previous_Update = r->Yt_previous;

				At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) );
				Vt_Update = p->v;
				Vt_Update = Vt_Update + ( At_Update * DeltaT );

				if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
					Vt_Update = 0.001;

				Xt_Update = p->x;
				Xt_Update_Previous = p->x;
				Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
				if (Xt_Update_Previous > Xt_Update)
					Xt_Update = Xt_Update_Previous;

				p->a_update = At_Update;
				p->v = Vt_Update;
 
					if (p->v <=0)
					{
						p->v = 0.001;
					};
 
				p->x = Xt_Update;

				if (ramp_check == 0)
				{
					if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
					{
						p->x = ramp_information[counter_ramp].Ramp_EndPoint;
						p->v = 0.001;
					}
				}
				if (ramp_check == 2)
				{
					if (counter_path < Path[p->carID].size() - 1)
						if (Path[p->carID][counter_path + 1] == 1)
							if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
									Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
				}
				p->headway = HEADWAY_FIRST_CAR;
				p->Spacing = HEADWAY_FIRST_CAR;
				p->DV = DV_FIRST_CAR;
				p->Yt = Yt_Update;
				p->Yt_previous = Yt_previous_Update;

			 
				p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

				 
				//p->DXL2 = 0.0;//Possibility 9999.0
				//p->DVL2 = 0.0;//Possibility 999.0
			
				//if (p->previous == NULL)
				//{
				//	p->DXF1 = 0.0;//Possibility 9999.0
				//	p->DVF1 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
				//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
				//}
 
				if (p->Mode != 0)
				{
					p->Mode = 0;
					p->Duration = 0.0;
					p->LCL = 0;

					p->PreviousLeaderID = -1;
				}
				else
				{
					if (p->changelaneinstance == 1)
					{
						p->changelaneinstance = 0;
						p->Duration = 0.0;
						p->LCL = 0;
					
						p->PreviousLeaderID = -1;
					}
					else
					{
						p->Duration = p->Duration + 0.1;
					
						if ( p->PreviousLeaderID != -1)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = -1;
						}		
					}
				}
 
			}
			else if (Simulation_Model == 1)
			{
 
			
				//Update Wallsten Parameters
				Deltan_D = r->Deltan;
				Tn_D = r->Tn;
				Son_D = r->Son;
				an_D = r->an;
				bn_D = r->bn;
				Von_D = r->Von;				

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r->So;
				Vo_D = r->Vd;//Transformed from an Earlier Model
				Amax_D = r->acc;//Transformed from an Earlier Model

				RT_D = r->Rt;
 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r->a_update;
				Vt_Update = r->v;
				Xt_Update = r->x;
				DeltaX_Update = r->headway;
				S_Update = r->Spacing;
				DeltaV_Update = r->DV;
				Yt_Update = r->Yt;
				Yt_previous_Update = r->Yt_previous;

				At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) );
				Vt_Update = p->v;
				Vt_Update = Vt_Update + ( At_Update * DeltaT );

				if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
					Vt_Update = 0.001;

				Xt_Update = p->x;
				Xt_Update_Previous = p->x;
				Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
				if (Xt_Update_Previous > Xt_Update)
					Xt_Update = Xt_Update_Previous;

				p->a_update = At_Update;
				p->v = Vt_Update;
 
					if (p->v <=0)
					{
						p->v = 0.001;
					};
 
				p->x = Xt_Update;

				if (ramp_check == 0)
				{
					if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
					{
						p->x = ramp_information[counter_ramp].Ramp_EndPoint;
						p->v = 0.001;
					}
				}
				if (ramp_check == 2)
				{
					if (counter_path < Path[p->carID].size() - 1)
						if (Path[p->carID][counter_path + 1] == 1)
							if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
									Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
				}
				p->headway = HEADWAY_FIRST_CAR;
				p->Spacing = HEADWAY_FIRST_CAR;
				p->DV = DV_FIRST_CAR;
				p->Yt = Yt_Update;
				p->Yt_previous = Yt_previous_Update;

	 
				p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

		 
				//p->DXL2 = 0.0;//Possibility 9999.0
				//p->DVL2 = 0.0;//Possibility 999.0
			
				//if (p->previous == NULL)
				//{
				//	p->DXF1 = 0.0;//Possibility 9999.0
				//	p->DVF1 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
				//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
				//}
 
				if (p->Mode != 0)
				{
					p->Mode = 0;
					p->Duration = 0.0;
					p->LCL = 0;

					p->PreviousLeaderID = -1;
				}
				else
				{
					if (p->changelaneinstance == 1)
					{
						p->changelaneinstance = 0;
						p->Duration = 0.0;
						p->LCL = 0;
					
						p->PreviousLeaderID = -1;
					}
					else
					{
						p->Duration = p->Duration + 0.1;
					
						if ( p->PreviousLeaderID != -1)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = -1;
						}		
					}
				}
 
			}
			else if (Simulation_Model == 2)
			{
				if (r->next != NULL)
				{
					spacing_temp = 0;					
					spacing_temp = r->next->x - r->x;

					if (spacing_temp < Sensor_Range)
					{						
						S_D = max(0.01, min((r->next->x - r->x - r->next->s) - (r->v * r->react) + ((r->next->v * r->next->v) / (2 * fabs(r->next->decc))), Sensor_Range - (r->v * r->react)));
						Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
						Amax_D = r->acc;
						rref_D = min (min(((r->v * r->v) / 2) * ((1 / fabs(r->next->decc)) - (1 / fabs(r->decc))), r->react * r->v), 2.0);
						if (((r->next->x - r->x - r->next->s) < 15) && (r->next->v < r->v))
							At_Update = max(r->decc,  ((r->next->v * r->next->v) - (r->v * r->v)) / ((r->next->x - r->x - r->next->s)));
						else
							At_Update = max(r->decc, min(r->K * (Vmax_D - r->v), r->Ka * r->next->a_update + r->Kv * (r->next->v - r->v) + r->Kd * ((r->next->x - r->x - r->next->s) - rref_D)));//Method II
					}
					else
					{						
						S_D = Sensor_Range - (r->v * r->react);
						Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
						Amax_D = r->acc;
						//At_Update = (Amax_D * (1 - (r->v / Vmax_D)));//Method I
						At_Update = r->K * (Vmax_D - r->v);//Method II
					}
				}
				else
				{
					S_D = Sensor_Range - (r->v * r->react);
					Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
					Amax_D = r->acc;
					//At_Update = (Amax_D * (1 - (r->v / Vmax_D)));//Method I
					At_Update = r->K * (Vmax_D - r->v);//Method II
				}
						
							
				Vt_Update = p->v;
				Vt_Update = Vt_Update + ( At_Update * DeltaT );

				if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
					Vt_Update = 0.001;

				Xt_Update = p->x;
				Xt_Update_Previous = p->x;
				Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
				if (Xt_Update_Previous > Xt_Update)
					Xt_Update = Xt_Update_Previous;

				p->a_update = At_Update;
				p->v = Vt_Update;
 
					if (p->v <=0)
					{
						p->v = 0.001;
					};
 
				p->x = Xt_Update;

				if (ramp_check == 0)
				{
					if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
					{
						p->x = ramp_information[counter_ramp].Ramp_EndPoint;
						p->v = 0.001;
					}
				}
				if (ramp_check == 2)
				{
					if (counter_path < Path[p->carID].size() - 1)
						if (Path[p->carID][counter_path + 1] == 1)
							if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
									Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
				}
				p->headway = HEADWAY_FIRST_CAR;
				p->Spacing = HEADWAY_FIRST_CAR;
				p->DV = DV_FIRST_CAR;				
				//p->Yt_previous = Yt_previous_Update;

 				p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

 				//p->DXL2 = 0.0;//Possibility 9999.0
				//p->DVL2 = 0.0;//Possibility 999.0
			
				//if (p->previous == NULL)
				//{
				//	p->DXF1 = 0.0;//Possibility 9999.0
				//	p->DVF1 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
				//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
				//}
 
				if (p->Mode != 0)
				{
					p->Mode = 0;
					p->Duration = 0.0;
					p->LCL = 0;

					p->PreviousLeaderID = -1;
				}
				else
				{
					if (p->changelaneinstance == 1)
					{
						p->changelaneinstance = 0;
						p->Duration = 0.0;
						p->LCL = 0;
					
						p->PreviousLeaderID = -1;
					}
					else
					{
						p->Duration = p->Duration + 0.1;
					
						if ( p->PreviousLeaderID != -1)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = -1;
						}		
					}
				}
				 
			}
		}
		else if (r->next != NULL)
		{
			if (Simulation_Model == 0)
			{
	 
			
				//Update Wallsten Parameters
				Gamma_D = r->Gamma;
				Wm_D = r->Wm;
				Wc_D = r->Wc;
				Tmax_D = r->Tmax;
				Alpha_D = r->Alpha;
				Beta_D = r->Beta;
				Tcorr_D = r->Tcorr;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r->So;
				Vo_D = r->Vd;//Transformed from an Earlier Model
				Amax_D = r->acc;//Transformed from an Earlier Model

				Wp_D = r->Wp;
				DeltaW_D = r->DeltaW;
				Sigma_D = r->Sigma;

				RT_D = r->Rt;
				//End of Added by Samer Hamdar: 10 September 2008

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r->a_update;
				Vt_Update = r->v;
				Xt_Update = r->x;
				DeltaX_Update = r->next->x - r->x;
				S_Update = r->next->x - r->next->s - r->x;
				DeltaV_Update = r->v - r->next->v;
				Yt_Update = r->Yt;
				Yt_previous_Update = r->Yt_previous;

				//Test: Seff: Martin's Version: Keep for Updating Purposes: Keep Errors Defined
				//if ( ( S_Update - So_D ) > 0.0 )
				if ( ( S_Update - So_D ) > 0.1 )
					Seff = ( S_Update - So_D );
				else
					Seff = 0.1;
					//Seff = 0.0;

				//Tau
				if ( DeltaV_Update > ( Seff / Tmax_D ) )
					Tau =  ( Seff / DeltaV_Update );
				else
					Tau = Tmax_D;

				////Test
				//if (Vt_Update < 0.1)
				//	Vt_Update = 0.1;

				//Zprime
				Zprime = ( Tau / ( 2.0 * Alpha_D * Vt_Update ) );

				//Zdoubleprime
				Zdoubleprime = 0.0;
			

				//Zstar
				Zstar = ( - sqrt ( 2.0 * log ( ( Ao_D * Wc_D * Zprime ) / ( sqrt ( 2.0 * PI ) ) ) ) );

				//Astar : Initial Estimate
				Astar = ( ( 2.0 / Tau) * ( ( Seff / Tau) - DeltaV_Update + ( Alpha_D * Vt_Update * Zstar ) ) );
				

				regime_choice = mrand(20);//(rand () % 1000) / 1000;
				D_Total = NULL;
				NumberOfVehicles_Total = 0;

				if (r->next != NULL)
				{
					if (r->previous != NULL)
					{
						D_Total = r1_head;						
						if (D_Total != NULL)
						{
							while (D_Total != NULL)
							{						
								if (( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
									NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
								D_Total = D_Total->previous;
							}
						}
						else
							NumberOfVehicles_Total = 0;
						regime_calculation = -37.8195 + 1.7535 * r->v + 0.0459 * (r->next->x - r->x) + 0.3259 * (r->next->v - r->v) +0.0931 * (r->x - r->previous->x) + -1.0300 * (r->v - r->previous->v) + 0.5911 * (NumberOfVehicles_Total);
					}
					else
					{
						D_Total = r1_head;						
						if (D_Total != NULL)
						{
							while (D_Total != NULL)
							{						
								if (( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
									NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
								D_Total = D_Total->previous;
							}
						}
						else
							NumberOfVehicles_Total = 0;
						regime_calculation = -37.8195 + 1.7535 * r->v + 0.0459 * (r->next->x - r->x) + 0.3259 * (r->next->v - r->v) +0.0931 * (100) + -1.0300 * (0) + 0.5911 * (NumberOfVehicles_Total);
					}
				}
				else
				{
					if (r->previous != NULL)
					{
						D_Total = r1_head;						
						if (D_Total != NULL)
						{
							while (D_Total != NULL)
							{						
								if (( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
									NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
								D_Total = D_Total->previous;
							}
						}
						else
							NumberOfVehicles_Total = 0;
						regime_calculation = -37.8195 + 1.7535 * r->v + 0.0459 * (100) + 0.3259 * (0) +0.0931 * (r->x - r->previous->x) + -1.0300 * (r->v - r->previous->v) + 0.5911 * (NumberOfVehicles_Total);
					}
					else
					{
						D_Total = r1_head;						
						if (D_Total != NULL)
						{
							while (D_Total != NULL)
							{						
								if (( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
									NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
								D_Total = D_Total->previous;
							}
						}
						else
							NumberOfVehicles_Total = 0;
						regime_calculation = -37.8195 + 1.7535 * r->v + 0.0459 * (100) + 0.3259 * (0) +0.0931 * (100) + -1.0300 * (0) + 0.5911 * (NumberOfVehicles_Total);
					}
				}

				regime_calculation = exp(regime_calculation) / (1 + exp(regime_calculation));
				//regime_choice = -100000.0;
				if (regime_choice < regime_calculation)
				{
					//Newton's Method loop
					for ( int NewtonCounter = 0; NewtonCounter < 4; NewtonCounter++ )
					{
						X = ( Astar / Ao_D );
					
						G = ( X / ( pow ( ( 1 + pow ( X , 2.0 ) ) , Gamma_D ) ) );//G = ( X / ( pow ( ( 1 + pow ( X , 2.0 ) ) , Sigma_D ) ) ); 
					
						Gprime = ( ( 1.0 / ( pow ( ( 1.0 + pow ( X , 2.0 )), Gamma_D ) ) ) - ( ( 2.0 * Gamma_D * pow ( X , 2.0 ) ) / ( pow ( ( 1 + pow ( X , 2.0 ) ) ,( Gamma_D + 1.0) ) ) ) ); 
						Gdoubleprime = ( ( ( - 6.0 * Gamma_D * X ) / ( pow ( ( 1.0 + pow ( X , 2.0 ) ) ,( Gamma_D + 1) ) ) ) + ( ( 4.0 * Gamma_D * ( Gamma_D + 1 ) * pow ( X , 3.0) ) / ( pow ( ( 1 + pow ( X , 2.0 ) ) ,( Gamma_D + 2.0) ) ) ) );
						//G = ( X * ( pow ( ( 1 + pow ( X , 2.0 ) ) , (Gamma_D / 2) ) ) );
						//Gprime = (pow((1 + pow(X, 2.0)),(Gamma_D / 2))) + (Gamma_D * pow(X , 2) * pow((1 + pow(X, 2.0)),((Gamma_D / 2) - 1)));
						//Gdoubleprime = (Gamma_D * X * pow((1 + pow(X, 2.0)),(Gamma_D / 2))) + (2 * Gamma_D * X * pow((1 + pow(X, 2.0)),((Gamma_D / 2) - 1))) + (2 * Gamma_D * ((Gamma_D / 2) - 1) * pow(X , 3) * pow((1 + pow(X, 2.0)),((Gamma_D / 2) - 2)));

						Uptprime = ( ( 1.0 / Ao_D ) *  ( ( ( Wm_D + ( ( DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gprime) ) + ( ( DeltaW_D * G ) / ( 2 * pow ( cosh ( X ) , 2 ) ) ) ) );
						Uptdoubleprime = ( ( 1.0 / ( pow (Ao_D , 2.0 ) ) ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gdoubleprime) ) + ( ( ( DeltaW_D ) / ( pow ( cosh ( X ) , 2.0 ) ) ) * ( Gprime - ( tanh ( X ) * G ) ) ) ) );

						Z = ( ( DeltaV_Update + ( ( 0.5 ) * Astar * Tau ) - ( Seff / Tau) ) / ( Alpha_D * Vt_Update ) );
						fn = ( 1.0 / sqrt ( 2 * PI) ) * exp (  - pow ( Z , 2.0) / 2.0 );
					
						F = Uptprime - Wc_D * fn * Zprime;
						Fprime = Uptdoubleprime - Wc_D * fn * ( Z * pow ( Zprime, 2.0) + Zdoubleprime );
					
						Astar = Astar - ( F / Fprime);
						//if (Astar >=100)
						//	Astar = 100;
						//if (Astar <= -100)
						//	Astar = -100;
					}

					//Compute U'' or F' for the last time for variance computation
				
					X = ( Astar / Ao_D );
					
					G = ( X / ( pow ( ( 1.0 + pow ( X , 2.0 ) ) , Gamma_D ) ) ); 
				
					Gprime = ( ( 1.0 / ( pow ( ( 1 + pow ( X , 2.0 )), Gamma_D ) ) ) - ( ( 2.0 * Gamma_D * pow ( X , 2.0 ) ) / ( pow ( ( 1 + pow ( X , 2.0 ) ) ,( Gamma_D + 1.0) ) ) ) ); 
					Gdoubleprime = ( ( ( - 6.0 * Gamma_D * X ) / ( pow ( ( 1.0 + pow ( X , 2.0 ) ) ,( Gamma_D + 1.0) ) ) ) + ( ( 4.0 * Gamma_D * ( Gamma_D + 1.0 ) * pow ( X , 3.0) ) / ( pow ( ( 1.0 + pow ( X , 2.0 ) ) ,( Gamma_D + 2.0) ) ) ) );

					Uptprime = ( ( 1.0 / Ao_D ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gprime) ) + ( ( DeltaW_D * G ) / ( 2 * pow ( cosh ( X ) , 2 ) ) ) ) );
					Uptdoubleprime = ( ( 1.0 / ( pow (Ao_D , 2.0 ) ) ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1.0 + tanh ( X ) ) ) / 2.0 ) ) * ( Gdoubleprime) ) + ( ( ( DeltaW_D ) / ( pow ( cosh ( X ) , 2.0 ) ) ) * ( Gprime - ( tanh ( X ) * G ) ) ) ) );

					Z = ( ( DeltaV_Update + ( ( 0.5 ) * Astar * Tau ) - ( Seff / Tau) ) / ( Alpha_D * Vt_Update ) );
					fn = ( 1.0 / sqrt ( 2 * PI ) ) * exp (  - pow ( Z , 2.0) / 2.0 );
				
					F = Uptprime - Wc_D * fn * Zprime;
					Fprime = Uptdoubleprime - Wc_D * fn * ( Z * pow ( Zprime, 2.0) + Zdoubleprime );

					Var = -1.0 / ( Beta_D * Fprime );

					//Random_Wiener = ( (rand()%1000) / 1000.0 );
					Random_Wiener = ( mrand( 38 ) ); //Return a Uniform Number Between Zero and One
					Yt_previous_Update = Yt_Update;

					//Yt_Update = Yt_Update * exp ( - DeltaT / Tau) + sqrt ( 24.0 * DeltaT / Tau ) * Random_Wiener; //Modified if the Wiener Process Takes the DT as the Reaction Time
					Yt_Update = Yt_Update * exp ( - RT_D / Tau) + sqrt ( 24.0 * RT_D / Tau ) * Random_Wiener;

					At_Update = Astar + Var * Yt_Update;

					 
					//Test
					//Incorporate Free-Speed Contrained Acceleration and Maximum Deceleration (Maximum Acceleration Just in Case
				}
				else
				{
					//Newton's Method loop
					for ( int NewtonCounter = 0; NewtonCounter < 4; NewtonCounter++ )
					{
						X = ( Astar / Ao_D );					
						
						if (X >= 0)
							G = pow(pow(X, 2.0), Gamma_D);
						else
							G = -1.0 * pow(pow(X, 2.0), Gamma_D);

						if (X >= 0)
							//Gprime = 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1);
							Gprime = 2.0 * Gamma_D * X * pow(pow(X, 2.0), Gamma_D - 1);
						else
							//Gprime = -1.0 * 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1);
							Gprime = -1.0 * 2.0 * Gamma_D * X * pow(pow(X, 2.0), Gamma_D - 1);

						if (X >= 0)
							//Gdoubleprime = Gamma_D * (Gamma_D - 1) * pow(X, Gamma_D - 2);
							Gdoubleprime = 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1) + 2.0 * Gamma_D * (Gamma_D - 1) * X * pow(pow(X, 2.0), Gamma_D - 2);
						else
							//Gdoubleprime = -1 * Gamma_D * (Gamma_D - 1) * pow(X, Gamma_D - 2);
							Gdoubleprime = -1.0 * 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1) + 2.0 * Gamma_D * (Gamma_D - 1) * X * pow(pow(X, 2.0), Gamma_D - 2);
						
						Uptprime = ( ( 1.0 / Ao_D ) *  ( ( ( Wm_D + ( ( DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gprime) ) + ( ( DeltaW_D * G ) / ( 2 * pow ( cosh ( X ) , 2 ) ) ) ) );
						Uptdoubleprime = ( ( 1.0 / ( pow (Ao_D , 2.0 ) ) ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gdoubleprime) ) + ( ( ( DeltaW_D ) / ( pow ( cosh ( X ) , 2.0 ) ) ) * ( Gprime - ( tanh ( X ) * G ) ) ) ) );

						Z = ( ( DeltaV_Update + ( ( 0.5 ) * Astar * Tau ) - ( Seff / Tau) ) / ( Alpha_D * Vt_Update ) );
						fn = ( 1.0 / sqrt ( 2 * PI) ) * exp (  - pow ( Z , 2.0) / 2.0 );
					
						F = Uptprime - Wc_D * fn * Zprime;
						Fprime = Uptdoubleprime - Wc_D * fn * ( Z * pow ( Zprime, 2.0) + Zdoubleprime );
					
						Astar = Astar - ( F / Fprime);
						//if (Astar >=100)
						//	Astar = 100;
						//if (Astar <= -100)
						//	Astar = -100;
					}

					//Compute U'' or F' for the last time for variance computation
				
					X = ( Astar / Ao_D );
					
					if (X >= 0)
						G = pow(pow(X, 2.0), Gamma_D);
					else
						G = -1.0 * pow(pow(X, 2.0), Gamma_D);

					if (X >= 0)
						//Gprime = 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1);
						Gprime = 2.0 * Gamma_D * X * pow(pow(X, 2.0), Gamma_D - 1);
					else
						//Gprime = -1.0 * 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1);
						Gprime = -1.0 * 2.0 * Gamma_D * X * pow(pow(X, 2.0), Gamma_D - 1);

					if (X >= 0)
						//Gdoubleprime = Gamma_D * (Gamma_D - 1) * pow(X, Gamma_D - 2);
						Gdoubleprime = 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1) + 2.0 * Gamma_D * (Gamma_D - 1) * X * pow(pow(X, 2.0), Gamma_D - 2);
					else
						//Gdoubleprime = -1 * Gamma_D * (Gamma_D - 1) * pow(X, Gamma_D - 2);
						Gdoubleprime = -1.0 * 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1) + 2.0 * Gamma_D * (Gamma_D - 1) * X * pow(pow(X, 2.0), Gamma_D - 2);

					Uptprime = ( ( 1.0 / Ao_D ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gprime) ) + ( ( DeltaW_D * G ) / ( 2 * pow ( cosh ( X ) , 2 ) ) ) ) );
					Uptdoubleprime = ( ( 1.0 / ( pow (Ao_D , 2.0 ) ) ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1.0 + tanh ( X ) ) ) / 2.0 ) ) * ( Gdoubleprime) ) + ( ( ( DeltaW_D ) / ( pow ( cosh ( X ) , 2.0 ) ) ) * ( Gprime - ( tanh ( X ) * G ) ) ) ) );

					Z = ( ( DeltaV_Update + ( ( 0.5 ) * Astar * Tau ) - ( Seff / Tau) ) / ( Alpha_D * Vt_Update ) );
					fn = ( 1.0 / sqrt ( 2 * PI ) ) * exp (  - pow ( Z , 2.0) / 2.0 );
				
					F = Uptprime - Wc_D * fn * Zprime;
					Fprime = Uptdoubleprime - Wc_D * fn * ( Z * pow ( Zprime, 2.0) + Zdoubleprime );

					Var = -1.0 / ( Beta_D * Fprime );

					//Random_Wiener = ( (rand()%1000) / 1000.0 );
					Random_Wiener = ( mrand( 38 ) ); //Return a Uniform Number Between Zero and One
					Yt_previous_Update = Yt_Update;

					//Yt_Update = Yt_Update * exp ( - DeltaT / Tau) + sqrt ( 24.0 * DeltaT / Tau ) * Random_Wiener; //Modified if the Wiener Process Takes the DT as the Reaction Time
					Yt_Update = Yt_Update * exp ( - RT_D / Tau) + sqrt ( 24.0 * RT_D / Tau ) * Random_Wiener;

					At_Update = Astar + Var * Yt_Update;

					 
					//Test
					//Incorporate Free-Speed Contrained Acceleration and Maximum Deceleration (Maximum Acceleration Just in Case

				}

				if ( At_Update < r->decc)
					At_Update = r->decc;
				if ( At_Update > r->acc)
					At_Update = r->acc;
		
				if (p->next == NULL)
				{
				
					if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) ) ) // Current Episode is Free-Flow Episode
						At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) );

					Vt_Update = p->v;
					Vt_Update = Vt_Update + ( At_Update * DeltaT );
				
					Xt_Update = p->x;
					Xt_Update_Previous = p->x;
					Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
					if (Xt_Update_Previous > Xt_Update)
						Xt_Update = Xt_Update_Previous;
				
					p->a_update = At_Update;
					p->v = Vt_Update;
					 
						if (p->v <=0)
						{
							p->v = 0.001;
						};
					 
					p->x = Xt_Update;
					if (ramp_check == 0)
					{
						if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
						{
							p->x = ramp_information[counter_ramp].Ramp_EndPoint;
							p->v = 0.001;
						}
					}
					if (ramp_check == 2)
					{
						if (counter_path < Path[p->carID].size() - 1)
							if (Path[p->carID][counter_path + 1] == 1)
								if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
									for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
										Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
					}
					p->headway = HEADWAY_FIRST_CAR;
					p->Spacing = HEADWAY_FIRST_CAR;
					p->DV = DV_FIRST_CAR;
					p->Yt = Yt_Update;
					p->Yt_previous = Yt_previous_Update;

					 
					p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

					 
					//p->DXL2 = 0.0;//Possibility 9999.0
					//p->DVL2 = 0.0;//Possibility 999.0
				
					//if (p->previous == NULL)
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
					//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
					//}
	 
 
					if (p->Mode != 0)
					{
						p->Mode = 0;
						p->Duration = 0.0;
						p->LCL = 0;

						p->PreviousLeaderID = -1;
					}
					else
					{
						if (p->changelaneinstance == 1)
						{
							p->changelaneinstance = 0;
							p->Duration = 0.0;
							p->LCL = 0;
						
							p->PreviousLeaderID = -1;
						}
						else
						{
							p->Duration = p->Duration + 0.1;
						
							if ( p->PreviousLeaderID != -1)
							{
								p->LCL = p->LCL + 1;
								p->PreviousLeaderID = -1;
							}		
						}
					}
		 
				}
				else
				{
					if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) ) ) // Current Episode is Free-Flow Episode
					{
						At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) );
					
						if ( p->Mode == 1 )//Previous Episode was a CF Episode
						{
							p->Mode = 0;
							p->Duration = 0.0;
							p->LCL = 0;

							p->PreviousLeaderID = p->next->carID;
					
						}
						else//Previous Episode was a FF Episoe
						{
							if (p->changelaneinstance == 1) // If driver changed lanes
							{
								p->changelaneinstance = 0;
								p->Mode = 0;//not essential
								p->Duration = 0.0;
								p->LCL = 0;

								p->PreviousLeaderID = p->next->carID;
							}
							else //drivers did not change lanes: same episode
							{
								p->Mode = 0;//not essential
								p->Duration = p->Duration + 0.1;

								if (p->PreviousLeaderID != p->next->carID)
								{
									p->LCL = p->LCL + 1;
									p->PreviousLeaderID = p->next->carID;
								}
							}
						}
					}
					else //Current Episode is Car-Following Episode
					{
						if ( p->Mode == 0 )//Previous Episode was a FF Episode
						{
							p->Mode = 1;
							p->Duration = 0.0;
							p->LCL = 0;

							p->PreviousLeaderID = p->next->carID;
					
						}
						else//Previous Episode was a CF Episoe
						{
							if (p->changelaneinstance == 1) // If driver changed lanes
							{
								p->changelaneinstance = 0;
								p->Mode = 1;//not essential
								p->Duration = 0.0;
								p->LCL = 0;

								p->PreviousLeaderID = p->next->carID;
							}
							else //drivers did not change lanes: same episode
							{
								p->Mode = 1;//not essential
								p->Duration = p->Duration + 0.1;

								if (p->PreviousLeaderID != p->next->carID)
								{
									p->LCL = p->LCL + 1;
									p->PreviousLeaderID = p->next->carID;
								}
							}
						}
					}
			 
					
				
					//if ( At_Update > 4.0 )
					//	At_Update = 4.0;
					//if ( At_Update < -6.0 )  //Problem: Test: Put lower limit on acceleration (?)
					//	At_Update = -6.0;


					//Find the Resulting Velocity, Relative Velocity, Location, Relative Headway for Updating the Next Acceleration
					Vt_Update = p->v;
					Vt_Update = Vt_Update + ( At_Update * DeltaT );
				
					if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
						Vt_Update = 0.001;
				
					Xt_Update = p->x;
					Xt_Update_Previous = p->x;
					Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
					if (Xt_Update_Previous > Xt_Update)
						Xt_Update = Xt_Update_Previous;
					DeltaX_Update = p->next->x - Xt_Update;
					S_Update = DeltaX_Update - p->next->s;
					
					/*if (S_Update <= 0)
					{
						S_Update = 0.0;
						Vt_Update = 0.0;
						At_Update = 0.0;
						p->crash = 1;

					}*///Left for the Crash Detection Section				
				
					DeltaV_Update = Vt_Update - p->next->v;
				
					//Store in Wallsten Measures for Each Frame
					p->a_update = At_Update;
					p->v = Vt_Update;
					 
						if (p->v <=0)
						{
							p->v = 0.001;
						};
			 
					p->x = Xt_Update;
					if (ramp_check == 0)
					{
						if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
						{
							p->x = ramp_information[counter_ramp].Ramp_EndPoint;
							p->v = 0.001;
						}
					}
					if (ramp_check == 2)
					{
						if (counter_path < Path[p->carID].size() - 1)
							if (Path[p->carID][counter_path + 1] == 1)
								if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
									for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
										Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
					}
					p->DV = DeltaV_Update;
					p->headway = DeltaX_Update;
					p->Spacing = S_Update;//Possible Error: Problem Since Space Gaps are Sometimes Negative: Use Space Headway Instead
					p->Yt = Yt_Update;
					p->Yt_previous = Yt_previous_Update;

				 
					p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

					//  (not necessary since does not go into equations; just for output if needed
					//if (p->next->next == NULL)
					//{	
					//	p->DXL2 = 0.0;//Possibility 9999.9
					//	p->DVL2 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXL2 = p->next->next->x - p->x;//maybe needed here the length of the two vehicles ahead
					//	p->DVL2 = p->v - p->next->next->v;//maybe needed here zero
					//}

					//if (p->previous == NULL)
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//maybe needed here the vehicle length
					//	p->DVF1 = p->v - p->previous->v;//maybe needed here zero
					//}
					 
				}

				 
			}
			else if (Simulation_Model == 1)
			{
				 
			
				//Update Wallsten Parameters
				Deltan_D = r->Deltan;
				Tn_D = r->Tn;
				Son_D = r->Son;
				an_D = r->an;
				bn_D = r->bn;
				Von_D = r->Von;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r->So;
				Vo_D = r->Vd;//Transformed from an Earlier Model
				Amax_D = r->acc;//Transformed from an Earlier Model

				Wp_D = r->Wp;
				DeltaW_D = r->DeltaW;
				Sigma_D = r->Sigma;

				RT_D = r->Rt;
				 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r->a_update;
				Vt_Update = r->v;
				Xt_Update = r->x;
				DeltaX_Update = r->next->x - r->x;
				S_Update = r->next->x - r->next->s - r->x;
				DeltaV_Update = r->v - r->next->v;
				Yt_Update = r->Yt;
				Yt_previous_Update = r->Yt_previous;

				At_Update = an_D * (1 - pow((Vt_Update/Von_D),Deltan_D) - ((Son_D + (Tn_D * Vt_Update) + ((Vt_Update * DeltaV_Update)/(2 * sqrt(an_D * bn_D))))/S_Update));
				
						
				if (p->next == NULL)
				{
				
					if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) ) ) // Current Episode is Free-Flow Episode
						At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) );

					Vt_Update = p->v;
					Vt_Update = Vt_Update + ( At_Update * DeltaT );
				
					Xt_Update = p->x;
					Xt_Update_Previous = p->x;
					Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
					if (Xt_Update_Previous > Xt_Update)
						Xt_Update = Xt_Update_Previous;
				
					p->a_update = At_Update;
					p->v = Vt_Update;
					 
						if (p->v <=0)
						{
							p->v = 0.001;
						};
					 
					p->x = Xt_Update;
					if (ramp_check == 0)
					{
						if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
						{
							p->x = ramp_information[counter_ramp].Ramp_EndPoint;
							p->v = 0.001;
						}
					}
					if (ramp_check == 2)
					{
						if (counter_path < Path[p->carID].size() - 1)
							if (Path[p->carID][counter_path + 1] == 1)
								if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
									for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
										Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
					}
					p->headway = HEADWAY_FIRST_CAR;
					p->Spacing = HEADWAY_FIRST_CAR;
					p->DV = DV_FIRST_CAR;
					p->Yt = Yt_Update;
					p->Yt_previous = Yt_previous_Update;

					 
					p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

					//Added by Samer Hamdar: 14 September 2008
					//p->DXL2 = 0.0;//Possibility 9999.0
					//p->DVL2 = 0.0;//Possibility 999.0
				
					//if (p->previous == NULL)
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
					//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
					//}
	 
					if (p->Mode != 0)
					{
						p->Mode = 0;
						p->Duration = 0.0;
						p->LCL = 0;

						p->PreviousLeaderID = -1;
					}
					else
					{
						if (p->changelaneinstance == 1)
						{
							p->changelaneinstance = 0;
							p->Duration = 0.0;
							p->LCL = 0;
						
							p->PreviousLeaderID = -1;
						}
						else
						{
							p->Duration = p->Duration + 0.1;
						
							if ( p->PreviousLeaderID != -1)
							{
								p->LCL = p->LCL + 1;
								p->PreviousLeaderID = -1;
							}		
						}
					}
					 
				}
				else
				{
					if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) ) ) // Current Episode is Free-Flow Episode
					{
						At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) );
					
						if ( p->Mode == 1 )//Previous Episode was a CF Episode
						{
							p->Mode = 0;
							p->Duration = 0.0;
							p->LCL = 0;

							p->PreviousLeaderID = p->next->carID;
					
						}
						else//Previous Episode was a FF Episoe
						{
							if (p->changelaneinstance == 1) // If driver changed lanes
							{
								p->changelaneinstance = 0;
								p->Mode = 0;//not essential
								p->Duration = 0.0;
								p->LCL = 0;

								p->PreviousLeaderID = p->next->carID;
							}
							else //drivers did not change lanes: same episode
							{
								p->Mode = 0;//not essential
								p->Duration = p->Duration + 0.1;

								if (p->PreviousLeaderID != p->next->carID)
								{
									p->LCL = p->LCL + 1;
									p->PreviousLeaderID = p->next->carID;
								}
							}
						}
					}
					else //Current Episode is Car-Following Episode
					{
						if ( p->Mode == 0 )//Previous Episode was a FF Episode
						{
							p->Mode = 1;
							p->Duration = 0.0;
							p->LCL = 0;

							p->PreviousLeaderID = p->next->carID;
					
						}
						else//Previous Episode was a CF Episoe
						{
							if (p->changelaneinstance == 1) // If driver changed lanes
							{
								p->changelaneinstance = 0;
								p->Mode = 1;//not essential
								p->Duration = 0.0;
								p->LCL = 0;

								p->PreviousLeaderID = p->next->carID;
							}
							else //drivers did not change lanes: same episode
							{
								p->Mode = 1;//not essential
								p->Duration = p->Duration + 0.1;

								if (p->PreviousLeaderID != p->next->carID)
								{
									p->LCL = p->LCL + 1;
									p->PreviousLeaderID = p->next->carID;
								}
							}
						}
					}
					 
					
				
					//if ( At_Update > 4.0 )
					//	At_Update = 4.0;
					//if ( At_Update < -6.0 )  //Problem: Test: Put lower limit on acceleration (?)
					//	At_Update = -6.0;


					//Find the Resulting Velocity, Relative Velocity, Location, Relative Headway for Updating the Next Acceleration
					Vt_Update = p->v;
					Vt_Update = Vt_Update + ( At_Update * DeltaT );
				
					if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
						Vt_Update = 0.001;
				
					Xt_Update = p->x;
					Xt_Update_Previous = p->x;
					Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
					if (Xt_Update_Previous > Xt_Update)
						Xt_Update = Xt_Update_Previous;
					DeltaX_Update = p->next->x - Xt_Update;
					S_Update = DeltaX_Update - p->next->s;
					
					/*if (S_Update <= 0)
					{
						S_Update = 0.0;
						Vt_Update = 0.0;
						At_Update = 0.0;
						p->crash = 1;

					}*///Left for the Crash Detection Section				
				
					DeltaV_Update = Vt_Update - p->next->v;
				
					//Store in Wallsten Measures for Each Frame
					p->a_update = At_Update;
					p->v = Vt_Update;
					 
						if (p->v <=0)
						{
							p->v = 0.001;
						};
					 
					p->x = Xt_Update;
					if (ramp_check == 0)
					{
						if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
						{
							p->x = ramp_information[counter_ramp].Ramp_EndPoint;
							p->v = 0.001;
						}
					}
					if (ramp_check == 2)
					{
						if (counter_path < Path[p->carID].size() - 1)
							if (Path[p->carID][counter_path + 1] == 1)
								if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
									for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
										Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
					}
					p->DV = DeltaV_Update;
					p->headway = DeltaX_Update;
					p->Spacing = S_Update;//Possible Error: Problem Since Space Gaps are Sometimes Negative: Use Space Headway Instead
					p->Yt = Yt_Update;
					p->Yt_previous = Yt_previous_Update;

	 
					p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

 					//if (p->next->next == NULL)
					//{	
					//	p->DXL2 = 0.0;//Possibility 9999.9
					//	p->DVL2 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXL2 = p->next->next->x - p->x;//maybe needed here the length of the two vehicles ahead
					//	p->DVL2 = p->v - p->next->next->v;//maybe needed here zero
					//}

					//if (p->previous == NULL)
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//maybe needed here the vehicle length
					//	p->DVF1 = p->v - p->previous->v;//maybe needed here zero
					//}
					 
				}
 
			}
			else if (Simulation_Model == 2)
			{
				if (r->next != NULL)
				{
					spacing_temp = 0;
					spacing_temp = r->next->x - r->x;

					if (spacing_temp < Sensor_Range)
					{
						S_D = max(0.01, min((r->next->x - r->x - r->next->s) - (r->v * r->react) + ((r->next->v * r->next->v) / (2 * fabs(r->next->decc))), Sensor_Range - (r->v * r->react)));
						Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
						Amax_D = r->acc;
						rref_D = min (min(((r->v * r->v) / 2) * ((1 / fabs(r->next->decc)) - (1 / fabs(r->decc))), r->react * r->v), 2.0);						
						if (((r->next->x - r->x - r->next->s) < 15) && (r->next->v < r->v))
							At_Update = max(r->decc,  ((r->next->v * r->next->v) - (r->v * r->v)) / ((r->next->x - r->x - r->next->s)));
						else
							At_Update = max(r->decc, min(r->K * (Vmax_D - r->v), r->Ka * r->next->a_update + r->Kv * (r->next->v - r->v) + r->Kd * ((r->next->x - r->x - r->next->s) - rref_D)));//Method II
					}
					else
					{						
						S_D = Sensor_Range - (r->v * r->react);
						Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
						Amax_D = r->acc;
						//At_Update = (Amax_D * (1 - (r->v / Vmax_D)));//Method I
						At_Update = r->K * (Vmax_D - r->v);//Method II
					}
				}
				else
				{
					S_D = Sensor_Range - (r->v * r->react);
					Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
					Amax_D = r->acc;
					//At_Update = (Amax_D * (1 - (r->v / Vmax_D)));//Method I
					At_Update = r->K * (Vmax_D - r->v);//Method II
				}

				if (p->next == NULL)
				{
					Vt_Update = p->v;
					Vt_Update = Vt_Update + ( At_Update * DeltaT );
				
					Xt_Update = p->x;
					Xt_Update_Previous = p->x;
					Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
					if (Xt_Update_Previous > Xt_Update)
						Xt_Update = Xt_Update_Previous;
				
					p->a_update = At_Update;
					p->v = Vt_Update;
			 
						if (p->v <=0)
						{
							p->v = 0.001;
						};
			 
					p->x = Xt_Update;
					if (ramp_check == 0)
					{
						if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
						{
							p->x = ramp_information[counter_ramp].Ramp_EndPoint;
							p->v = 0.001;
						}
					}
					if (ramp_check == 2)
					{
						if (counter_path < Path[p->carID].size() - 1)
							if (Path[p->carID][counter_path + 1] == 1)
								if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
									for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
										Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
					}
					p->headway = HEADWAY_FIRST_CAR;
					p->Spacing = HEADWAY_FIRST_CAR;
					p->DV = DV_FIRST_CAR;
					//p->Yt = Yt_Update;
					//p->Yt_previous = Yt_previous_Update;

				 
					p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

					 
					//p->DXL2 = 0.0;//Possibility 9999.0
					//p->DVL2 = 0.0;//Possibility 999.0
				
					//if (p->previous == NULL)
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
					//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
					//}
				 

				 
					if (p->Mode != 0)
					{
						p->Mode = 0;
						p->Duration = 0.0;
						p->LCL = 0;

						p->PreviousLeaderID = -1;
					}
					else
					{
						if (p->changelaneinstance == 1)
						{
							p->changelaneinstance = 0;
							p->Duration = 0.0;
							p->LCL = 0;
						
							p->PreviousLeaderID = -1;
						}
						else
						{
							p->Duration = p->Duration + 0.1;
						
							if ( p->PreviousLeaderID != -1)
							{
								p->LCL = p->LCL + 1;
								p->PreviousLeaderID = -1;
							}		
						}
					}
					 
				}
				else
				{
					if ( p->next->x - p->x > Sensor_Range)
					{					
						if ( p->Mode == 1 )//Previous Episode was a CF Episode
						{
							p->Mode = 0;
							p->Duration = 0.0;
							p->LCL = 0;

							p->PreviousLeaderID = p->next->carID;
					
						}
						else//Previous Episode was a FF Episoe
						{
							if (p->changelaneinstance == 1) // If driver changed lanes
							{
								p->changelaneinstance = 0;
								p->Mode = 0;//not essential
								p->Duration = 0.0;
								p->LCL = 0;

								p->PreviousLeaderID = p->next->carID;
							}
							else //drivers did not change lanes: same episode
							{
								p->Mode = 0;//not essential
								p->Duration = p->Duration + 0.1;

								if (p->PreviousLeaderID != p->next->carID)
								{
									p->LCL = p->LCL + 1;
									p->PreviousLeaderID = p->next->carID;
								}
							}
						}
					}
					else //Current Episode is Car-Following Episode
					{
						if ( p->Mode == 0 )//Previous Episode was a FF Episode
						{
							p->Mode = 1;
							p->Duration = 0.0;
							p->LCL = 0;

							p->PreviousLeaderID = p->next->carID;
					
						}
						else//Previous Episode was a CF Episoe
						{
							if (p->changelaneinstance == 1) // If driver changed lanes
							{
								p->changelaneinstance = 0;
								p->Mode = 1;//not essential
								p->Duration = 0.0;
								p->LCL = 0;

								p->PreviousLeaderID = p->next->carID;
							}
							else //drivers did not change lanes: same episode
							{
								p->Mode = 1;//not essential
								p->Duration = p->Duration + 0.1;

								if (p->PreviousLeaderID != p->next->carID)
								{
									p->LCL = p->LCL + 1;
									p->PreviousLeaderID = p->next->carID;
								}
							}
						}
					}
				 
					
				
					//if ( At_Update > 4.0 )
					//	At_Update = 4.0;
					//if ( At_Update < -6.0 )  //Problem: Test: Put lower limit on acceleration (?)
					//	At_Update = -6.0;


					//Find the Resulting Velocity, Relative Velocity, Location, Relative Headway for Updating the Next Acceleration
					Vt_Update = p->v;
					Vt_Update = Vt_Update + ( At_Update * DeltaT );
				
					if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
						Vt_Update = 0.001;
				
					Xt_Update = p->x;
					Xt_Update_Previous = p->x;
					Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
					if (Xt_Update_Previous > Xt_Update)
						Xt_Update = Xt_Update_Previous;
					DeltaX_Update = p->next->x - Xt_Update;
					S_Update = DeltaX_Update - p->next->s;
					
					/*if (S_Update <= 0)
					{
						S_Update = 0.0;
						Vt_Update = 0.0;
						At_Update = 0.0;
						p->crash = 1;

					}*///Left for the Crash Detection Section				
				
					DeltaV_Update = Vt_Update - p->next->v;
				
					//Store in Wallsten Measures for Each Frame
					p->a_update = At_Update;
					p->v = Vt_Update;
					 
						if (p->v <=0)
						{
							p->v = 0.001;
						};
					 
					p->x = Xt_Update;
					if (ramp_check == 0)
					{
						if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
						{
							p->x = ramp_information[counter_ramp].Ramp_EndPoint;
							p->v = 0.001;
						}
					}
					if (ramp_check == 2)
					{
						if (counter_path < Path[p->carID].size() - 1)
							if (Path[p->carID][counter_path + 1] == 1)
								if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
									for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
										Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
					}
					p->DV = DeltaV_Update;
					p->headway = DeltaX_Update;
					p->Spacing = S_Update;//Possible Error: Problem Since Space Gaps are Sometimes Negative: Use Space Headway Instead
					//p->Yt = Yt_Update;
					//p->Yt_previous = Yt_previous_Update;

					 
					p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

					 
					//if (p->next->next == NULL)
					//{	
					//	p->DXL2 = 0.0;//Possibility 9999.9
					//	p->DVL2 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXL2 = p->next->next->x - p->x;//maybe needed here the length of the two vehicles ahead
					//	p->DVL2 = p->v - p->next->next->v;//maybe needed here zero
					//}

					//if (p->previous == NULL)
					//{
					//	p->DXF1 = 0.0;//Possibility 9999.0
					//	p->DVF1 = 0.0;//Possibility 999.0
					//}
					//else
					//{
					//	p->DXF1 = p->x - p->previous->x;//maybe needed here the vehicle length
					//	p->DVF1 = p->v - p->previous->v;//maybe needed here zero
					//}
		 
				}

			 
			}
		}
		else if (r->next == NULL && ramp_check == 0)		
		{
			if (Simulation_Model == 0)
			{
				 
			
				//Update Wallsten Parameters
				Gamma_D = r->Gamma;
				Wm_D = r->Wm;
				Wc_D = r->Wc;
				Tmax_D = r->Tmax;
				Alpha_D = r->Alpha;
				Beta_D = r->Beta;
				Tcorr_D = r->Tcorr;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r->So;
				Vo_D = r->Vd;//Transformed from an Earlier Model
				Amax_D = r->acc;//Transformed from an Earlier Model

				Wp_D = r->Wp;
				DeltaW_D = r->DeltaW;
				Sigma_D = r->Sigma;

				RT_D = r->Rt;
				 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r->a_update;
				Vt_Update = r->v;
				Xt_Update = r->x;
				DeltaX_Update = ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length - r->x;
				S_Update = ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length - r->x;
				DeltaV_Update = r->v - 0.001;
				Yt_Update = r->Yt;
				Yt_previous_Update = r->Yt_previous;

				//Test: Seff: Martin's Version: Keep for Updating Purposes: Keep Errors Defined
				//if ( ( S_Update - So_D ) > 0.0 )
				if ( ( S_Update - So_D ) > 0.1 )
					Seff = ( S_Update - So_D );
				else
					Seff = 0.1;
					//Seff = 0.0;

				//Tau
				if ( DeltaV_Update > ( Seff / Tmax_D ) )
					Tau =  ( Seff / DeltaV_Update );
				else
					Tau = Tmax_D;

				////Test
				//if (Vt_Update < 0.1)
				//	Vt_Update = 0.1;

				//Zprime
				Zprime = ( Tau / ( 2.0 * Alpha_D * Vt_Update ) );

				//Zdoubleprime
				Zdoubleprime = 0.0;
			

				//Zstar
				Zstar = ( - sqrt ( 2.0 * log ( ( Ao_D * Wc_D * Zprime ) / ( sqrt ( 2.0 * PI ) ) ) ) );

				//Astar : Initial Estimate
				Astar = ( ( 2.0 / Tau) * ( ( Seff / Tau) - DeltaV_Update + ( Alpha_D * Vt_Update * Zstar ) ) );
						
				regime_choice = mrand(20);//(rand () % 1000) / 1000;
				D_Total = NULL;
				NumberOfVehicles_Total = 0;

				if (r->next != NULL)
				{
					if (r->previous != NULL)
					{
						D_Total = r1_head;						
						if (D_Total != NULL)
						{
							while (D_Total != NULL)
							{						
								if (( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
									NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
								D_Total = D_Total->previous;
							}
						}
						else
							NumberOfVehicles_Total = 0;
						regime_calculation = -37.8195 + 1.7535 * r->v + 0.0459 * (r->next->x - r->x) + 0.3259 * (r->next->v - r->v) +0.0931 * (r->x - r->previous->x) + -1.0300 * (r->v - r->previous->v) + 0.5911 * (NumberOfVehicles_Total);
					}
					else
					{
						D_Total = r1_head;						
						if (D_Total != NULL)
						{
							while (D_Total != NULL)
							{						
								if (( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
									NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
								D_Total = D_Total->previous;
							}
						}
						else
							NumberOfVehicles_Total = 0;
						regime_calculation = -37.8195 + 1.7535 * r->v + 0.0459 * (r->next->x - r->x) + 0.3259 * (r->next->v - r->v) +0.0931 * (100) + -1.0300 * (0) + 0.5911 * (NumberOfVehicles_Total);
					}
				}
				else
				{
					if (r->previous != NULL)
					{
						D_Total = r1_head;						
						if (D_Total != NULL)
						{
							while (D_Total != NULL)
							{						
								if (( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
									NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
								D_Total = D_Total->previous;
							}
						}
						else
							NumberOfVehicles_Total = 0;
						regime_calculation = -37.8195 + 1.7535 * r->v + 0.0459 * (100) + 0.3259 * (0) +0.0931 * (r->x - r->previous->x) + -1.0300 * (r->v - r->previous->v) + 0.5911 * (NumberOfVehicles_Total);
					}
					else
					{
						D_Total = r1_head;						
						if (D_Total != NULL)
						{
							while (D_Total != NULL)
							{						
								if (( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
									NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
								D_Total = D_Total->previous;
							}
						}
						else
							NumberOfVehicles_Total = 0;
						regime_calculation = -37.8195 + 1.7535 * r->v + 0.0459 * (100) + 0.3259 * (0) +0.0931 * (100) + -1.0300 * (0) + 0.5911 * (NumberOfVehicles_Total);
					}
				}

				regime_calculation = regime_calculation / (1 + regime_calculation);
				//regime_choice = -100000.0;
				if (regime_choice < regime_calculation)
				{
					//Newton's Method loop
					for ( int NewtonCounter = 0; NewtonCounter < 4; NewtonCounter++ )
					{
						X = ( Astar / Ao_D );
					
						G = ( X / ( pow ( ( 1 + pow ( X , 2.0 ) ) , Gamma_D ) ) );//G = ( X / ( pow ( ( 1 + pow ( X , 2.0 ) ) , Sigma_D ) ) ); 
					
						Gprime = ( ( 1.0 / ( pow ( ( 1.0 + pow ( X , 2.0 )), Gamma_D ) ) ) - ( ( 2.0 * Gamma_D * pow ( X , 2.0 ) ) / ( pow ( ( 1 + pow ( X , 2.0 ) ) ,( Gamma_D + 1.0) ) ) ) ); 
						Gdoubleprime = ( ( ( - 6.0 * Gamma_D * X ) / ( pow ( ( 1.0 + pow ( X , 2.0 ) ) ,( Gamma_D + 1) ) ) ) + ( ( 4.0 * Gamma_D * ( Gamma_D + 1 ) * pow ( X , 3.0) ) / ( pow ( ( 1 + pow ( X , 2.0 ) ) ,( Gamma_D + 2.0) ) ) ) );
						//G = ( X * ( pow ( ( 1 + pow ( X , 2.0 ) ) , (Gamma_D / 2) ) ) );
						//Gprime = (pow((1 + pow(X, 2.0)),(Gamma_D / 2))) + (Gamma_D * pow(X , 2) * pow((1 + pow(X, 2.0)),((Gamma_D / 2) - 1)));
						//Gdoubleprime = (Gamma_D * X * pow((1 + pow(X, 2.0)),(Gamma_D / 2))) + (2 * Gamma_D * X * pow((1 + pow(X, 2.0)),((Gamma_D / 2) - 1))) + (2 * Gamma_D * ((Gamma_D / 2) - 1) * pow(X , 3) * pow((1 + pow(X, 2.0)),((Gamma_D / 2) - 2)));

						Uptprime = ( ( 1.0 / Ao_D ) *  ( ( ( Wm_D + ( ( DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gprime) ) + ( ( DeltaW_D * G ) / ( 2 * pow ( cosh ( X ) , 2 ) ) ) ) );
						Uptdoubleprime = ( ( 1.0 / ( pow (Ao_D , 2.0 ) ) ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gdoubleprime) ) + ( ( ( DeltaW_D ) / ( pow ( cosh ( X ) , 2.0 ) ) ) * ( Gprime - ( tanh ( X ) * G ) ) ) ) );

						Z = ( ( DeltaV_Update + ( ( 0.5 ) * Astar * Tau ) - ( Seff / Tau) ) / ( Alpha_D * Vt_Update ) );
						fn = ( 1.0 / sqrt ( 2 * PI) ) * exp (  - pow ( Z , 2.0) / 2.0 );
					
						F = Uptprime - Wc_D * fn * Zprime;
						Fprime = Uptdoubleprime - Wc_D * fn * ( Z * pow ( Zprime, 2.0) + Zdoubleprime );
					
						Astar = Astar - ( F / Fprime);
						//if (Astar >=100)
						//	Astar = 100;
						//if (Astar <= -100)
						//	Astar = -100;
					}

					//Compute U'' or F' for the last time for variance computation
				
					X = ( Astar / Ao_D );
					
					G = ( X / ( pow ( ( 1.0 + pow ( X , 2.0 ) ) , Gamma_D ) ) ); 
				
					Gprime = ( ( 1.0 / ( pow ( ( 1 + pow ( X , 2.0 )), Gamma_D ) ) ) - ( ( 2.0 * Gamma_D * pow ( X , 2.0 ) ) / ( pow ( ( 1 + pow ( X , 2.0 ) ) ,( Gamma_D + 1.0) ) ) ) ); 
					Gdoubleprime = ( ( ( - 6.0 * Gamma_D * X ) / ( pow ( ( 1.0 + pow ( X , 2.0 ) ) ,( Gamma_D + 1.0) ) ) ) + ( ( 4.0 * Gamma_D * ( Gamma_D + 1.0 ) * pow ( X , 3.0) ) / ( pow ( ( 1.0 + pow ( X , 2.0 ) ) ,( Gamma_D + 2.0) ) ) ) );

					Uptprime = ( ( 1.0 / Ao_D ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gprime) ) + ( ( DeltaW_D * G ) / ( 2 * pow ( cosh ( X ) , 2 ) ) ) ) );
					Uptdoubleprime = ( ( 1.0 / ( pow (Ao_D , 2.0 ) ) ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1.0 + tanh ( X ) ) ) / 2.0 ) ) * ( Gdoubleprime) ) + ( ( ( DeltaW_D ) / ( pow ( cosh ( X ) , 2.0 ) ) ) * ( Gprime - ( tanh ( X ) * G ) ) ) ) );

					Z = ( ( DeltaV_Update + ( ( 0.5 ) * Astar * Tau ) - ( Seff / Tau) ) / ( Alpha_D * Vt_Update ) );
					fn = ( 1.0 / sqrt ( 2 * PI ) ) * exp (  - pow ( Z , 2.0) / 2.0 );
				
					F = Uptprime - Wc_D * fn * Zprime;
					Fprime = Uptdoubleprime - Wc_D * fn * ( Z * pow ( Zprime, 2.0) + Zdoubleprime );

					Var = -1.0 / ( Beta_D * Fprime );

					//Random_Wiener = ( (rand()%1000) / 1000.0 );
					Random_Wiener = ( mrand( 38 ) ); //Return a Uniform Number Between Zero and One
					Yt_previous_Update = Yt_Update;

					//Yt_Update = Yt_Update * exp ( - DeltaT / Tau) + sqrt ( 24.0 * DeltaT / Tau ) * Random_Wiener; //Modified if the Wiener Process Takes the DT as the Reaction Time
					Yt_Update = Yt_Update * exp ( - RT_D / Tau) + sqrt ( 24.0 * RT_D / Tau ) * Random_Wiener;

					At_Update = Astar + Var * Yt_Update;

					 
					//Test
					//Incorporate Free-Speed Contrained Acceleration and Maximum Deceleration (Maximum Acceleration Just in Case
				}
				else
				{
					//Newton's Method loop
					for ( int NewtonCounter = 0; NewtonCounter < 4; NewtonCounter++ )
					{
						X = ( Astar / Ao_D );
					
						if (X >= 0)
							G = pow(pow(X, 2.0), Gamma_D);
						else
							G = -1.0 * pow(pow(X, 2.0), Gamma_D);

						if (X >= 0)
							//Gprime = 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1);
							Gprime = 2.0 * Gamma_D * X * pow(pow(X, 2.0), Gamma_D - 1);
						else
							//Gprime = -1.0 * 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1);
							Gprime = -1.0 * 2.0 * Gamma_D * X * pow(pow(X, 2.0), Gamma_D - 1);

						if (X >= 0)
							//Gdoubleprime = Gamma_D * (Gamma_D - 1) * pow(X, Gamma_D - 2);
							Gdoubleprime = 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1) + 2.0 * Gamma_D * (Gamma_D - 1) * X * pow(pow(X, 2.0), Gamma_D - 2);
						else
							//Gdoubleprime = -1 * Gamma_D * (Gamma_D - 1) * pow(X, Gamma_D - 2);
							Gdoubleprime = -1.0 * 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1) + 2.0 * Gamma_D * (Gamma_D - 1) * X * pow(pow(X, 2.0), Gamma_D - 2);

						Uptprime = ( ( 1.0 / Ao_D ) *  ( ( ( Wm_D + ( ( DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gprime) ) + ( ( DeltaW_D * G ) / ( 2 * pow ( cosh ( X ) , 2 ) ) ) ) );
						Uptdoubleprime = ( ( 1.0 / ( pow (Ao_D , 2.0 ) ) ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gdoubleprime) ) + ( ( ( DeltaW_D ) / ( pow ( cosh ( X ) , 2.0 ) ) ) * ( Gprime - ( tanh ( X ) * G ) ) ) ) );

						Z = ( ( DeltaV_Update + ( ( 0.5 ) * Astar * Tau ) - ( Seff / Tau) ) / ( Alpha_D * Vt_Update ) );
						fn = ( 1.0 / sqrt ( 2 * PI) ) * exp (  - pow ( Z , 2.0) / 2.0 );
					
						F = Uptprime - Wc_D * fn * Zprime;
						Fprime = Uptdoubleprime - Wc_D * fn * ( Z * pow ( Zprime, 2.0) + Zdoubleprime );
					
						Astar = Astar - ( F / Fprime);
						//if (Astar >=100)
						//	Astar = 100;
						//if (Astar <= -100)
						//	Astar = -100;
					}

					//Compute U'' or F' for the last time for variance computation
				
					X = ( Astar / Ao_D );
					
					if (X >= 0)
						G = pow(pow(X, 2.0), Gamma_D);
					else
						G = -1.0 * pow(pow(X, 2.0), Gamma_D);

					if (X >= 0)
						//Gprime = 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1);
						Gprime = 2.0 * Gamma_D * X * pow(pow(X, 2.0), Gamma_D - 1);
					else
						//Gprime = -1.0 * 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1);
						Gprime = -1.0 * 2.0 * Gamma_D * X * pow(pow(X, 2.0), Gamma_D - 1);

					if (X >= 0)
						//Gdoubleprime = Gamma_D * (Gamma_D - 1) * pow(X, Gamma_D - 2);
						Gdoubleprime = 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1) + 2.0 * Gamma_D * (Gamma_D - 1) * X * pow(pow(X, 2.0), Gamma_D - 2);
					else
						//Gdoubleprime = -1 * Gamma_D * (Gamma_D - 1) * pow(X, Gamma_D - 2);
						Gdoubleprime = -1.0 * 2.0 * Gamma_D * pow(pow(X, 2.0), Gamma_D - 1) + 2.0 * Gamma_D * (Gamma_D - 1) * X * pow(pow(X, 2.0), Gamma_D - 2);

					Uptprime = ( ( 1.0 / Ao_D ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1 + tanh ( X ) ) ) / 2.0 ) ) * ( Gprime) ) + ( ( DeltaW_D * G ) / ( 2 * pow ( cosh ( X ) , 2 ) ) ) ) );
					Uptdoubleprime = ( ( 1.0 / ( pow (Ao_D , 2.0 ) ) ) *  ( ( ( Wm_D + (  (  DeltaW_D * ( 1.0 + tanh ( X ) ) ) / 2.0 ) ) * ( Gdoubleprime) ) + ( ( ( DeltaW_D ) / ( pow ( cosh ( X ) , 2.0 ) ) ) * ( Gprime - ( tanh ( X ) * G ) ) ) ) );

					Z = ( ( DeltaV_Update + ( ( 0.5 ) * Astar * Tau ) - ( Seff / Tau) ) / ( Alpha_D * Vt_Update ) );
					fn = ( 1.0 / sqrt ( 2 * PI ) ) * exp (  - pow ( Z , 2.0) / 2.0 );
				
					F = Uptprime - Wc_D * fn * Zprime;
					Fprime = Uptdoubleprime - Wc_D * fn * ( Z * pow ( Zprime, 2.0) + Zdoubleprime );

					Var = -1.0 / ( Beta_D * Fprime );

					//Random_Wiener = ( (rand()%1000) / 1000.0 );
					Random_Wiener = ( mrand( 38 ) ); //Return a Uniform Number Between Zero and One
					Yt_previous_Update = Yt_Update;

					//Yt_Update = Yt_Update * exp ( - DeltaT / Tau) + sqrt ( 24.0 * DeltaT / Tau ) * Random_Wiener; //Modified if the Wiener Process Takes the DT as the Reaction Time
					Yt_Update = Yt_Update * exp ( - RT_D / Tau) + sqrt ( 24.0 * RT_D / Tau ) * Random_Wiener;

					At_Update = Astar + Var * Yt_Update;

					 
					//Test
					//Incorporate Free-Speed Contrained Acceleration and Maximum Deceleration (Maximum Acceleration Just in Case

				}

				if ( At_Update < r->decc)
					At_Update = r->decc;
				if ( At_Update > r->acc)
					At_Update = r->acc;
		
			
				if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) ) ) // Current Episode is Free-Flow Episode
					At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) );
				
				if (p->Mode != 0)
				{
					p->Mode = 0;
					p->Duration = 0.0;
					p->LCL = 0;

					p->PreviousLeaderID = -1;
				}
				else
				{
					if (p->changelaneinstance == 1)
					{
						p->changelaneinstance = 0;
						p->Duration = 0.0;
						p->LCL = 0;
					
						p->PreviousLeaderID = -1;
					}
					else
					{
						p->Duration = p->Duration + 0.1;
					
						if ( p->PreviousLeaderID != -1)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = -1;
						}		
					}
				}				
			
				//if ( At_Update > 4.0 )
				//	At_Update = 4.0;
				//if ( At_Update < -6.0 )  //Problem: Test: Put lower limit on acceleration (?)
				//	At_Update = -6.0;


				//Find the Resulting Velocity, Relative Velocity, Location, Relative Headway for Updating the Next Acceleration
				Vt_Update = p->v;
				Vt_Update = Vt_Update + ( At_Update * DeltaT );
			
				if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
					Vt_Update = 0.001;
			
				Xt_Update = p->x;
				Xt_Update_Previous = p->x;
				Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
				if (Xt_Update_Previous > Xt_Update)
					Xt_Update = Xt_Update_Previous;
				DeltaX_Update = ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length - Xt_Update;
				S_Update = DeltaX_Update;
				
				/*if (S_Update <= 0)
				{
					S_Update = 0.0;
					Vt_Update = 0.0;
					At_Update = 0.0;
					p->crash = 1;

				}*///Left for the Crash Detection Section				
			
				DeltaV_Update = Vt_Update - 0.001;
			
				//Store in Wallsten Measures for Each Frame
				p->a_update = At_Update;
				p->v = Vt_Update;
			 
					if (p->v <=0)
					{
						p->v = 0.001;
					};
				 
				p->x = Xt_Update;
				if (ramp_check == 0)
				{
					if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
					{
						p->x = ramp_information[counter_ramp].Ramp_EndPoint;
						p->v = 0.001;
					}
				}
				if (ramp_check == 2)
				{
					if (counter_path < Path[p->carID].size() - 1)
						if (Path[p->carID][counter_path + 1] == 1)
							if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
									Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
				}
				p->DV = DeltaV_Update;
				p->headway = DeltaX_Update;
				p->Spacing = S_Update;//Possible Error: Problem Since Space Gaps are Sometimes Negative: Use Space Headway Instead
				p->Yt = Yt_Update;
				p->Yt_previous = Yt_previous_Update;

 
				p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

				 
				//if (p->next->next == NULL)
				//{	
				//	p->DXL2 = 0.0;//Possibility 9999.9
				//	p->DVL2 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXL2 = p->next->next->x - p->x;//maybe needed here the length of the two vehicles ahead
				//	p->DVL2 = p->v - p->next->next->v;//maybe needed here zero
				//}

				//if (p->previous == NULL)
				//{
				//	p->DXF1 = 0.0;//Possibility 9999.0
				//	p->DVF1 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXF1 = p->x - p->previous->x;//maybe needed here the vehicle length
				//	p->DVF1 = p->v - p->previous->v;//maybe needed here zero
				//}
 
			}
			else if (Simulation_Model == 1)
			{
		 
			
				//Update Wallsten Parameters
				Deltan_D = r->Deltan;
				Tn_D = r->Tn;
				Son_D = r->Son;
				an_D = r->an;
				bn_D = r->bn;
				Von_D = r->Von;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r->So;
				Vo_D = r->Vd;//Transformed from an Earlier Model
				Amax_D = r->acc;//Transformed from an Earlier Model

				Wp_D = r->Wp;
				DeltaW_D = r->DeltaW;
				Sigma_D = r->Sigma;

				RT_D = r->Rt;
		 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r->a_update;
				Vt_Update = r->v;
				Xt_Update = r->x;
				DeltaX_Update = ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length - r->x;
				S_Update = ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length - r->x;
				DeltaV_Update = r->v - 0.001;
				Yt_Update = r->Yt;
				Yt_previous_Update = r->Yt_previous;

				At_Update = an_D * (1 - pow((Vt_Update/Von_D),Deltan_D) - ((Son_D + (Tn_D * Vt_Update) + ((Vt_Update * DeltaV_Update)/(2 * sqrt(an_D * bn_D))))/S_Update));
			
				if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) ) ) // Current Episode is Free-Flow Episode
					At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) );
				
				if (p->Mode != 0)
				{
					p->Mode = 0;
					p->Duration = 0.0;
					p->LCL = 0;

					p->PreviousLeaderID = -1;
				}
				else
				{
					if (p->changelaneinstance == 1)
					{
						p->changelaneinstance = 0;
						p->Duration = 0.0;
						p->LCL = 0;
					
						p->PreviousLeaderID = -1;
					}
					else
					{
						p->Duration = p->Duration + 0.1;
					
						if ( p->PreviousLeaderID != -1)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = -1;
						}		
					}
				}				
			
				//if ( At_Update > 4.0 )
				//	At_Update = 4.0;
				//if ( At_Update < -6.0 )  //Problem: Test: Put lower limit on acceleration (?)
				//	At_Update = -6.0;


				//Find the Resulting Velocity, Relative Velocity, Location, Relative Headway for Updating the Next Acceleration
				Vt_Update = p->v;
				Vt_Update = Vt_Update + ( At_Update * DeltaT );
			
				if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
					Vt_Update = 0.001;
			
				Xt_Update = p->x;
				Xt_Update_Previous = p->x;
				Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
				if (Xt_Update_Previous > Xt_Update)
					Xt_Update = Xt_Update_Previous;
				DeltaX_Update = ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length - Xt_Update;
				S_Update = DeltaX_Update;
				
				/*if (S_Update <= 0)
				{
					S_Update = 0.0;
					Vt_Update = 0.0;
					At_Update = 0.0;
					p->crash = 1;

				}*///Left for the Crash Detection Section				
			
				DeltaV_Update = Vt_Update - 0.001;
			
				//Store in Wallsten Measures for Each Frame
				p->a_update = At_Update;
				p->v = Vt_Update;
	 
					if (p->v <=0)
					{
						p->v = 0.001;
					};
 
				p->x = Xt_Update;
				if (ramp_check == 0)
				{
					if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
					{
						p->x = ramp_information[counter_ramp].Ramp_EndPoint;
						p->v = 0.001;
					}
				}
				if (ramp_check == 2)
				{
					if (counter_path < Path[p->carID].size() - 1)
						if (Path[p->carID][counter_path + 1] == 1)
							if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
									Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
				}
				p->DV = DeltaV_Update;
				p->headway = DeltaX_Update;
				p->Spacing = S_Update;//Possible Error: Problem Since Space Gaps are Sometimes Negative: Use Space Headway Instead
				p->Yt = Yt_Update;
				p->Yt_previous = Yt_previous_Update;

			 
				p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

 
				//if (p->next->next == NULL)
				//{	
				//	p->DXL2 = 0.0;//Possibility 9999.9
				//	p->DVL2 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXL2 = p->next->next->x - p->x;//maybe needed here the length of the two vehicles ahead
				//	p->DVL2 = p->v - p->next->next->v;//maybe needed here zero
				//}

				//if (p->previous == NULL)
				//{
				//	p->DXF1 = 0.0;//Possibility 9999.0
				//	p->DVF1 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXF1 = p->x - p->previous->x;//maybe needed here the vehicle length
				//	p->DVF1 = p->v - p->previous->v;//maybe needed here zero
				//}
 
			}
			else if (Simulation_Model == 2)
			{
				if (r->next != NULL)
				{
					spacing_temp = 0;
					spacing_temp = r->next->x - r->x;

					if (spacing_temp < Sensor_Range)
					{
						S_D = max(0.01, min((r->next->x - r->x - r->next->s) - (r->v * r->react) + ((r->next->v * r->next->v) / (2 * fabs(r->next->decc))), Sensor_Range - (r->v * r->react)));
						Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
						Amax_D = r->acc;
						rref_D = min (min(((r->v * r->v) / 2) * ((1 / fabs(r->next->decc)) - (1 / fabs(r->decc))), r->react * r->v), 2.0);
						if (((r->next->x - r->x - r->next->s) < 15) && (r->next->v < r->v))
							At_Update = max(r->decc,  ((r->next->v * r->next->v) - (r->v * r->v)) / ((r->next->x - r->x - r->next->s)));
						else
							At_Update = max(r->decc, min(r->K * (Vmax_D - r->v), r->Ka * r->next->a_update + r->Kv * (r->next->v - r->v) + r->Kd * ((r->next->x - r->x - r->next->s) - rref_D)));//Method II
					}
					else
					{						
						S_D = Sensor_Range - (r->v * r->react);
						Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
						Amax_D = r->acc;
						//At_Update = (Amax_D * (1 - (r->v / Vmax_D)));//Method I
						At_Update = r->K * (Vmax_D - r->v);//Method II
					}
				}
				else
				{
					S_D = Sensor_Range - (r->v * r->react);
					Vmax_D = min(sqrt(2 * fabs(r->decc) * S_D), r->Vd);
					Amax_D = r->acc;
					//At_Update = (Amax_D * (1 - (r->v / Vmax_D)));//Method I
					At_Update = r->K * (Vmax_D - r->v);//Method II
				}
			 
				
				if (p->Mode != 0)
				{
					p->Mode = 0;
					p->Duration = 0.0;
					p->LCL = 0;

					p->PreviousLeaderID = -1;
				}
				else
				{
					if (p->changelaneinstance == 1)
					{
						p->changelaneinstance = 0;
						p->Duration = 0.0;
						p->LCL = 0;
					
						p->PreviousLeaderID = -1;
					}
					else
					{
						p->Duration = p->Duration + 0.1;
					
						if ( p->PreviousLeaderID != -1)
						{
							p->LCL = p->LCL + 1;
							p->PreviousLeaderID = -1;
						}		
					}
				}				
			
				
				//Find the Resulting Velocity, Relative Velocity, Location, Relative Headway for Updating the Next Acceleration
				Vt_Update = p->v;
				Vt_Update = Vt_Update + ( At_Update * DeltaT );
			
				if ( Vt_Update <= 0.0 )//Problem: Test: To Avoid Negative Velocities (?)
					Vt_Update = 0.001;
			
				Xt_Update = p->x;
				Xt_Update_Previous = p->x;
				Xt_Update = Xt_Update + ( Vt_Update * DeltaT ) + 0.5 * ( At_Update * DeltaT * DeltaT );
				if (Xt_Update_Previous > Xt_Update)
					Xt_Update = Xt_Update_Previous;
				DeltaX_Update = ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length - Xt_Update;
				S_Update = DeltaX_Update;
				
				/*if (S_Update <= 0)
				{
					S_Update = 0.0;
					Vt_Update = 0.0;
					At_Update = 0.0;
					p->crash = 1;

				}*///Left for the Crash Detection Section				
			
				DeltaV_Update = Vt_Update - 0.001;
			
				//Store in Wallsten Measures for Each Frame
				p->a_update = At_Update;
				p->v = Vt_Update;
			 
					if (p->v <=0)
					{
						p->v = 0.001;
					};
			 
				p->x = Xt_Update;
				if (ramp_check == 0)
				{
					if (p->x > ramp_information[counter_ramp].Ramp_EndPoint)
					{
						p->x = ramp_information[counter_ramp].Ramp_EndPoint;
						p->v = 0.001;
					}
				}
				if (ramp_check == 2)
				{
					if (counter_path < Path[p->carID].size() - 1)
						if (Path[p->carID][counter_path + 1] == 1)
							if (p->x > ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								for (short ss = counter_path + 1; ss <= Path[p->carID].size() - 1; ss++)
									Path[p->carID][ss] = ramp_information[counter_ramp].Ramp_Number;
				}
				p->DV = DeltaV_Update;
				p->headway = DeltaX_Update;
				p->Spacing = S_Update;//Possible Error: Problem Since Space Gaps are Sometimes Negative: Use Space Headway Instead
				//p->Yt = Yt_Update;
				//p->Yt_previous = Yt_previous_Update;

				 
				p->DistanceToRamp = fabs ( ( RampLocation + MergingLength ) - p->x);

				 
				//if (p->next->next == NULL)
				//{	
				//	p->DXL2 = 0.0;//Possibility 9999.9
				//	p->DVL2 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXL2 = p->next->next->x - p->x;//maybe needed here the length of the two vehicles ahead
				//	p->DVL2 = p->v - p->next->next->v;//maybe needed here zero
				//}

				//if (p->previous == NULL)
				//{
				//	p->DXF1 = 0.0;//Possibility 9999.0
				//	p->DVF1 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXF1 = p->x - p->previous->x;//maybe needed here the vehicle length
				//	p->DVF1 = p->v - p->previous->v;//maybe needed here zero
				//}
 
			}
		}
	

		if (p->previous == NULL)
			s = p;
		p = p->previous;
	}


	p = r1_head;
	while (p != NULL)
	{
		/*if (p->carID == 1)
		{
			cout<<TIME<<"	"<<p->lane<<"	"<<p->v<<endl;
			getch();
		};*/
		Vehicle_History[p->carID].pop_front();
		TempP_history = (car *) malloc(sizeof(car));
		TempP_history = CopyStream (p);
		Vehicle_History[p->carID].push_back(TempP_history );
		p = p->previous;
	};

	return(s);
}

//--------------------------------------End of Car-Following---------------------------------------





//-----------------------------------------Crash Detection-----------------------------------------
struct car* CrashDetection (car *r1)
{
	car * p = NULL, * s = NULL;
	int riskIsZero = 0;

	p = r1;
	Crash crash;
	if (crashes.size()==0)
	{
		//crash.CrashIndex = 0;
	}
	crash.CrashLane = 0;
	crash.CrashPostion = 0.0;
	crash.CarID = -1;
	crash.CrashTime = -1;
	
	if (p!=NULL)
	{
		while (p!=NULL)
		{
			/*
			if (p->carID == 87)//61)//&& p->next == NULL)// && TIME > 885)// || p->carID == 7 )//&& p->x > 4900)//26)
			{
				cout<<"0"<<"	"<<TIME<<"	"<<p->carID<<"	"<<p->v<<"	"<<p->x<<"	"<<p->crash<<"	"<<p->lane<<"	"<<p->Duration<<"	"<<p->l<<"	"<<p->Vd<<endl;//"	"<<p->crash<<"	"<<p->next->v<<"	"<<p->next->carID<<"	"<<p->next->crash<<endl;
				if (p->next != NULL)
				{
					cout<<"1"<<"	"<<TIME<<"	"<<p->next->carID<<"	"<<p->next->v<<"	"<<p->next->x<<"	"<<p->next->crash<<"	"<<p->next->lane<<"	"<<p->next->Duration<<"	"<<p->next->l<<endl;//"	"<<p->crash<<"	"<<p->next->v<<"	"<<p->next->carID<<"	"<<p->next->crash<<endl;
					if (p->next->next != NULL)
						cout<<"2"<<"	"<<TIME<<"	"<<p->next->next->carID<<"	"<<p->next->next->v<<"	"<<p->next->next->x<<"	"<<p->next->next->crash<<"	"<<p->next->next->lane<<"	"<<p->next->next->Duration<<"	"<<p->next->next->l<<endl;//"	"<<p->crash<<"	"<<p->next->v<<"	"<<p->next->carID<<"	"<<p->next->crash<<endl;
				}
				if (p->previous != NULL)
					cout<<"-1"<<"	"<<TIME<<"	"<<p->previous->carID<<"	"<<p->previous->v<<"	"<<p->previous->x<<"	"<<p->previous->crash<<"	"<<p->previous->lane<<"	"<<p->previous->Duration<<"	"<<p->previous->l<<endl;//"	"<<p->crash<<"	"<<p->previous->v<<"	"<<p->previous->carID<<"	"<<p->previous->crash<<endl;
				
				cout<<endl;
				getch();
			}*/


			if (p->next != NULL)
			{
				 
				p->Spacing = p->next->x - p->next->s - p->x;
				 

				if (riskIsZero == 0)
				{
					if (p->Spacing < 0 && (p->crash != 1 || p->next->crash !=1)) //accident with car in front!
					{
						p->Spacing = 0.0; //prevent negative headway
						p->headway = p->next->s;
						p->DV = 0.0;

						p->next->x = p->x + p->next->s; //maintain list integrity and car lengths

					 
						//if (p->next->next == NULL)
						//{	
						//	p->DXL2 = 0.0;//Possibility 9999.9
						//	p->DVL2 = 0.0;//Possibility 999.0
						//}
						//else
						//{
						//	p->DXL2 = p->next->next->x - p->x;//maybe needed here the length of the two vehicles ahead
						//	p->DVL2 = p->v - p->next->next->v;//maybe needed here zero
						//}

						//if (p->previous == NULL)
						//{
						//	p->DXF1 = 0.0;//Possibility 9999.0
						//	p->DVF1 = 0.0;//Possibility 999.0
						//}
						//else
						//{
						//	p->DXF1 = p->x - p->previous->x;//maybe needed here the vehicle length
						//	p->DVF1 = p->v - p->previous->v;//maybe needed here zero
						//}
						 

						if (p->crash ==0)
						{
							if (p->next->crash ==0)
							{
 								p->crash = 1;
								//crash.CrashIndex = crash.CrashIndex + 1;
								crash.CrashLane = p->lane;
								crash.CrashPostion = p->x;
								crash.CarID = p->carID;
								crash.CrashTime = TIME;
								crashes.push_back(crash);
							}
							else
							{
								p->crash = 1;
								for (int j=0;j<crashes.size();j++)
								{
									if (crashes[j].CarID == p->next->carID)
									{
										//crash.CrashIndex = crashes[j].CrashIndex;
										break;
									}
								}

								crash.CrashLane = p->lane;
								crash.CrashPostion = p->x;
								crash.CarID = p->carID;
								crash.CrashTime = TIME;
								crashes.push_back(crash);
							}
						}

						if (p->next->crash==0)
						{
							//crash.CrashIndex = crash.CrashIndex;
							p->next->crash = 1;
							crash.CrashLane = p->next->lane;
							crash.CrashPostion = p->next->x;
							crash.CarID = p->next->carID;
							crash.CrashTime = TIME;
							crashes.push_back(crash);
						}
					}
				}
			}
			else //front of lane
			{
				p->Spacing = HEADWAY_FIRST_CAR;
				p->headway = HEADWAY_FIRST_CAR;
				p->DV = DV_FIRST_CAR;

				 
				//p->DXL2 = 0.0;//Possibility 9999.0
				//p->DVL2 = 0.0;//Possibility 999.0
				
				//if (TIME == 2057)
				//	getch();
				//if (p->previous == NULL)
				//{
				//	p->DXF1 = 0.0;//Possibility 9999.0
				//	p->DVF1 = 0.0;//Possibility 999.0
				//}
				//else
				//{
				//	p->DXF1 = p->x - p->previous->x;//issue of being the vehicle length if crash
				//	p->DVF1 = p->v - p->previous->v;//issue of being zero if crash
				//}
				 
			}

			if (p->next == NULL)
				s = p;
			p = p->next;

		}
	}
		
	return(s);
};
//-------------------------------------End of Crash Detection--------------------------------------
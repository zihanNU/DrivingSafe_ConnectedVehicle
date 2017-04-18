/*

		   \  L20i \		  /	 L20i /
			\		\		 /		 /
			 \		 --------		/
			  \			(6)		   /
--------------- --  --  --  -- -- ---------------------
L0-lane 1		-->q  r2	(5)				(0)
-------------------------------------------------------
L1-lane 2		-->p  r1					(1)
-------------------------------------------------------
L2-lane 3		-->s  r3					(1)
-------------------------------------------------------
L3-lane 4					(3)				(2)
--------------- --  --  --  -- -- ---------------------
			  /		   (4)		  \
			 /      ---------	   \
			/      /		 \	    \
		   / L10i /			  \ L10i \
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
#include "Car_Following.h"


//-----------------------------------------Lane Changing-----------------------------------------
struct lane_changing_data
{
	car * Ltmp1;
	car * Ltmp1_head;
	car * Ltmp2;
	car * Ltmp2_head;
	car * Ltmp3;
	car * Ltmp3_head;	
};

struct lane_changing_data_RML
{
	car * LtmpRML1;
	car * LtmpRML2;
	car * LtmpRML3;
	car * LtmpRML10;
	car * LtmpRML11;
	car * LtmpRML12;
	car * LtmpRML13;
	car * LtmpRML14;
	car * LtmpRML15;
	car * LtmpRML16;
	car * LtmpRML17;
	car * LtmpRML18;
	

	car * LtmpRML1_head;
	car * LtmpRML2_head;
	car * LtmpRML3_head;
	car * LtmpRML10_head;
	car * LtmpRML11_head;
	car * LtmpRML12_head;
	car * LtmpRML13_head;
	car * LtmpRML14_head;
	car * LtmpRML15_head;
	car * LtmpRML16_head;
	car * LtmpRML17_head;
	car * LtmpRML18_head;
	
};

struct safety_criteria
{
	short safety;
	float acc;	
};

short Lane_Decision (car *r1, double hazard, short targetrampside)
{
	short i = -10;

	if (r1->lane == 0)
	{
		if (hazard == 10 && targetrampside == 1)
			i = 0;
		if (hazard == 10 && targetrampside == 0)
			i = 5;
		if (hazard != 10)
			i = 0;
	}

	if (r1->lane == 1)
	{
		if (hazard == 10 && targetrampside == 1)
			i = 0;
		if (hazard == 10 && targetrampside == 0)
			i = 2;
		if (hazard != 10)
			i = 1;
	}

	if (r1->lane == 2)
	{
		if (hazard == 10 && targetrampside == 1)
			i = 0;
		if (hazard == 10 && targetrampside == 0)
			i = 2;
		if (hazard != 10)
			i = 1;
	}
	if (r1->lane == 3)
	{		
		if (hazard == 10 && targetrampside == 1)
			i = 3;
		if (hazard == 10 && targetrampside == 0)
			i = 2;
		if (hazard != 10)
			i = 2;

	}
	if (r1->lane >= 10 && r1->lane < 200)
		i = 4;
	if (r1->lane > 199)
		i = 6;

	return (i);
};

car * ramp_output (short ramp_id, car * L10, car * L11, car * L12, car * L13, car * L14, car * L15, car * L16, car * L17, car * L18)
{
	if (ramp_id == 0)
		return (L10);
	else if (ramp_id == 1)
		return (L11);
	else if (ramp_id == 2)
		return (L12);
	else if (ramp_id == 3)
		return (L13);
	else if (ramp_id == 4)
		return (L14);
	else if (ramp_id == 5)
		return (L15);
	else if (ramp_id == 6)
		return (L16);
	else if (ramp_id == 7)
		return (L17);
	else if (ramp_id == 8)
		return (L18);	
	else 
		return (NULL);
};


struct safety_criteria * Safety_Criteria_Ahead (car * r1, car * r2, car * r1_head, car * r2_head)
{
	safety_criteria * Safety_temp;	
	Safety_temp = (safety_criteria *) malloc(sizeof(Safety_temp));

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
	double DeltaV_Update;
	double DeltaX_Update;
	double S_Update;
	double Yt_Update;
	double Yt_previous_Update;

	double DeltaT = 0.1;
	double Random_Wiener;
	
	short SafetyCriterionAhead = 0;
	short SafetyCriterionBehind = 0;
	double spacing_temp;
	double rref_D;
	double S_D;
	double Vmax_D;
	
	if (r2 != NULL)
	{
		if (r2_head->x <= r1->x)
		{
			Safety_temp->safety = 1;
			Safety_temp->acc = 10;
		}
		else
		{
			if (r1->Connectivity_Status == 0)
			{
				Gamma_D = r1->Gamma;
				Wm_D = r1->Wm;
				Wc_D = r1->Wc;
				Tmax_D = r1->Tmax;
				Alpha_D = r1->Alpha;
				Beta_D = r1->Beta;
				Tcorr_D = r1->Tcorr;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r1->So;
				Vo_D = r1->Vd;//Transformed from an Earlier Model
				Amax_D = r1->acc;//Transformed from an Earlier Model

				Wp_D = r1->Wp;
				DeltaW_D = r1->DeltaW;
				Sigma_D = r1->Sigma;

				RT_D = r1->Rt;
				 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r1->a_update;
				Vt_Update = r1->v;
				Xt_Update = r1->x;
				DeltaX_Update = r2->x - r1->x;
				S_Update = r2->x - r2->s - r1->x;
				DeltaV_Update = r1->v - r2->v;
				Yt_Update = r1->Yt;
				Yt_previous_Update = r1->Yt_previous;
				
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


				//Var (Variance)

				Var = -1.0 / ( Beta_D * Fprime );

				//Random_Wiener = ( (rand()%1000) / 1000.0 );
				Random_Wiener = ( mrand( 38 ) ); //Return a Uniform Number Between Zero and One
				Yt_previous_Update = Yt_Update;

				//Yt_Update = Yt_Update * exp ( - DeltaT / Tau) + sqrt ( 24.0 * DeltaT / Tau ) * Random_Wiener; //Modified if the Wiener Process Takes the DT as the Reaction Time
				Yt_Update = Yt_Update * exp ( - RT_D / Tau) + sqrt ( 24.0 * RT_D / Tau ) * Random_Wiener;

				At_Update = Astar + Var * Yt_Update;

				//Incorporate Free-Speed Contrained Acceleration and Maximum Deceleration (Maximum Acceleration Just in Case
				if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) ) ) // Current Episode is Free-Flow Episode
				{
					At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) );
				}
			}
			else if (r1->Connectivity_Status == 1)
			{
				//Update Wallsten Parameters
				Deltan_D = r1->Deltan;
				Tn_D = r1->Tn;
				Son_D = r1->Son;
				an_D = r1->an;
				bn_D = r1->bn;
				Von_D = r1->Von;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r1->So;
				Vo_D = r1->Vd;//Transformed from an Earlier Model
				Amax_D = r1->acc;//Transformed from an Earlier Model

				Wp_D = r1->Wp;
				DeltaW_D = r1->DeltaW;
				Sigma_D = r1->Sigma;

				RT_D = r1->Rt;
				 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r1->a_update;
				Vt_Update = r1->v;
				Xt_Update = r1->x;
				DeltaX_Update = r2->x - r1->x;
				S_Update = r2->x - r2->s - r1->x;
				DeltaV_Update = r1->v - r2->v;
				Yt_Update = r1->Yt;
				Yt_previous_Update = r1->Yt_previous;

				At_Update = an_D * (1 - pow((Vt_Update/Von_D),Deltan_D) - ((Son_D + (Tn_D * Vt_Update) + ((Vt_Update * DeltaV_Update)/(2 * sqrt(an_D * bn_D))))/S_Update));
				
				if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) ) ) // Current Episode is Free-Flow Episode
						At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) );
			}
			else if (r1->Connectivity_Status == 2)
			{
				spacing_temp = 0;
				spacing_temp = r2->x - r1->x;

				if (spacing_temp < Sensor_Range)
				{
					S_D = max(0.01, min((r2->x - r1->x - r2->s) - (r1->v * r1->react) + ((r2->v * r2->v) / (2 * fabs(r2->decc))), Sensor_Range - (r1->v * r1->react)));
					Vmax_D = min(sqrt(2 * fabs(r1->decc) * S_D), r1->Vd);
					Amax_D = r1->acc;
					rref_D = min (min(((r1->v * r1->v) / 2) * ((1 / fabs(r2->decc)) - (1 / fabs(r1->decc))), r1->react * r1->v), 2.0);						
					if (((r2->x - r1->x - r2->s) < 15) && (r2->v < r1->v))
						At_Update = max(r1->decc,  ((r2->v * r2->v) - (r1->v * r1->v)) / ((r2->x - r1->x - r2->s)));
					else
						At_Update = max(r1->decc, min(r1->K * (Vmax_D - r1->v), r1->Ka * r2->a_update + r1->Kv * (r2->v - r1->v) + r1->Kd * ((r2->x - r1->x - r2->s) - rref_D)));//Method II
				}
				else
				{						
					S_D = Sensor_Range - (r1->v * r1->react);
					Vmax_D = min(sqrt(2 * fabs(r1->decc) * S_D), r1->Vd);
					Amax_D = r1->acc;
					//At_Update = (Amax_D * (1 - (r->v / Vmax_D)));//Method I
					At_Update = r1->K * (Vmax_D - r1->v);//Method II
				}
				
				Vt_Update = r1->v;
				Vo_D = r1->Vd;
				Von_D = r1->Von;

				if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) ) ) // Current Episode is Free-Flow Episode
						At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) );
			}

			if ( At_Update > r1->decc )
			{
				Safety_temp->safety = 1;
				Safety_temp->acc = At_Update;
			}
			else
			{
				Safety_temp->safety = 0;
				Safety_temp->acc = -10;
			}
			
			//End of Safety with Respect to the Leader
		}
	}
	else
	{
		Safety_temp->safety = 1;
		Safety_temp->acc = 10;
	}
	
	return (Safety_temp);
};



struct safety_criteria * Safety_Criteria_Behind (car * r1, car * r2, car * r1_head, car * r2_head)
{
	safety_criteria * Safety_temp;	
	Safety_temp = (safety_criteria *) malloc(sizeof(Safety_temp));

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
	double DeltaV_Update;
	double DeltaX_Update;
	double S_Update;
	double Yt_Update;
	double Yt_previous_Update;

	double DeltaT = 0.1;
	double Random_Wiener;
	
	short SafetyCriterionAhead = 0;
	short SafetyCriterionBehind = 0;

	double spacing_temp;
	double rref_D;
	double S_D;
	double Vmax_D;


	
	if (r2 != NULL)
	{
		if (r2->previous == NULL)
		{
			Safety_temp->safety = 1;
			Safety_temp->acc = 10;
		}
		else
		{
			if (r2->previous->Connectivity_Status == 0)
			{
				//Update Wallsten Parameters
				Gamma_D = r2->previous->Gamma;
				Wm_D = r2->previous->Wm;
				Wc_D = r2->previous->Wc;
				Tmax_D = r2->previous->Tmax;
				Alpha_D = r2->previous->Alpha;
				Beta_D = r2->previous->Beta;
				Tcorr_D = r2->previous->Tcorr;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r2->previous->So;
				Vo_D = r2->previous->Vd;//Transformed from an Earlier Model
				Amax_D = r2->previous->acc;//Transformed from an Earlier Model

				Wp_D = r2->previous->Wp;
				DeltaW_D = r2->previous->DeltaW;
				Sigma_D = r2->previous->Sigma;

				if (r2->previous != NULL)
				{
					RT_D = r2->previous->Rt;
				}
				else
				{
					RT_D = 0.001;
				}
				 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r2->previous->a_update;
				Vt_Update = r2->previous->v;
				Xt_Update = r2->previous->x;
				DeltaX_Update = r1->x - r2->previous->x;
				S_Update = r1->x - r1->s - r2->previous->x;
				DeltaV_Update = r2->previous->v - r1->v;
				Yt_Update = r2->previous->Yt;
				Yt_previous_Update = r2->previous->Yt_previous;
				
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


				//Var (Variance)

				Var = -1.0 / ( Beta_D * Fprime );

				//Random_Wiener = ( (rand()%1000) / 1000.0 );
				Random_Wiener = ( mrand( 38 ) ); //Return a Uniform Number Between Zero and One
				Yt_previous_Update = Yt_Update;

				//Yt_Update = Yt_Update * exp ( - DeltaT / Tau) + sqrt ( 24.0 * DeltaT / Tau ) * Random_Wiener; //Modified if the Wiener Process Takes the DT as the Reaction Time
				Yt_Update = Yt_Update * exp ( - RT_D / Tau) + sqrt ( 24.0 * RT_D / Tau ) * Random_Wiener;

				At_Update = Astar + Var * Yt_Update;

				//Incorporate Free-Speed Contrained Acceleration and Maximum Deceleration (Maximum Acceleration Just in Case
				if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) ) ) // Current Episode is Free-Flow Episode
				{
					At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Vo_D ) ) );
				}
			}
			else if (r2->previous->Connectivity_Status == 1)
			{
				//Update Wallsten Parameters
				Deltan_D = r2->previous->Deltan;
				Tn_D = r2->previous->Tn;
				Son_D = r2->previous->Son;
				an_D = r2->previous->an;
				bn_D = r2->previous->bn;
				Von_D = r2->previous->Von;

				Ao_D = 1.0;//Just for Normalization Reasons: Incorporated for Consistency with the Model
				So_D = r2->previous->So;
				Vo_D = r2->previous->Vd;//Transformed from an Earlier Model
				Amax_D = r2->previous->acc;//Transformed from an Earlier Model

				Wp_D = r2->previous->Wp;
				DeltaW_D = r2->previous->DeltaW;
				Sigma_D = r2->previous->Sigma;

				RT_D = r2->previous->Rt;
				 

				//Update Latest Microscopic Measures of Effectiveness
				At_Update = r2->previous->a_update;
				Vt_Update = r2->previous->v;
				Xt_Update = r2->previous->x;
				DeltaX_Update = r2->x - r2->previous->x;
				S_Update = r1->x - r1->s - r2->previous->x;
				DeltaV_Update = r2->previous->v - r1->v;
				Yt_Update = r2->previous->Yt;
				Yt_previous_Update = r2->previous->Yt_previous;

				At_Update = an_D * (1 - pow((Vt_Update/Von_D),Deltan_D) - ((Son_D + (Tn_D * Vt_Update) + ((Vt_Update * DeltaV_Update)/(2 * sqrt(an_D * bn_D))))/S_Update));
				
				if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) ) ) // Current Episode is Free-Flow Episode
						At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) );
			}
			else if (r2->previous->Connectivity_Status == 2)
			{
				spacing_temp = 0;
				spacing_temp = r1->x - r2->previous->x;

				if (spacing_temp < Sensor_Range)
				{
					S_D = max(0.01, min((r1->x - r2->previous->x - r1->s) - (r2->previous->v * r2->previous->react) + ((r1->v * r1->v) / (2 * fabs(r1->decc))), Sensor_Range - (r2->previous->v * r2->previous->react)));
					Vmax_D = min(sqrt(2 * fabs(r2->previous->decc) * S_D), r2->previous->Vd);
					Amax_D = r2->previous->acc;
					rref_D = min (min(((r2->previous->v * r2->previous->v) / 2) * ((1 / fabs(r1->decc)) - (1 / fabs(r2->previous->decc))), r2->previous->react * r2->previous->v), 2.0);						
					if (((r1->x - r2->previous->x - r1->s) < 15) && (r1->v < r2->previous->v))
						At_Update = max(r2->previous->decc,  ((r1->v * r1->v) - (r2->previous->v * r2->previous->v)) / ((r1->x - r2->previous->x - r1->s)));
					else
						At_Update = max(r2->previous->decc, min(r2->previous->K * (Vmax_D - r2->previous->v), r2->previous->Ka * r1->a_update + r2->previous->Kv * (r1->v - r2->previous->v) + r2->previous->Kd * ((r1->x - r2->previous->x - r1->s) - rref_D)));//Method II
				}
				else
				{						
					S_D = Sensor_Range - (r2->previous->v * r2->previous->react);
					Vmax_D = min(sqrt(2 * fabs(r2->previous->decc) * S_D), r2->previous->Vd);
					Amax_D = r2->previous->acc;
					//At_Update = (Amax_D * (1 - (r->v / Vmax_D)));//Method I
					At_Update = r2->previous->K * (Vmax_D - r2->previous->v);//Method II
				}

				Vt_Update = r2->previous->v;
				Vo_D = r2->previous->Vd;
				Von_D = r2->previous->Von;
				
				if ( At_Update >  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) ) ) // Current Episode is Free-Flow Episode
						At_Update =  ( Amax_D * ( 1 - ( Vt_Update / Von_D ) ) );
			}

			if ( At_Update > r2->previous->decc )
			{
				Safety_temp->safety = 1;
				Safety_temp->acc = 10;
			}
			else
			{
				Safety_temp->safety = 0;
				Safety_temp->acc = -10;
			}
		}
	}
	else
	{
		Safety_temp->safety = 1;
		Safety_temp->acc = 10;
	}

	return (Safety_temp);
};

struct lane_changing_data * Lane_Changing (car * r1 , car * r2, car * r3, double DesiredSpeedLimit_temp, double VSLStartLocation_temp, double VSLEndLocation_temp)
{
	//=============================================Variable Definition================================================
	lane_changing_data * Lane_Changing_temp;
	Lane_Changing_temp = (lane_changing_data *) malloc(sizeof(Lane_Changing_temp));
		
	car * p, * r1_head;	
	car * q, * r2_head;
	car * s, * r3_head;
	car * r;
	car * origin = NULL, * origin_head = NULL, *destination = NULL, * destination_head = NULL, * destination_tail = NULL, * thirdlane = NULL, * thirdlane_head = NULL;
	car * D1_Side, *D2_Side, *D_Total;
	
	float DXL1 = 0;
	float DVL1 = 0;
	float DXL2 = 0;
	float DVL2 = 0;
	float DXF1 = 0;
	float DVF1 = 0;
	float DXL1R = 0;
	float DVL1R = 0;
	float DXF1R = 0;
	float DVF1R = 0;
	float DXL1L = 0;
	float DVL1L = 0;
	float DXF1L = 0;
	float DVF1L = 0;
	float KR = 0;
	float KL = 0;
	float K = 0;

	float a;
	short b;
	short mandatorylanechanging = 0;
	//PT Model Parameters: Update Purposes Taken from "car" Structs
	double Gamma_D;
	double Wm_D;
	double Wc_D;
	double Tmax_D;
	double Alpha_D;
	double Beta_D;
	double Tcorr_D;

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
	double DeltaV_Update;
	double DeltaX_Update;
	double S_Update;
	double Yt_Update;
	double Yt_previous_Update;

	double DeltaT = 0.1;
	double Random_Wiener;


	 
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
	safety_criteria * SafetyCriterionAhead_R = 0;
	safety_criteria * SafetyCriterionAhead_L = 0;
	safety_criteria * SafetyCriterionBehind_R = 0;
	safety_criteria * SafetyCriterionBehind_L = 0;
	short IncentiveCriterion = 0;
	short lanechangingdecision = -10;

	short counter_path, counter_ramp;

	short flag = 0;
	short current_stream = 0;
 
	//==========================================End of Variable Definition============================================
	
	if (r1 != NULL)
	{
		p = r1;
		while (p->next != NULL)
			p = p->next;
		r1_head = p;
		p = r1;
	}
	else
	{
		r1_head = NULL;
		p = r1;
	}

	if (r2 != NULL)
	{
		q = r2;
		while (q->next != NULL)
			q = q->next;
		r2_head = q;
		q = r2;
	}
	else
	{
		r2_head = NULL;
		q = r2;
	}

	if (r3 != NULL)
	{
		s = r3;
		while (s->next != NULL)
			s = s->next;
		r3_head = s;
		s = r3;
	}
	else
	{
		r3_head = NULL;
		s = r3;
	}

	 
	
	
	
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************	
	
	
	
	while(p!=NULL)
	{	
		/*if (p->x < 1000)
			safeLC = 3 * safeLC1;
		else
			safeLC = safeLC1;*/

		if (p->x <(LaneDropLocation - 500)) // the vehicles should not change lane to the dropped lane after 500m to the point.
		{
			/*if (p->carID == 1)// && TIME > 885)// || p->carID == 7 )//&& p->x > 4900)//26)
			{
				cout<<"0"<<"	"<<TIME<<"	"<<p->carID<<"	"<<p->v<<"	"<<p->x<<"	"<<p->lanechangingflag<<"	"<<p->lane<<"	"<<p->Duration<<"	"<<p->l<<"	"<<p->Vd<<endl;//"	"<<p->crash<<"	"<<p->next->v<<"	"<<p->next->carID<<"	"<<p->next->crash<<endl;
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
			
			DXL1 = 0;
			DVL1 = 0;
			DXL2 = 0;
			DVL2 = 0;
			DXF1 = 0;
			DVF1 = 0;
			DXL1R = 0;
			DVL1R = 0;
			DXF1R = 0;
			DVF1R = 0;
			DXL1L = 0;
			DVL1L = 0;
			DXF1L = 0;
			DVF1L = 0;
			KR = 0;
			KL = 0;
			K = 0;

			origin = NULL;
			origin_head = NULL;
			destination = NULL;
			destination_head = NULL;
			destination_tail = NULL;
			thirdlane = NULL;
			thirdlane_head = NULL;
		 
			int compare_result;
			car *TempP;
			TempP = r1;
			compare_result = CompareLinkedLists (TempP, p);
			while (compare_result != 1)
			{
				TempP = TempP->next;
				compare_result = CompareLinkedLists (TempP, p);
			}

			p = TempP;
		 
			
			q = r2;
			s = r3;

			IncentiveCriterion = 0;

			r = p;

			if (r->crash == 1)
			{
				p=p->next;
				continue; //no lane change for crashed car!
			}

			if (p->next != NULL) //move p on down the list
			{
				p = p->next;
			}
			else //end of lane list reached
			{
				p = NULL;
				//break; //no lane changing for front car in lane
			}
			
			int change_flag = 0;

	//CASE 1
			
			//-----------------------------------------make decision here-----------------------------------------
			//Incentive Criterion: Hazard Computation and Monte-Carlo Simulation
		 
			//(?): Why r and not p

			if (r->next == NULL)
			{
				DXL1 = 0.0;
				DVL1 = 0.0;
			}
			else
			{
				DXL1 = r->headway;
				DVL1 = r->DV;
			}

			 
			if (r->next != NULL)
			{
				if (r->next->next == NULL)
				{
					DXL2 = 0.0;
					DVL2 = 0.0;
				}
				else
				{
					DXL2 = r->next->next->x - r->x;
					DVL2 = r->v - r->next->next->v;
				}
			}
			else
			{
				DXL2 = 0.0;
				DVL2 = 0.0;
			}


			if (r->previous == NULL)
			{
				DXF1 = 0;
				DVF1 = 0;
			}
			else
			{
				DXF1 = r->previous->headway;
				DVF1 = r->previous->DV;
			}
			 
			//Left Lane
			if (r2_head != NULL)
			{
				if (r->x < r2_head->x)
				{
					while(q != NULL)
					{
						if (r->x >= q->x) //check lane switch for p
						{
							q = q->next;
						}
						else
						{
							break;
						}
					}
					DXL1L = q->x - r->x;
					DVL1L = r->v - q->v;

					if ( q->previous != NULL )
					{
						DXF1L = r->x - q->previous->x;
						DVF1L = r->v - q->previous->v;
					}
					else
					{
						DXF1L = 0.0;//possibly 9999.0
						DVF1L = 0.0;//possibly 999.0
					}
				}
				else
				{
					DXL1L = 0.0;
					DVL1L = 0.0;

					DXF1L = r->x - r2_head->x;
					DVF1L = r->v - r2_head->v;

					q = r2_head;
				}
			}
			else
			{
				DXL1L = 0.0;
				DVL1L = 0.0;
				DXF1L = 0.0;
				DVF1L = 0.0;
			}
 
			//Right Lane
			if (r3_head != NULL)
			{
				if (r->x < r3_head->x)
				{
					while(s != NULL)
					{
						if (r->x >= s->x) //check lane switch for p
						{
							s = s->next;
						}
						else
						{
							break;
						}
					}
					DXL1R = s->x - r->x;
					DVL1R = r->v - s->v;

					if ( s->previous != NULL )
					{
						DXF1R = r->x - s->previous->x;
						DVF1R = r->v - s->previous->v;
					}
					else
					{
						DXF1R = 0.0;//possibly 9999.0
						DVF1R = 0.0;//possibly 999.0
					}
				}
				else
				{
					DXL1R = 0.0;
					DVL1R = 0.0;

					DXF1R = r->x - r3_head->x;
					DVF1R = r->v - r3_head->v;
					
					s = r3_head;
				}
			}
			else
			{
				DXL1R = 0.0;
				DVL1R = 0.0;
				DXF1R = 0.0;
				DVF1R = 0.0;
			}
	 

			//Density Computations
			//KL
			D2_Side = r2;
			NumberOfVehicles_Side2 = 0.0;

			while (D2_Side != NULL)
			{
				//if ( fabs ( D2_Side->x - r->x ) <= 1000)
				if (  ( r->x - D2_Side->x ) <= DistanceDensityBehind && ( r->x - D2_Side->x ) >= -DistanceDensityAhead  )
					NumberOfVehicles_Side2 = NumberOfVehicles_Side2 + 1.0;
			
				D2_Side = D2_Side->next;					
			}

			//KR
			D1_Side = r3;
			NumberOfVehicles_Side1 = 0.0;

			while (D1_Side != NULL)
			{
				//if ( fabs ( D2_Side->x - r->x ) <= 1000)
				if ((r->x - D1_Side->x) <= DistanceDensityBehind && (r->x - D1_Side->x) >= -DistanceDensityAhead)
					NumberOfVehicles_Side1 = NumberOfVehicles_Side1 + 1.0;
			
				D1_Side = D1_Side->next;
			}

			//K
			D_Total = r1;
			NumberOfVehicles_Total = 0.0;

			while (D_Total != NULL)
			{
				//if ( fabs ( D_Total->x - r->x ) <= 1000)
				if (  ( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
					NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
			
				D_Total = D_Total->next;
			}

			NumberOfVehicles_Total = NumberOfVehicles_Total + NumberOfVehicles_Side2;

			if (r->x >=DistanceDensityBehind && (L - r->x) >= DistanceDensityAhead)
				Distance_Density = (DistanceDensityBehind + DistanceDensityAhead);
			if (r->x >=DistanceDensityBehind && (L - r->x) < DistanceDensityAhead)
				Distance_Density = ( DistanceDensityBehind + (L - r->x) ) ;
			if (r->x < DistanceDensityBehind && (L - r->x) >= DistanceDensityAhead)
				Distance_Density = ( r->x + DistanceDensityAhead );
			if (r->x < DistanceDensityBehind && (L - r->x) < DistanceDensityAhead)
				Distance_Density = ( r->x + (L - r->x) ); 

			KL = ( NumberOfVehicles_Side2 * 1000.0) / Distance_Density; //in veh/Km.lane
			KR = ( NumberOfVehicles_Side1 * 1000.0) / Distance_Density; //in veh/Km.lane
			K = ( NumberOfVehicles_Total * 1000.0) / Distance_Density; //in veh/Km.lane
			
			//Car-Following Mode:
			//Assuming Weibull Distribution
			if ( r->Mode == 1)
			{
				//Number One: Limdep Technique
				Lambda_tmp = exp ( -1.0 * ( BetaOne_CF + BetaLCL_CF * r->LCL + BetaV_CF * r->v + BetaDXL1_CF * DXL1 + BetaDVL1_CF * DVL1 + BetaDXL2_CF * DXL2 + BetaDVL2_CF * DVL2 + BetaDXF1_CF * DXF1 + BetaDVF1_CF * DVF1 + BetaDXL1R_CF * DXL1R + BetaDVL1R_CF * DVL1R + BetaDXF1R_CF * DXF1R + BetaDVF1R_CF * DVF1R + BetaDXL1L_CF * DXL1L + BetaDVL1L_CF * DVL1L + BetaDXF1L_CF * DXF1L + BetaDVF1L_CF * DVF1L + BetaKR_CF * KR + BetaKL_CF * KL + BetaK_CF * K + BetaRightLane_CF * r->RightLaneDummy + BetaLeftLane_CF * r->LeftLaneDummy + BetaRamp_CF * r->RampDummy + BetaDistanceToRamp_CF * r->DistanceToRamp) );
				
				Hazard_ParameterA = Lambda_tmp;
				Hazard_ParameterB = 2.67;
				Hazard_tmp = Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) );
				Survival_tmp = exp (  -1.0 * pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB )  );
			
				R_Heterogeneity = K_Heterogeneity;

				Teta_tmp = ( 1.0 / K_Heterogeneity );

				Survival2_tmp = Survival_tmp * Teta_tmp;

				Hazard2_tmp = Survival2_tmp * Hazard_tmp;

				////Number Two: Bhat Technique
				//R_Heterogeneity = K_Heterogeneity;
				//Hazard_tmp = Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) );
				//Survival_tmp = exp (  -1.0 * pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB )  );
				//
				////Wrong:Gamma_tmp = ( pow ( K_Heterogeneity, R_Heterogeneity ) / double ( factorial ( int ( R_Heterogeneity - 1 ) ) ) ) * exp ( - 1.0 * K_Heterogeneity * V_tmp) * pow ( V_tmp , ( R_Heterogeneity - 1)  );
				//
				//Gamma_tmp = p->GammaHeterogeneity;
				//W_tmp = log (Gamma_tmp);//(?): What is 
				//Lambda_tmp = exp ( -1.0 * ( BetaOne_CF + BetaLCL_CF * r->LCL + BetaV_CF * r->v + BetaDXL1_CF * r->DXL1 + BetaDVL1_CF * r->DVL1 + BetaDXL2_CF * r->DXL2 + BetaDVL2_CF * r->DVL2 + BetaDXF1_CF * r->DXF1 + BetaDVF1_CF * r->DVF1 + BetaDXL1R_CF * r->DXL1R + BetaDVL1R_CF * r->DVL1R + BetaDXF1R_CF * r->DXF1R + BetaDVF1R_CF * r->DVF1R + BetaDXL1L_CF * r->DXL1L + BetaDVL1L_CF * r->DVL1L + BetaDXF1L_CF * r->DXF1L + BetaDVF1L_CF * r->DVF1L + BetaKR_CF * r->KR + BetaKL_CF * r->KL + BetaK_CF * r->K + BetaRightLane_CF * r->RightLaneDummy + BetaLeftLane_CF * r->LeftLaneDummy + BetaRamp_CF * r->RampDummy + BetaDistanceToRamp_CF * r->DistanceToRamp) + W_tmp);
				//Hazard2_tmp = Hazard_tmp * Lambda_tmp;
			}
			
			
			//Free-Flow Mode:
			//Assuming Exponential Distribution
			if ( r->Mode == 0)
			{
				//Number One: Limdep Technique
				Lambda_tmp = exp ( -1.0 * ( BetaOne_FF + BetaLCL_FF * r->LCL + BetaV_FF * r->v + BetaDXL1_FF * DXL1 + BetaDVL1_FF * DVL1 + BetaDXL2_FF * DXL2 + BetaDVL2_FF * DVL2 + BetaDXF1_FF * DXF1 + BetaDVF1_FF * DVF1 + BetaDXL1R_FF * DXL1R + BetaDVL1R_FF * DVL1R + BetaDXF1R_FF * DXF1R + BetaDVF1R_FF * DVF1R + BetaDXL1L_FF * DXL1L + BetaDVL1L_FF * DVL1L + BetaDXF1L_FF * DXF1L + BetaDVF1L_FF * DVF1L + BetaKR_FF * KR + BetaKL_FF * KL + BetaK_FF * K + BetaRightLane_FF * r->RightLaneDummy + BetaLeftLane_FF * r->LeftLaneDummy + BetaRamp_FF * r->RampDummy + BetaDistanceToRamp_FF * r->DistanceToRamp) );
				
				Hazard_ParameterA = Lambda_tmp;
				Hazard_ParameterB = 1.0;//Difference Between Exponential and Weibull
				Hazard_tmp = Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) );
				Survival_tmp = exp (  -1.0 * pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB )  );
			
				R_Heterogeneity = K_Heterogeneity;

				Teta_tmp = ( 1.0 / K_Heterogeneity );

				Survival2_tmp = Survival_tmp * Teta_tmp;

				Hazard2_tmp = Survival2_tmp * Hazard_tmp;

				////Number Two: Bhat Technique
				//R_Heterogeneity = K_Heterogeneity;
				//Hazard_ParameterB = 1.0;
				//Hazard_tmp = Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) );
				//Survival_tmp = exp (  -1.0 * pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB )  );
				//
				////Wrong:Gamma_tmp = ( pow ( K_Heterogeneity, R_Heterogeneity ) / double ( factorial ( int ( R_Heterogeneity - 1 ) ) ) ) * exp ( - 1.0 * K_Heterogeneity * V_tmp) * pow ( V_tmp , ( R_Heterogeneity - 1)  );
				//
				//Gamma_tmp = p->GammaHeterogeneity;
				//W_tmp = log (Gamma_tmp);//(?): What is v
				//Lambda_tmp = exp ( -1.0 * ( BetaOne_FF + BetaLCL_FF * r->LCL + BetaV_FF * r->v + BetaDXL1_FF * r->DXL1 + BetaDVL1_FF * r->DVL1 + BetaDXL2_FF * r->DXL2 + BetaDVL2_FF * r->DVL2 + BetaDXF1_FF * r->DXF1 + BetaDVF1_FF * r->DVF1 + BetaDXL1R_FF * r->DXL1R + BetaDVL1R_FF * r->DVL1R + BetaDXF1R_FF * r->DXF1R + BetaDVF1R_FF * r->DVF1R + BetaDXL1L_FF * r->DXL1L + BetaDVL1L_FF * r->DVL1L + BetaDXF1L_FF * r->DXF1L + BetaDVF1L_FF * r->DVF1L + BetaKR_FF * r->KR + BetaKL_FF * r->KL + BetaK_FF * r->K + BetaRightLane_FF * r->RightLaneDummy + BetaLeftLane_FF * r->LeftLaneDummy + BetaRamp_FF * r->RampDummy + BetaDistanceToRamp_FF * r->DistanceToRamp) + W_tmp);
				//Hazard2_tmp = Hazard_tmp * Lambda_tmp;
			}
			

			////Others Assuming Loglogistic Distribution
			////No Heterogeneity (Limdep Technique)
			//Lambda_tmp = exp ( -1.0 * ( BetaOne_CF + BetaLCL_CF * r->LCL + BetaV_CF * r->v + BetaDXL1_CF * r->DXL1 + BetaDVL1_CF * r->DVL1 + BetaDXL2_CF * r->DXL2 + BetaDVL2_CF * r->DVL2 + BetaDXF1_CF * r->DXF1 + BetaDVF1_CF * r->DVF1 + BetaDXL1R_CF * r->DXL1R + BetaDVL1R_CF * r->DVL1R + BetaDXF1R_CF * r->DXF1R + BetaDVF1R_CF * r->DVF1R + BetaDXL1L_CF * r->DXL1L + BetaDVL1L_CF * r->DVL1L + BetaDXF1L_CF * r->DXF1L + BetaDVF1L_CF * r->DVF1L + BetaKR_CF * r->KR + BetaKL_CF * r->KL + BetaK_CF * r->K) );
			//
			//Hazard_ParameterA = Lambda_tmp;
			//Hazard_tmp = ( ( Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) ) ) / ( 1 + pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB ) ) );
			//Survival_tmp = ( 1.0 / (  1.0 +  pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB ) ) );
		
			////R_Heterogeneity = K_Heterogeneity;

			////Teta_tmp = ( 1.0 / K_Heterogeneity );

			////Survival2_tmp = Survival_tmp * Teta_tmp;

			////Hazard2_tmp = Survival2_tmp * Hazard_tmp;


			////With Heterogeneity: Bhat Technique

			//R_Heterogeneity = K_Heterogeneity;
			//
			//Hazard_tmp = ( ( Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) ) ) / ( 1 + pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB ) ) );
			//Survival_tmp = ( 1.0 / (  1.0 +  pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB ) ) );
			//
			////Wrong:Gamma_tmp = ( pow ( K_Heterogeneity, R_Heterogeneity ) / double ( factorial ( int ( R_Heterogeneity - 1 ) ) ) ) * exp ( - 1.0 * K_Heterogeneity * V_tmp) * pow ( V_tmp , ( R_Heterogeneity - 1)  );
			//
			//Gamma_tmp = p->GammaHeterogeneity;
			//W_tmp = log (Gamma_tmp);//(?): What is v
			//Lambda_tmp = exp ( -1.0 * ( BetaOne_CF + BetaLCL_CF * r->LCL + BetaV_CF * r->v + BetaDXL1_CF * r->DXL1 + BetaDVL1_CF * r->DVL1 + BetaDXL2_CF * r->DXL2 + BetaDVL2_CF * r->DVL2 + BetaDXF1_CF * r->DXF1 + BetaDVF1_CF * r->DVF1 + BetaDXL1R_CF * r->DXL1R + BetaDVL1R_CF * r->DVL1R + BetaDXF1R_CF * r->DXF1R + BetaDVF1R_CF * r->DVF1R + BetaDXL1L_CF * r->DXL1L + BetaDVL1L_CF * r->DVL1L + BetaDXF1L_CF * r->DXF1L + BetaDVF1L_CF * r->DVF1L + BetaKR_CF * r->KR + BetaKL_CF * r->KL + BetaK_CF * r->K) + W_tmp);
			//Hazard2_tmp = Hazard_tmp * Lambda_tmp;

			//Crash Mode:
			if ( r->Mode == -1 )
			{
				Hazard2_tmp = 0.0;
			}

			if (Hazard2_tmp > -10 && Hazard2_tmp < 10)
				r->basehazard = Hazard2_tmp;
			else 
				r->basehazard = -1000;

			PercentLaneChange = mrand(102); //PercentLaneChange is between 0 and 1

			r->Hazard = Hazard2_tmp;

			if (r->next != NULL)
			{
				if(r->next->crash == 1)
				{
					r->Hazard = 10;
				}
			}

			//***************************************************************************************************
			//***************************************************************************************************
			//*************************************Mandatory Lane Changing***************************************
			//***************************************************************************************************
			//***************************************************************************************************
			r->targetrampside = 2;//initial value
			current_stream = 0;
			if (r->lane < 10)
				current_stream = 1;
			else
				current_stream = r->lane;

			if (current_stream != 1) //Not in the Main Stream
			{
				counter_path = 0;
				while (Path[r->carID][counter_path] != current_stream)
					counter_path++;

				counter_ramp = 0;
				while (Path[r->carID][counter_path] != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;

				if (ramp_information[counter_ramp].Ramp_Type == 1)
					r->Hazard = -10;

				if (ramp_information[counter_ramp].Ramp_Type == 0)
				{
					if (r->x >= ramp_information[counter_ramp].Ramp_Location && r->x <= (ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length))
						r->Hazard = 10;
					else
						r->Hazard = -10;
				}
				if (ramp_information[counter_ramp].Ramp_Type == 2)
				{
					if (counter_path < Path[r->carID].size() - 1)
						if (r->x >= ramp_information[counter_ramp].Ramp_Location && r->x <= (ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length))
							r->Hazard = 10;
						else
						{
							r->Hazard = -10;
							r->l = 0;
						}
					else
					{
						r->Hazard = -10;
						r->l = 0;
					}
				}				
			}
			else //In the Main Stream
			{
				counter_path = 0;
				while (Path[r->carID][counter_path] != current_stream)
					counter_path++;
				
				//counter_path = counter_path + 1;
				if (counter_path == (Path[r->carID].size() - 1))
					r->Hazard = Hazard2_tmp;
				else
				{
					r->Hazard = Hazard2_tmp;
					counter_ramp = 0;
					while (Path[r->carID][counter_path + 1] != ramp_information[counter_ramp].Ramp_Number)
						counter_ramp++;

					if (ramp_information[counter_ramp].Ramp_Side == 1) // ramp on the righ hand side in the direction of travel
					{
						r->targetrampside = 1;
						if (r->lane == 0)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 1500 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 1)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 1000 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 2)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 500 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 3)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
					}
					else // ramp on the left hand side in the direction of travel
					{
						r->targetrampside = 0;
						if (r->lane == 3)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 1500 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 2)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 1000 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 1)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 500 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 0)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
					}
				}
			}

			lanechangingdecision = Lane_Decision(r, r->Hazard, r->targetrampside);
			//***************************************************************************************************
			//***************************************************************************************************
			//*************************************Mandatory Lane Changing***************************************
			//***************************************************************************************************
			//***************************************************************************************************
			if (current_stream = 1) //Not in the Main Stream
				if (r->next != NULL)
					if (r->next->crash == 1)
						r->Hazard = 10;

			if ( PercentLaneChange <= r->Hazard )
				IncentiveCriterion = 1;
			else
				IncentiveCriterion = 0;
			 

			//Safety Criterion
		 
			//--------------------------------------end of make decision here-------------------------------------

			if (IncentiveCriterion == 1 || r->l > 0)
			{
				SafetyCriterionAhead_L = Safety_Criteria_Ahead (r, q, r1_head, r2_head);
				SafetyCriterionAhead_R = Safety_Criteria_Ahead (r, s, r1_head, r3_head);
				SafetyCriterionBehind_L = Safety_Criteria_Behind (r, q, r1_head, r2_head);
				SafetyCriterionBehind_R = Safety_Criteria_Behind (r, s, r1_head, r3_head);

				/*

						   \  L20i \		  /	 L20i /
							\		\		 /		 /
							 \		 --------		/
							  \			(6)		   /
				--------------- --  --  --  -- -- ---------------------
				L0-lane 1		-->q  r2	(5)				(0)
				-------------------------------------------------------
				L1-lane 2		-->p  r1					(1)
				-------------------------------------------------------
				L2-lane 3		-->s  r3					(1)
				-------------------------------------------------------
				L3-lane 4					(3)				(2)
				--------------- --  --  --  -- -- ---------------------
							  /		   (4)		  \
							 /      ---------	   \
							/      /		 \	    \
						   / L10i /			  \ L10i \
				*/

				if (lanechangingdecision == 0)
				{
					if (( SafetyCriterionAhead_R->safety == 1) && ( SafetyCriterionBehind_R->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = s;
						destination_head = r3_head;
						destination_tail = r3;
						thirdlane = NULL;
						thirdlane_head = NULL;
						if (destination == NULL)
							origin->destlane = 3;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = NULL;
							thirdlane_head = NULL;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
				}

				if (lanechangingdecision == 1)
				{
					if (( SafetyCriterionAhead_R->safety > SafetyCriterionBehind_L->safety))
					{
						if ( SafetyCriterionBehind_R->safety == 1)
						{
							if (r->lanechangingflag == 0)
							{
								r->l = r->LCT; //switch lanes after LCT timesteps
								r->lanechangingflag = 1;
							}
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = r2;
							thirdlane_head = r2_head;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
					else if (( SafetyCriterionAhead_R->safety < SafetyCriterionBehind_L->safety))
					{
						if ( SafetyCriterionBehind_L->safety == 1)
						{
							if (r->lanechangingflag == 0)
							{
								r->l = r->LCT; //switch lanes after LCT timesteps
								r->lanechangingflag = 1;
							}
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = r3;
							thirdlane_head = r3_head;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
					else if (( SafetyCriterionAhead_R->safety == SafetyCriterionBehind_L->safety) && (SafetyCriterionBehind_L->safety == 1))
					{
						if ((SafetyCriterionAhead_R->acc + SafetyCriterionBehind_R->acc) >= (SafetyCriterionAhead_L->acc + SafetyCriterionBehind_L->acc))
						{							
							if (r->lanechangingflag == 0)
							{
								r->l = r->LCT; //switch lanes after LCT timesteps
								r->lanechangingflag = 1;
								r->changelane_direction = 1;
							}
							else
							{
								if (r->changelane_direction == 0)
									r->l = r->LCT;
							}
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = r2;
							thirdlane_head = r2_head;
							if (destination == NULL)
								origin->destlane = 3;
						}
						else
						{
							if (r->lanechangingflag == 0)
							{
								r->l = r->LCT; //switch lanes after LCT timesteps
								r->lanechangingflag = 1;
								r->changelane_direction = 0;
							}
							else
							{
								if (r->changelane_direction == 1)
									r->l = r->LCT;
							}
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = r3;
							thirdlane_head = r3_head;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = NULL;
							thirdlane_head = NULL;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
				}

				if (lanechangingdecision == 2)
				{
					if (( SafetyCriterionAhead_L->safety == 1) && ( SafetyCriterionBehind_L->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						else
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
						}
						origin = r;
						origin_head = r1_head;
						destination = q;
						destination_head = r2_head;
						destination_tail = r2;
						thirdlane = NULL;
						thirdlane_head = NULL;
						if (destination == NULL)
							origin->destlane = 2;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = NULL;
							thirdlane_head = NULL;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
				}

				if (lanechangingdecision == 3)
				{
					if (( SafetyCriterionAhead_R->safety == 1) && ( SafetyCriterionBehind_R->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = s;
						destination_head = r3_head;
						destination_tail = r3;
						thirdlane = r2;
						thirdlane_head = r2_head;
						if (destination == NULL)
							origin->destlane = 3;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = r2;
							thirdlane_head = r2_head;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
				}

				if (lanechangingdecision == 4)
				{
					if (( SafetyCriterionAhead_L->safety == 1) && ( SafetyCriterionBehind_L->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = q;
						destination_head = r2_head;
						destination_tail = r2;
						thirdlane = r3;
						thirdlane_head = r3_head;
						if (destination == NULL)
							origin->destlane = 2;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = r3;
							thirdlane_head = r3_head;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
				}

				if (lanechangingdecision == 5)
				{
					if (( SafetyCriterionAhead_L->safety == 1) && ( SafetyCriterionBehind_L->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = q;
						destination_head = r2_head;
						destination_tail = r2;
						thirdlane = r3;
						thirdlane_head = r3_head;
						if (destination == NULL)
							origin->destlane = 2;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = r3;
							thirdlane_head = r3_head;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
				}

				if (lanechangingdecision == 6)
				{
					if (( SafetyCriterionAhead_R->safety == 1) && ( SafetyCriterionBehind_R->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = s;
						destination_head = r3_head;
						destination_tail = r3;
						thirdlane = r2;
						thirdlane_head = r2_head;
						if (destination == NULL)
							origin->destlane = 3;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = r2;
							thirdlane_head = r2_head;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
				}
			}
			else
				origin = r;


			//lane change process 
			if (origin != NULL)
			{
				if (origin->l > 0) //lane switch coming up for that car
				{
					origin->l--;

					//do lane change
					if (origin->l == 0) //time to change lanes reached
					{  //change lanes
						
						//a = (int) (mrand(38) * 100); //random int between 1 and 100
						//a = 0;//test
 						b=0; //using as a flag
						if (destination != NULL)
						{
							if (destination->previous == NULL)
							{// there is no car behind q
								if ( (destination->x - destination->s) > (origin->x + safeLC) )// && a < r->LCcrash)
								{//change lane
									b=1;
								}
							}
							else
							{// there is some car behind q
								if (((destination->x - destination->s) > (origin->x + safeLC)) && ((origin->x - origin->s) > (destination->previous->x + safeLC)))//&& a < r->LCcrash)
								{
									b=1;
								}
							}
							if (destination->next == NULL && b == 0)
							{
								if ( (destination->x + safeLC) < (origin->x - origin->s) )// && a < r->LCcrash)
								{//change lane
									b = 0;
									origin->lanechangingflag = 0;

									origin->changelaneinstance = 1;
									flag = 1;
									 
									car *CopyTemp;
									CopyTemp = NULL;
									CopyTemp = CopyStream(r1);

									int compare;
									compare = CompareLinkedLists (CopyTemp, origin);
									while (compare == -1)
									{
										CopyTemp = CopyTemp->next;
										compare = CompareLinkedLists (CopyTemp , origin);
									};
									
									 
									
								 
									//CopyTemp->lane_prev = origin->lane;
									//CopyTemp->lane = destination->lane;
																								
									// remove CopyTemp from r1
									if (CopyTemp->previous != NULL)
									{
										//car behind CopyTemp
										CopyTemp->previous->next = CopyTemp->next;
										
										if (CopyTemp->next!=NULL)
										{
											//car in front of CopyTemp
											CopyTemp->next->previous = CopyTemp->previous;
											CopyTemp = CopyTemp->next;
										}
										else
										{
											r1_head = CopyTemp->previous;
											CopyTemp = CopyTemp->previous;
										}
									}
									else //was the first of its lane
									{
										if (CopyTemp->next!=NULL)
										{// set r->next is the r1
											CopyTemp->next->previous = NULL;
											r1 = CopyTemp->next;
											CopyTemp = CopyTemp->next;
											
											if (r1->next == NULL)
											{
												r1_head = r1;
											}
										}
										else
										{
											r1=NULL;
											r1_head = NULL;
											CopyTemp = NULL;
										}
									}
									 

									// insert r into r2, q<>NULL now
																	
									 
									if (origin->lane < 10 || (origin->lane >= 10 && destination->lane < 10))
									{								
										if (origin->x >= VSLStartLocation && origin->x < VSLEndLocation)
											origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//(r->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//DriversSpeedLimit(DesiredSpeedLimit_temp, Compliance);
										else										
											origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeed) + DesiredSpeed;
									}
									 

									origin->next = NULL;
									origin->previous = destination;
									origin->lane_prev = origin->lane;
									origin->lane = destination->lane;

									if (destination->previous!=NULL)
									{
										destination->next = origin;
										destination_head = origin;
									}
									else
									{
										destination->next = origin;
										destination_tail = destination;
										destination_head = origin ;
									}
									

									 
									car *Tempr2q;
									Tempr2q = NULL;
									while (destination->previous != NULL)
									{
										destination = destination->previous;
									}
									Tempr2q = CopyStream(destination);
									destination_tail = NULL;
									destination_tail = CopyStream(Tempr2q);
									
									if (r2 != NULL)
									{
										if (destination_tail->lane == r2->lane)
											r2 = destination_tail;
									}
									if (r3!= NULL)
									{
										if (destination_tail->lane == r3->lane)
											r3 = destination_tail;
									}
									
									car *Tempr2_headq;
									Tempr2_headq= NULL;
									Tempr2_headq = destination_tail;
									while (Tempr2_headq->next != NULL)
									{
										Tempr2_headq = Tempr2_headq->next;
									}
									destination_head = Tempr2_headq;

									if (r2 != NULL)
									{
										if (destination_head->lane == r2->lane)
											r2_head = destination_head;							
									}
									if (r3 != NULL)
									{
										if (destination_head->lane == r3->lane)
											r3_head = destination_head;
									}

									 

									 
									if (CopyTemp != NULL)
									{
										while (CopyTemp->previous != NULL)
										{
											CopyTemp = CopyTemp->previous;
										};
										r1 = CopyStream(CopyTemp);
										
										if (r1->next == NULL)
										{
											r1_head = r1;
										}
									}
									else
									{
										r1 = NULL;
										r1_head = NULL;
									};

									if (r1 != NULL)
									{
										car *tempupdate;
										tempupdate = r1;
										while (tempupdate->next != NULL)
										{
											tempupdate = tempupdate->next;
										}
										r1_head = tempupdate;
									}
									 
								}
							}

							
							origin->lanechangingflag = 0;

							//lane change
							if (b==1)
							{// lane change
								origin->changelaneinstance = 1;
								flag = 1;
								
								car *CopyTemp;
								CopyTemp = NULL;
								CopyTemp = CopyStream(r1);

								int compare;
								compare = CompareLinkedLists (CopyTemp, origin);
								while (compare == -1)
								{
									CopyTemp = CopyTemp->next;
									compare = CompareLinkedLists (CopyTemp , origin);
								};
								
								 
								
								 
								//CopyTemp->lane_prev = origin->lane;
								//CopyTemp->lane = destination->lane;
																							
								// remove CopyTemp from r1
								if (CopyTemp->previous != NULL)
								{
									//car behind CopyTemp
									CopyTemp->previous->next = CopyTemp->next;
									
									if (CopyTemp->next!=NULL)
									{
										//car in front of CopyTemp
										CopyTemp->next->previous = CopyTemp->previous;
										CopyTemp = CopyTemp->next;
									}
									else
									{
										r1_head = CopyTemp->previous;
										CopyTemp = CopyTemp->previous;
									}
								}
								else //was the first of its lane
								{
									if (CopyTemp->next!=NULL)
									{// set r->next is the r1
										CopyTemp->next->previous = NULL;
										r1 = CopyTemp->next;
										CopyTemp = CopyTemp->next;
										
										if (r1->next == NULL)
										{
											r1_head = r1;
										}
									}
									else
									{
										r1=NULL;
										r1_head = NULL;
										CopyTemp = NULL;
									}
								}
								 

								// insert r into r2, q<>NULL now
																
								 
								if (origin->lane < 10 || (origin->lane >= 10 && destination->lane < 10))
								{
									if (origin->x >= VSLStartLocation && origin->x < VSLEndLocation)
										origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//(r->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//DriversSpeedLimit(DesiredSpeedLimit_temp, Compliance);
									else										
										origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeed) + DesiredSpeed;
								}
								 

								origin->next = destination;
								origin->previous = destination->previous;
								origin->lane_prev = origin->lane;
								origin->lane = destination->lane;

								if (destination->previous!=NULL)
								{
									destination->previous->next = origin;
								}
								else
								{
									destination->previous = origin;
									destination_tail = origin;
									
									if (destination_tail->next == NULL)
									{
										destination_head = destination_tail ;
									}
								}
								

								 
								car *Tempr2q;
								Tempr2q = NULL;
								while (destination->previous != NULL)
								{
									destination = destination->previous;
								}
								Tempr2q = CopyStream(destination);
								destination_tail = NULL;
								destination_tail = CopyStream(Tempr2q);
								
								if (r2 != NULL)
								{
									if (destination_tail->lane == r2->lane)
										r2 = destination_tail;
								}
								if (r3!= NULL)
								{
									if (destination_tail->lane == r3->lane)
										r3 = destination_tail;
								}
								
								car *Tempr2_headq;
								Tempr2_headq= NULL;
								Tempr2_headq = destination_tail;
								while (Tempr2_headq->next != NULL)
								{
									Tempr2_headq = Tempr2_headq->next;
								}
								destination_head = Tempr2_headq;

								if (r2 != NULL)
								{
									if (destination_head->lane == r2->lane)
										r2_head = destination_head;							
								}
								if (r3 != NULL)
								{
									if (destination_head->lane == r3->lane)
										r3_head = destination_head;
								}

								 

							 
								if (CopyTemp != NULL)
								{
									while (CopyTemp->previous != NULL)
									{
										CopyTemp = CopyTemp->previous;
									};
									r1 = CopyStream(CopyTemp);
									
									if (r1->next == NULL)
									{
										r1_head = r1;
									}
								}
								else
								{
									r1 = NULL;
									r1_head = NULL;
								};

								if (r1 != NULL)
								{
									car *tempupdate;
									tempupdate = r1;
									while (tempupdate->next != NULL)
									{
										tempupdate = tempupdate->next;
									}
									r1_head = tempupdate;
								}
								 
															
							}
						}
						else
						{
							flag = 1;
							origin->changelaneinstance = 1;
							origin->lanechangingflag = 0;
							 
							car *CopyTemp;
							CopyTemp = NULL;
							CopyTemp = CopyStream(r1);

							int compare;
							compare = CompareLinkedLists (CopyTemp, origin);
							while (compare == -1)
							{
								CopyTemp = CopyTemp->next;
								compare = CompareLinkedLists (CopyTemp , origin);
							};
							
							 
							//CopyTemp->lane_prev = origin->lane;
							//CopyTemp->lane = destination->lane;
																						
							// remove CopyTemp from r1
							if (CopyTemp->previous != NULL)
							{
								//car behind CopyTemp
								CopyTemp->previous->next = CopyTemp->next;
								
								if (CopyTemp->next!=NULL)
								{
									//car in front of CopyTemp
									CopyTemp->next->previous = CopyTemp->previous;
									CopyTemp = CopyTemp->next;
								}
								else
								{
									r1_head = CopyTemp->previous;
									CopyTemp = CopyTemp->previous;
								}
							}
							else //was the first of its lane
							{
								if (CopyTemp->next!=NULL)
								{// set r->next is the r1
									CopyTemp->next->previous = NULL;
									r1 = CopyTemp->next;
									CopyTemp = CopyTemp->next;
									
									if (r1->next == NULL)
									{
										r1_head = r1;
									}
								}
								else
								{
									r1=NULL;
									r1_head = NULL;
									CopyTemp = NULL;
								}
							}
							 

							// insert r into r2, q<>NULL now
															
							 
							if (origin->lane < 10)
							{
								if (origin->x >= VSLStartLocation && origin->x < VSLEndLocation)
									origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//(r->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//DriversSpeedLimit(DesiredSpeedLimit_temp, Compliance);
								else										
									origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeed) + DesiredSpeed;
							}
							 
							if (CopyTemp != NULL)
							{
								while (CopyTemp->previous != NULL)
								{
									CopyTemp = CopyTemp->previous;
								};
								r1 = CopyStream(CopyTemp);
								
								if (r1->next == NULL)
								{
									r1_head = r1;
								}
							}
							else
							{
								r1 = NULL;
								r1_head = NULL;
							};

							if (r1 != NULL)
							{
								car *tempupdate;
								tempupdate = r1;
								while (tempupdate->next != NULL)
								{
									tempupdate = tempupdate->next;
								}
								r1_head = tempupdate;
							}
							 
							
							if (origin->destlane == 2)
							{
								origin->next = NULL;
								origin->previous = NULL;
								origin->lane_prev = origin->lane;
								if (origin->lane < 10)
									origin->lane = (origin->lane - 1);
								else
									origin->lane = 3;
								r2 = r2_head = origin;
							}
							if (origin->destlane == 3)
							{
								origin->next = NULL;
								origin->previous = NULL;								
								origin->lane_prev = origin->lane;
								if (origin->lane < 10)
									origin->lane = (origin->lane + 1);
								else
									origin->lane = 3;
								r3 = r3_head = origin;
							}
						}
					}
					else
					{
						while (origin->previous != NULL)
							origin = origin->previous;
						r1 = origin;
						while (origin->next != NULL)
							origin = origin->next;
						r1_head = origin;
					}
				}
			}
		}
	}


	Lane_Changing_temp->Ltmp1 = r1;
	Lane_Changing_temp->Ltmp2= r2;
	Lane_Changing_temp->Ltmp3 = r3;
	Lane_Changing_temp->Ltmp1_head = r1_head;
	Lane_Changing_temp->Ltmp2_head = r2_head;
	Lane_Changing_temp->Ltmp3_head = r3_head;

	/*p = NULL;
	r1_head = NULL;
	q = NULL;
	r2_head = NULL;
	s = NULL;
	r3_head = NULL;
	r = NULL;
	origin = NULL;
	origin_head = NULL;
	destination = NULL;
	destination_head = NULL;
	destination_tail = NULL;
	thirdlane = NULL;
	thirdlane_head = NULL;
	D1_Side = NULL;
	D2_Side = NULL;
	D_Total = NULL;
	
	free(p);
	free(q);
	free(r1_head);
	free(r2_head);
	free(s);
	free(r3_head);
	free(r);
	free(origin);
	free(origin_head);
	free(destination);
	free(destination_head);
	free(destination_tail);
	free(thirdlane);
	free(thirdlane_head);
	free(D1_Side);
	free(D2_Side);
	free(D_Total);*/

	return (Lane_Changing_temp);
	free(Lane_Changing_temp);

};


//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################
//##############################################################################################################################


struct lane_changing_data_RML * Lane_Changing_RML (car * r1, car * r2, car * r10, car * r11, car * r12, car * r13, car * r14, car * r15, car * r16, car * r17, car * r18, double DesiredSpeedLimit_temp, double VSLStartLocation_temp, double VSLEndLocation_temp)//RML: Right Most Lane
{
	//=============================================Variable Definition================================================	
	lane_changing_data_RML * Lane_Changing_temp_RML = new lane_changing_data_RML;
	//Lane_Changing_temp_RML = (lane_changing_data_RML *) malloc(sizeof(Lane_Changing_temp_RML));
		Lane_Changing_temp_RML->LtmpRML1 = NULL;
		Lane_Changing_temp_RML->LtmpRML2 = NULL;	
		Lane_Changing_temp_RML->LtmpRML10 = NULL;
		Lane_Changing_temp_RML->LtmpRML11 = NULL;
		Lane_Changing_temp_RML->LtmpRML12 = NULL;
		Lane_Changing_temp_RML->LtmpRML13 = NULL;
		Lane_Changing_temp_RML->LtmpRML14 = NULL;
		Lane_Changing_temp_RML->LtmpRML15 = NULL;
		Lane_Changing_temp_RML->LtmpRML16 = NULL;
		Lane_Changing_temp_RML->LtmpRML17 = NULL;
		Lane_Changing_temp_RML->LtmpRML18 = NULL;		
		Lane_Changing_temp_RML->LtmpRML1_head = NULL;
		Lane_Changing_temp_RML->LtmpRML2_head = NULL;	
		Lane_Changing_temp_RML->LtmpRML10_head = NULL;
		Lane_Changing_temp_RML->LtmpRML11_head = NULL;
		Lane_Changing_temp_RML->LtmpRML12_head = NULL;
		Lane_Changing_temp_RML->LtmpRML13_head = NULL;
		Lane_Changing_temp_RML->LtmpRML14_head = NULL;
		Lane_Changing_temp_RML->LtmpRML15_head = NULL;
		Lane_Changing_temp_RML->LtmpRML16_head = NULL;
		Lane_Changing_temp_RML->LtmpRML17_head = NULL;
		Lane_Changing_temp_RML->LtmpRML18_head = NULL;
		

		
	float DXL1 = 0;
	float DVL1 = 0;
	float DXL2 = 0;
	float DVL2 = 0;
	float DXF1 = 0;
	float DVF1 = 0;
	float DXL1R = 0;
	float DVL1R = 0;
	float DXF1R = 0;
	float DVF1R = 0;
	float DXL1L = 0;
	float DVL1L = 0;
	float DXF1L = 0;
	float DVF1L = 0;
	float KR = 0;
	float KL = 0;
	float K = 0;
		
	car * p, * r1_head;
	car * q, * r2_head;
	car * s, * r3_head = NULL, * r3 = NULL;
	car * r10_head, * r11_head, * r12_head, * r13_head, * r14_head, * r15_head, * r16_head, * r17_head, * r18_head, * r19_head, * r20_head; 
	car * r;
	car * origin = NULL, * origin_head = NULL, *destination = NULL, * destination_head = NULL, * destination_tail = NULL, * thirdlane = NULL, * thirdlane_head = NULL;
	car * D1_Side, *D2_Side, *D_Total;
	float a;
	short b;
	short mandatorylanechanging = 0;
	//PT Model Parameters: Update Purposes Taken from "car" Structs
	double Gamma_D;
	double Wm_D;
	double Wc_D;
	double Tmax_D;
	double Alpha_D;
	double Beta_D;
	double Tcorr_D;

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
	double DeltaV_Update;
	double DeltaX_Update;
	double S_Update;
	double Yt_Update;
	double Yt_previous_Update;

	double DeltaT = 0.1;
	double Random_Wiener;


	 
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
	safety_criteria * SafetyCriterionAhead_R = 0;
	safety_criteria * SafetyCriterionAhead_L = 0;
	safety_criteria * SafetyCriterionBehind_R = 0;
	safety_criteria * SafetyCriterionBehind_L = 0;
	short IncentiveCriterion = 0;
	short lanechangingdecision = -10;

	short counter_path, counter_ramp;

	short flag = 0;
	short current_stream = 0;
	short check_ramp = -1;	
 
	//==========================================End of Variable Definition============================================
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r1 != NULL)
	{
		p = r1;
		while (p->next != NULL)
			p = p->next;
		r1_head = p;
		p = r1;
	}
	else
	{
		r1_head = NULL;
		p = r1;
	}	
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r10 != NULL)
	{
		q = r10;
		while (q->next != NULL)
			q = q->next;
		r10_head = q;
		q = r10;
	}
	else
	{
		r10_head = NULL;
		q = r10;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r11 != NULL)
	{
		q = r11;
		while (q->next != NULL)
			q = q->next;
		r11_head = q;
		q = r11;
	}
	else
	{
		r11_head = NULL;
		q = r11;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r12 != NULL)
	{
		q = r12;
		while (q->next != NULL)
			q = q->next;
		r12_head = q;
		q = r12;
	}
	else
	{
		r12_head = NULL;
		q = r12;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r13 != NULL)
	{
		q = r13;
		while (q->next != NULL)
			q = q->next;
		r13_head = q;
		q = r13;
	}
	else
	{
		r13_head = NULL;
		q = r13;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r14 != NULL)
	{
		q = r14;
		while (q->next != NULL)
			q = q->next;
		r14_head = q;
		q = r14;
	}
	else
	{
		r14_head = NULL;
		q = r14;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r15 != NULL)
	{
		q = r15;
		while (q->next != NULL)
			q = q->next;
		r15_head = q;
		q = r15;
	}
	else
	{
		r15_head = NULL;
		q = r15;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r16 != NULL)
	{
		q = r16;
		while (q->next != NULL)
			q = q->next;
		r16_head = q;
		q = r16;
	}
	else
	{
		r16_head = NULL;
		q = r16;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r17 != NULL)
	{
		q = r17;
		while (q->next != NULL)
			q = q->next;
		r17_head = q;
		q = r17;
	}
	else
	{
		r17_head = NULL;
		q = r17;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r18 != NULL)
	{
		q = r18;
		while (q->next != NULL)
			q = q->next;
		r18_head = q;
		q = r18;
	}
	else
	{
		r18_head = NULL;
		q = r18;
	}	
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	if (r2 != NULL)
	{
		q = r2;
		while (q->next != NULL)
			q = q->next;
		r2_head = q;
		q = r2;
	}
	else
	{
		r2_head = NULL;
		q = r2;
	}
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	
	 
	
	
	
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************
	//*******************************************************************************************************	
	
	
	
	while(p!=NULL)
	{	
		/*if (p->x < 1000)
			safeLC = 3 * safeLC1;
		else
			safeLC = safeLC1;*/

		if (p->x <(LaneDropLocation - 500)) // the vehicles should not change lane to the dropped lane after 500m to the point.
		{
			// added by ART 07/23/11
			DXL1 = 0;
			DVL1 = 0;
			DXL2 = 0;
			DVL2 = 0;
			DXF1 = 0;
			DVF1 = 0;
			DXL1R = 0;
			DVL1R = 0;
			DXF1R = 0;
			DVF1R = 0;
			DXL1L = 0;
			DVL1L = 0;
			DXF1L = 0;
			DVF1L = 0;
			KR = 0;
			KL = 0;
			K = 0;
			
			origin = NULL;
			origin_head = NULL;
			destination = NULL;
			destination_head = NULL;
			destination_tail = NULL;
			thirdlane = NULL;
			thirdlane_head = NULL;
			
			
			int compare_result;
			car *TempP;
			TempP = r1;
			compare_result = CompareLinkedLists (TempP, p);
			while (compare_result != 1)
			{
				TempP = TempP->next;
				compare_result = CompareLinkedLists (TempP, p);
			}

			p = TempP;
		 
			check_ramp = -1;

			for (int cont = 0; cont < ramp_information.size() - 1; cont ++)
			{
				if ((p->x >= ramp_information[cont].Ramp_Location) && (p->x <= (ramp_information[cont].Ramp_Location + ramp_information[cont].Ramp_Merging_Length)))
				{
					check_ramp = cont;
					break;
				}
			}



			r3 = ramp_output (check_ramp, r10, r11, r12, r13, r14, r15, r16, r17, r18);
			if (r3 != NULL)
			{
				s = r3;
				while (s->next != NULL)
					s = s->next;
				r3_head = s;
				s = r3;
			}
			else
			{
				r3_head = NULL;
				s = r3;
			}		

			
			q = r2;
			s = r3;

			IncentiveCriterion = 0;

			r = p;

			if (r->crash == 1)
			{
				p=p->next;
				continue; //no lane change for crashed car!
			}

			if (p->next != NULL) //move p on down the list
			{
				p = p->next;
			}
			else //end of lane list reached
			{
				p = NULL;
				//break; //no lane changing for front car in lane
			}
			
			int change_flag = 0;

	//CASE 1
			
			//-----------------------------------------make decision here-----------------------------------------
			//Incentive Criterion: Hazard Computation and Monte-Carlo Simulation
		 
			//(?): Why r and not p

			if (r->next == NULL)
			{
				DXL1 = 0.0;
				DVL1 = 0.0;
			}
			else
			{
				DXL1 = r->headway;
				DVL1 = r->DV;
			}

			// added by ART 07/20/11
			if (r->next != NULL)
			{
				if (r->next->next == NULL)
				{
					DXL2 = 0.0;
					DVL2 = 0.0;
				}
				else
				{
					DXL2 = r->next->next->x - r->x;
					DVL2 = r->v - r->next->next->v;
				}
			}
			else
			{
				DXL2 = 0.0;
				DVL2 = 0.0;
			}


			if (r->previous == NULL)
			{
				DXF1 = 0;
				DVF1 = 0;
			}
			else
			{
				DXF1 = r->previous->headway;
				DVF1 = r->previous->DV;
			}
	 
			//Left Lane
			if (r2_head != NULL)
			{
				if (r->x < r2_head->x)
				{
					while(q != NULL)
					{
						if (r->x >= q->x) //check lane switch for p
						{
							q = q->next;
						}
						else
						{
							break;
						}
					}
					DXL1L = q->x - r->x;
					DVL1L = r->v - q->v;

					if ( q->previous != NULL )
					{
						DXF1L = r->x - q->previous->x;
						DVF1L = r->v - q->previous->v;
					}
					else
					{
						DXF1L = 0.0;//possibly 9999.0
						DVF1L = 0.0;//possibly 999.0
					}
				}
				else
				{
					DXL1L = 0.0;
					DVL1L = 0.0;

					DXF1L = r->x - r2_head->x;
					DVF1L = r->v - r2_head->v;

					q = r2_head;
				}
			}
			else
			{
				DXL1L = 0.0;
				DVL1L = 0.0;
				DXF1L = 0.0;
				DVF1L = 0.0;
			}
			 
			//Right Lane
			if (r3_head != NULL)
			{
				if (r->x < r3_head->x)
				{
					while(s != NULL)
					{
						if (r->x >= s->x) //check lane switch for p
						{
							s = s->next;
						}
						else
						{
							break;
						}
					}
					DXL1R = s->x - r->x;
					DVL1R = r->v - s->v;

					if ( s->previous != NULL )
					{
						DXF1R = r->x - s->previous->x;
						DVF1R = r->v - s->previous->v;
					}
					else
					{
						DXF1R = 0.0;//possibly 9999.0
						DVF1R = 0.0;//possibly 999.0
					}
				}
				else
				{
					DXL1R = 0.0;
					DVL1R = 0.0;

					DXF1R = r->x - r3_head->x;
					DVF1R = r->v - r3_head->v;

					s = r3_head;
				}
			}
			else
			{
				DXL1R = 0.0;
				DVL1R = 0.0;
				DXF1R = 0.0;
				DVF1R = 0.0;
			}
 

			//Density Computations
			//KL
			D2_Side = r2;
			NumberOfVehicles_Side2 = 0.0;

			while (D2_Side != NULL)
			{
				//if ( fabs ( D2_Side->x - r->x ) <= 1000)
				if (  ( r->x - D2_Side->x ) <= DistanceDensityBehind && ( r->x - D2_Side->x ) >= -DistanceDensityAhead  )
					NumberOfVehicles_Side2 = NumberOfVehicles_Side2 + 1.0;
			
				D2_Side = D2_Side->next;					
			}

			//KR
			D1_Side = r3;
			NumberOfVehicles_Side1 = 0.0;

			while (D1_Side != NULL)
			{
				//if ( fabs ( D2_Side->x - r->x ) <= 1000)
				if ((r->x - D1_Side->x) <= DistanceDensityBehind && (r->x - D1_Side->x) >= -DistanceDensityAhead)
					NumberOfVehicles_Side1 = NumberOfVehicles_Side1 + 1.0;
			
				D1_Side = D1_Side->next;
			}

			//K
			D_Total = r1;
			NumberOfVehicles_Total = 0.0;

			while (D_Total != NULL)
			{
				//if ( fabs ( D_Total->x - r->x ) <= 1000)
				if (  ( r->x - D_Total->x ) <= DistanceDensityBehind && ( r->x - D_Total->x ) >= -DistanceDensityAhead  )
					NumberOfVehicles_Total = NumberOfVehicles_Total + 1.0;
			
				D_Total = D_Total->next;
			}

			NumberOfVehicles_Total = NumberOfVehicles_Total + NumberOfVehicles_Side2;

			if (r->x >=DistanceDensityBehind && (L - r->x) >= DistanceDensityAhead)
				Distance_Density = (DistanceDensityBehind + DistanceDensityAhead);
			if (r->x >=DistanceDensityBehind && (L - r->x) < DistanceDensityAhead)
				Distance_Density = ( DistanceDensityBehind + (L - r->x) ) ;
			if (r->x < DistanceDensityBehind && (L - r->x) >= DistanceDensityAhead)
				Distance_Density = ( r->x + DistanceDensityAhead );
			if (r->x < DistanceDensityBehind && (L - r->x) < DistanceDensityAhead)
				Distance_Density = ( r->x + (L - r->x) ); 

			KL = ( NumberOfVehicles_Side2 * 1000.0) / Distance_Density; //in veh/Km.lane
			KR = ( NumberOfVehicles_Side1 * 1000.0) / Distance_Density; //in veh/Km.lane
			K = ( NumberOfVehicles_Total * 1000.0) / Distance_Density; //in veh/Km.lane
			
			//Car-Following Mode:
			//Assuming Weibull Distribution
			if ( r->Mode == 1)
			{
				//Number One: Limdep Technique
				Lambda_tmp = exp ( -1.0 * ( BetaOne_CF + BetaLCL_CF * r->LCL + BetaV_CF * r->v + BetaDXL1_CF * DXL1 + BetaDVL1_CF * DVL1 + BetaDXL2_CF * DXL2 + BetaDVL2_CF * DVL2 + BetaDXF1_CF * DXF1 + BetaDVF1_CF * DVF1 + BetaDXL1R_CF * DXL1R + BetaDVL1R_CF * DVL1R + BetaDXF1R_CF * DXF1R + BetaDVF1R_CF * DVF1R + BetaDXL1L_CF * DXL1L + BetaDVL1L_CF * DVL1L + BetaDXF1L_CF * DXF1L + BetaDVF1L_CF * DVF1L + BetaKR_CF * KR + BetaKL_CF * KL + BetaK_CF * K + BetaRightLane_CF * r->RightLaneDummy + BetaLeftLane_CF * r->LeftLaneDummy + BetaRamp_CF * r->RampDummy + BetaDistanceToRamp_CF * r->DistanceToRamp) );
				
				Hazard_ParameterA = Lambda_tmp;
				Hazard_ParameterB = 2.67;
				Hazard_tmp = Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) );
				Survival_tmp = exp (  -1.0 * pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB )  );
			
				R_Heterogeneity = K_Heterogeneity;

				Teta_tmp = ( 1.0 / K_Heterogeneity );

				Survival2_tmp = Survival_tmp * Teta_tmp;

				Hazard2_tmp = Survival2_tmp * Hazard_tmp;

				////Number Two: Bhat Technique
				//R_Heterogeneity = K_Heterogeneity;
				//Hazard_tmp = Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) );
				//Survival_tmp = exp (  -1.0 * pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB )  );
				//
				////Wrong:Gamma_tmp = ( pow ( K_Heterogeneity, R_Heterogeneity ) / double ( factorial ( int ( R_Heterogeneity - 1 ) ) ) ) * exp ( - 1.0 * K_Heterogeneity * V_tmp) * pow ( V_tmp , ( R_Heterogeneity - 1)  );
				//
				//Gamma_tmp = p->GammaHeterogeneity;
				//W_tmp = log (Gamma_tmp);//(?): What is 
				//Lambda_tmp = exp ( -1.0 * ( BetaOne_CF + BetaLCL_CF * r->LCL + BetaV_CF * r->v + BetaDXL1_CF * r->DXL1 + BetaDVL1_CF * r->DVL1 + BetaDXL2_CF * r->DXL2 + BetaDVL2_CF * r->DVL2 + BetaDXF1_CF * r->DXF1 + BetaDVF1_CF * r->DVF1 + BetaDXL1R_CF * r->DXL1R + BetaDVL1R_CF * r->DVL1R + BetaDXF1R_CF * r->DXF1R + BetaDVF1R_CF * r->DVF1R + BetaDXL1L_CF * r->DXL1L + BetaDVL1L_CF * r->DVL1L + BetaDXF1L_CF * r->DXF1L + BetaDVF1L_CF * r->DVF1L + BetaKR_CF * r->KR + BetaKL_CF * r->KL + BetaK_CF * r->K + BetaRightLane_CF * r->RightLaneDummy + BetaLeftLane_CF * r->LeftLaneDummy + BetaRamp_CF * r->RampDummy + BetaDistanceToRamp_CF * r->DistanceToRamp) + W_tmp);
				//Hazard2_tmp = Hazard_tmp * Lambda_tmp;
			}
			
			
			//Free-Flow Mode:
			//Assuming Exponential Distribution
			if ( r->Mode == 0)
			{
				//Number One: Limdep Technique
				Lambda_tmp = exp ( -1.0 * ( BetaOne_FF + BetaLCL_FF * r->LCL + BetaV_FF * r->v + BetaDXL1_FF * DXL1 + BetaDVL1_FF * DVL1 + BetaDXL2_FF * DXL2 + BetaDVL2_FF * DVL2 + BetaDXF1_FF * DXF1 + BetaDVF1_FF * DVF1 + BetaDXL1R_FF * DXL1R + BetaDVL1R_FF * DVL1R + BetaDXF1R_FF * DXF1R + BetaDVF1R_FF * DVF1R + BetaDXL1L_FF * DXL1L + BetaDVL1L_FF * DVL1L + BetaDXF1L_FF * DXF1L + BetaDVF1L_FF * DVF1L + BetaKR_FF * KR + BetaKL_FF * KL + BetaK_FF * K + BetaRightLane_FF * r->RightLaneDummy + BetaLeftLane_FF * r->LeftLaneDummy + BetaRamp_FF * r->RampDummy + BetaDistanceToRamp_FF * r->DistanceToRamp) );
				
				Hazard_ParameterA = Lambda_tmp;
				Hazard_ParameterB = 1.0;//Difference Between Exponential and Weibull
				Hazard_tmp = Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) );
				Survival_tmp = exp (  -1.0 * pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB )  );
			
				R_Heterogeneity = K_Heterogeneity;

				Teta_tmp = ( 1.0 / K_Heterogeneity );

				Survival2_tmp = Survival_tmp * Teta_tmp;

				Hazard2_tmp = Survival2_tmp * Hazard_tmp;

				////Number Two: Bhat Technique
				//R_Heterogeneity = K_Heterogeneity;
				//Hazard_ParameterB = 1.0;
				//Hazard_tmp = Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) );
				//Survival_tmp = exp (  -1.0 * pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB )  );
				//
				////Wrong:Gamma_tmp = ( pow ( K_Heterogeneity, R_Heterogeneity ) / double ( factorial ( int ( R_Heterogeneity - 1 ) ) ) ) * exp ( - 1.0 * K_Heterogeneity * V_tmp) * pow ( V_tmp , ( R_Heterogeneity - 1)  );
				//
				//Gamma_tmp = p->GammaHeterogeneity;
				//W_tmp = log (Gamma_tmp);//(?): What is v
				//Lambda_tmp = exp ( -1.0 * ( BetaOne_FF + BetaLCL_FF * r->LCL + BetaV_FF * r->v + BetaDXL1_FF * r->DXL1 + BetaDVL1_FF * r->DVL1 + BetaDXL2_FF * r->DXL2 + BetaDVL2_FF * r->DVL2 + BetaDXF1_FF * r->DXF1 + BetaDVF1_FF * r->DVF1 + BetaDXL1R_FF * r->DXL1R + BetaDVL1R_FF * r->DVL1R + BetaDXF1R_FF * r->DXF1R + BetaDVF1R_FF * r->DVF1R + BetaDXL1L_FF * r->DXL1L + BetaDVL1L_FF * r->DVL1L + BetaDXF1L_FF * r->DXF1L + BetaDVF1L_FF * r->DVF1L + BetaKR_FF * r->KR + BetaKL_FF * r->KL + BetaK_FF * r->K + BetaRightLane_FF * r->RightLaneDummy + BetaLeftLane_FF * r->LeftLaneDummy + BetaRamp_FF * r->RampDummy + BetaDistanceToRamp_FF * r->DistanceToRamp) + W_tmp);
				//Hazard2_tmp = Hazard_tmp * Lambda_tmp;
			}
			

			////Others Assuming Loglogistic Distribution
			////No Heterogeneity (Limdep Technique)
			//Lambda_tmp = exp ( -1.0 * ( BetaOne_CF + BetaLCL_CF * r->LCL + BetaV_CF * r->v + BetaDXL1_CF * r->DXL1 + BetaDVL1_CF * r->DVL1 + BetaDXL2_CF * r->DXL2 + BetaDVL2_CF * r->DVL2 + BetaDXF1_CF * r->DXF1 + BetaDVF1_CF * r->DVF1 + BetaDXL1R_CF * r->DXL1R + BetaDVL1R_CF * r->DVL1R + BetaDXF1R_CF * r->DXF1R + BetaDVF1R_CF * r->DVF1R + BetaDXL1L_CF * r->DXL1L + BetaDVL1L_CF * r->DVL1L + BetaDXF1L_CF * r->DXF1L + BetaDVF1L_CF * r->DVF1L + BetaKR_CF * r->KR + BetaKL_CF * r->KL + BetaK_CF * r->K) );
			//
			//Hazard_ParameterA = Lambda_tmp;
			//Hazard_tmp = ( ( Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) ) ) / ( 1 + pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB ) ) );
			//Survival_tmp = ( 1.0 / (  1.0 +  pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB ) ) );
		
			////R_Heterogeneity = K_Heterogeneity;

			////Teta_tmp = ( 1.0 / K_Heterogeneity );

			////Survival2_tmp = Survival_tmp * Teta_tmp;

			////Hazard2_tmp = Survival2_tmp * Hazard_tmp;


			////With Heterogeneity: Bhat Technique

			//R_Heterogeneity = K_Heterogeneity;
			//
			//Hazard_tmp = ( ( Hazard_ParameterA * Hazard_ParameterB * pow ( ( Hazard_ParameterA * r->Duration ),( Hazard_ParameterB - 1 ) ) ) / ( 1 + pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB ) ) );
			//Survival_tmp = ( 1.0 / (  1.0 +  pow ( ( Hazard_ParameterA * r->Duration ) , Hazard_ParameterB ) ) );
			//
			////Wrong:Gamma_tmp = ( pow ( K_Heterogeneity, R_Heterogeneity ) / double ( factorial ( int ( R_Heterogeneity - 1 ) ) ) ) * exp ( - 1.0 * K_Heterogeneity * V_tmp) * pow ( V_tmp , ( R_Heterogeneity - 1)  );
			//
			//Gamma_tmp = p->GammaHeterogeneity;
			//W_tmp = log (Gamma_tmp);//(?): What is v
			//Lambda_tmp = exp ( -1.0 * ( BetaOne_CF + BetaLCL_CF * r->LCL + BetaV_CF * r->v + BetaDXL1_CF * r->DXL1 + BetaDVL1_CF * r->DVL1 + BetaDXL2_CF * r->DXL2 + BetaDVL2_CF * r->DVL2 + BetaDXF1_CF * r->DXF1 + BetaDVF1_CF * r->DVF1 + BetaDXL1R_CF * r->DXL1R + BetaDVL1R_CF * r->DVL1R + BetaDXF1R_CF * r->DXF1R + BetaDVF1R_CF * r->DVF1R + BetaDXL1L_CF * r->DXL1L + BetaDVL1L_CF * r->DVL1L + BetaDXF1L_CF * r->DXF1L + BetaDVF1L_CF * r->DVF1L + BetaKR_CF * r->KR + BetaKL_CF * r->KL + BetaK_CF * r->K) + W_tmp);
			//Hazard2_tmp = Hazard_tmp * Lambda_tmp;

			//Crash Mode:
			if ( r->Mode == -1 )
			{
				Hazard2_tmp = 0.0;
			}

			if (Hazard2_tmp > -10 && Hazard2_tmp < 10)
				r->basehazard = Hazard2_tmp;
			else 
				r->basehazard = -1000;

			PercentLaneChange = mrand(102); //PercentLaneChange is between 0 and 1

			r->Hazard = Hazard2_tmp;

			if (r->next != NULL)
			{
				if(r->next->crash == 1)
				{
					r->Hazard = 10;
				}
			}

			//***************************************************************************************************
			//***************************************************************************************************
			//*************************************Mandatory Lane Changing***************************************
			//***************************************************************************************************
			//***************************************************************************************************
			r->targetrampside = 2;//initial value
			current_stream = 0;
			if (r->lane < 10)
				current_stream = 1;
			else
				current_stream = r->lane;

			if (current_stream != 1) //Not in the Main Stream
			{
				counter_path = 0;
				while (Path[r->carID][counter_path] != current_stream)
					counter_path++;

				counter_ramp = 0;
				while (Path[r->carID][counter_path] != ramp_information[counter_ramp].Ramp_Number)
					counter_ramp++;

				if (ramp_information[counter_ramp].Ramp_Type == 1)
					r->Hazard = -10;

				if (ramp_information[counter_ramp].Ramp_Type == 0)
				{
					if (r->x >= ramp_information[counter_ramp].Ramp_Location && r->x <= ramp_information[counter_ramp].Ramp_Merging_Length)
						r->Hazard = 10;
					else
						r->Hazard = -10;
				}
				if (ramp_information[counter_ramp].Ramp_Type == 2)
				{
					if (counter_path < Path[r->carID].size() - 1)
						if (r->x >= ramp_information[counter_ramp].Ramp_Location && r->x <= ramp_information[counter_ramp].Ramp_Merging_Length)
							r->Hazard = 10;
						else
						{
							r->Hazard = -10;
							r->l = 0;
						}
					else
					{
						r->Hazard = -10;
						r->l = 0;
					}
				}				
			}
			else //In the Main Stream
			{
				counter_path = 0;
				while (Path[r->carID][counter_path] != current_stream)
					counter_path++;
				
				//counter_path = counter_path + 1;
				if (counter_path == (Path[r->carID].size() - 1))
					r->Hazard = Hazard2_tmp;
				else
				{
					r->Hazard = Hazard2_tmp;
					counter_ramp = 0;
					while (Path[r->carID][counter_path + 1] != ramp_information[counter_ramp].Ramp_Number)
						counter_ramp++;

					if (ramp_information[counter_ramp].Ramp_Side == 1) // ramp on the righ hand side in the direction of travel
					{
						r->targetrampside = 1;
						if (r->lane == 0)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 1500 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 1)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 1000 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 2)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 500 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 3)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
					}
					else // ramp on the left hand side in the direction of travel
					{
						r->targetrampside = 0;
						if (r->lane == 3)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 1500 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 2)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 1000 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 1)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location - 500 && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
						if (r->lane == 0)
							if (r->x >= ramp_information[counter_ramp].Ramp_Location && r->x <= ramp_information[counter_ramp].Ramp_Location + ramp_information[counter_ramp].Ramp_Merging_Length)
								r->Hazard = 10;
					}
				}
			}

			lanechangingdecision = Lane_Decision(r, r->Hazard, r->targetrampside);
			//***************************************************************************************************
			//***************************************************************************************************
			//*************************************Mandatory Lane Changing***************************************
			//***************************************************************************************************
			//***************************************************************************************************
			if (current_stream = 1) //Not in the Main Stream
				if (r->next != NULL)
					if (r->next->crash == 1)
						r->Hazard = 10;

			if ( PercentLaneChange <= r->Hazard )
				IncentiveCriterion = 1;
			else
				IncentiveCriterion = 0;
			 

			//Safety Criterion
	 
			//--------------------------------------end of make decision here-------------------------------------

			if (IncentiveCriterion == 1 || r->l > 0)
			{
				SafetyCriterionAhead_L = Safety_Criteria_Ahead (r, q, r1_head, r2_head);
				SafetyCriterionAhead_R = Safety_Criteria_Ahead (r, s, r1_head, r3_head);
				SafetyCriterionBehind_L = Safety_Criteria_Behind (r, q, r1_head, r2_head);
				SafetyCriterionBehind_R = Safety_Criteria_Behind (r, s, r1_head, r3_head);

				/*

						   \  L20i \		  /	 L20i /
							\		\		 /		 /
							 \		 --------		/
							  \			(6)		   /
				--------------- --  --  --  -- -- ---------------------
				L0-lane 1		-->q  r2	(5)				(0)
				-------------------------------------------------------
				L1-lane 2		-->p  r1					(1)
				-------------------------------------------------------
				L2-lane 3		-->s  r3					(1)
				-------------------------------------------------------
				L3-lane 4					(3)				(2)
				--------------- --  --  --  -- -- ---------------------
							  /		   (4)		  \
							 /      ---------	   \
							/      /		 \	    \
						   / L10i /			  \ L10i \
				*/

				if (lanechangingdecision == 0)
				{
					if (( SafetyCriterionAhead_R->safety == 1) && ( SafetyCriterionBehind_R->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = s;
						destination_head = r3_head;
						destination_tail = r3;
						thirdlane = NULL;
						thirdlane_head = NULL;
						if (destination == NULL)
							origin->destlane = 3;						
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = NULL;
							thirdlane_head = NULL;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
				}

				if (lanechangingdecision == 1)
				{
					if (( SafetyCriterionAhead_R->safety > SafetyCriterionBehind_L->safety))
					{
						if ( SafetyCriterionBehind_R->safety == 1)
						{
							if (r->lanechangingflag == 0)
							{
								r->l = r->LCT; //switch lanes after LCT timesteps
								r->lanechangingflag = 1;
							}
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = r2;
							thirdlane_head = r2_head;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
					else if (( SafetyCriterionAhead_R->safety < SafetyCriterionBehind_L->safety))
					{
						if ( SafetyCriterionBehind_L->safety == 1)
						{
							if (r->lanechangingflag == 0)
							{
								r->l = r->LCT; //switch lanes after LCT timesteps
								r->lanechangingflag = 1;
							}
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = r3;
							thirdlane_head = r3_head;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
					else if (( SafetyCriterionAhead_R->safety == SafetyCriterionBehind_L->safety) && (SafetyCriterionBehind_L->safety == 1))
					{
						if ((SafetyCriterionAhead_R->acc + SafetyCriterionBehind_R->acc) >= (SafetyCriterionAhead_L->acc + SafetyCriterionBehind_L->acc))
						{							
							if (r->lanechangingflag == 0)
							{
								r->l = r->LCT; //switch lanes after LCT timesteps
								r->lanechangingflag = 1;
								r->changelane_direction = 1;
							}
							else
							{
								if (r->changelane_direction == 0)
									r->l = r->LCT;
							}
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = r2;
							thirdlane_head = r2_head;
							if (destination == NULL)
								origin->destlane = 3;
						}
						else
						{
							if (r->lanechangingflag == 0)
							{
								r->l = r->LCT; //switch lanes after LCT timesteps
								r->lanechangingflag = 1;
								r->changelane_direction = 0;
							}
							else
							{
								if (r->changelane_direction == 1)
									r->l = r->LCT;
							}
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = r3;
							thirdlane_head = r3_head;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = NULL;
							thirdlane_head = NULL;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
				}

				if (lanechangingdecision == 2)
				{
					if (( SafetyCriterionAhead_L->safety == 1) && ( SafetyCriterionBehind_L->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						else
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
						}
						origin = r;
						origin_head = r1_head;
						destination = q;
						destination_head = r2_head;
						destination_tail = r2;
						thirdlane = NULL;
						thirdlane_head = NULL;
						if (destination == NULL)
							origin->destlane = 2;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = NULL;
							thirdlane_head = NULL;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
				}

				if (lanechangingdecision == 3)
				{
					if (( SafetyCriterionAhead_R->safety == 1) && ( SafetyCriterionBehind_R->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = s;
						destination_head = r3_head;
						destination_tail = r3;
						thirdlane = r2;
						thirdlane_head = r2_head;
						if (destination == NULL)
							origin->destlane = 3;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = r2;
							thirdlane_head = r2_head;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
				}

				if (lanechangingdecision == 4)
				{
					if (( SafetyCriterionAhead_L->safety == 1) && ( SafetyCriterionBehind_L->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = q;
						destination_head = r2_head;
						destination_tail = r2;
						thirdlane = r3;
						thirdlane_head = r3_head;
						if (destination == NULL)
							origin->destlane = 2;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = r3;
							thirdlane_head = r3_head;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
				}

				if (lanechangingdecision == 5)
				{
					if (( SafetyCriterionAhead_L->safety == 1) && ( SafetyCriterionBehind_L->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = q;
						destination_head = r2_head;
						destination_tail = r2;
						thirdlane = r3;
						thirdlane_head = r3_head;
						if (destination == NULL)
							origin->destlane = 2;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = q;
							destination_head = r2_head;
							destination_tail = r2;
							thirdlane = r3;
							thirdlane_head = r3_head;
							if (destination == NULL)
								origin->destlane = 2;
						}
					}
				}

				if (lanechangingdecision == 6)
				{
					if (( SafetyCriterionAhead_R->safety == 1) && ( SafetyCriterionBehind_R->safety == 1))
					{
						if (r->lanechangingflag == 0)
						{
							r->l = r->LCT; //switch lanes after LCT timesteps
							r->lanechangingflag = 1;
						}
						
						origin = r;
						origin_head = r1_head;
						destination = s;
						destination_head = r3_head;
						destination_tail = r3;
						thirdlane = r2;
						thirdlane_head = r2_head;
						if (destination == NULL)
							origin->destlane = 3;
					}
					else
					{
						if (r->lanechangingflag == 1)
						{
							r->l = 0; //switch lanes after LCT timesteps
							r->lanechangingflag = 0;
							
							origin = r;
							origin_head = r1_head;
							destination = s;
							destination_head = r3_head;
							destination_tail = r3;
							thirdlane = r2;
							thirdlane_head = r2_head;
							if (destination == NULL)
								origin->destlane = 3;
						}
					}
				}
			}
			else
				origin = r;


			//lane change process 
			if (origin != NULL)
			{
				if (origin->l > 0) //lane switch coming up for that car
				{
					origin->l--;

					//do lane change
					if (origin->l == 0) //time to change lanes reached
					{  //change lanes
						
						//a = (int) (mrand(38) * 100); //random int between 1 and 100
						//a = 0;//test
						b=0; //using as a flag
						if (destination != NULL)
						{
							if (destination->previous == NULL)
							{// there is no car behind q
								if ( (destination->x - destination->s) > (origin->x + safeLC) )// && a < r->LCcrash)
								{//change lane
									b=1;
								}
							}
							else
							{// there is some car behind q
								if (((destination->x - destination->s) > (origin->x + safeLC)) && ((origin->x - origin->s) > (destination->previous->x + safeLC)))//&& a < r->LCcrash)
								{
									b=1;
								}
							}

							if (destination->next == NULL && b == 0)
							{
								if ( (destination->x + safeLC) < (origin->x - origin->s) )// && a < r->LCcrash)
								{//change lane
									b = 0;
									origin->lanechangingflag = 0;

									origin->changelaneinstance = 1;
									flag = 1;
									//    Ramp Case 1
									car *CopyTemp;
									CopyTemp = NULL;
									CopyTemp = CopyStream(r1);

									int compare;
									compare = CompareLinkedLists (CopyTemp, origin);
									while (compare == -1)
									{
										CopyTemp = CopyTemp->next;
										compare = CompareLinkedLists (CopyTemp , origin);
									};
									
									//  
									
									//  
									//CopyTemp->lane_prev = origin->lane;
									//CopyTemp->lane = destination->lane;
																								
									// remove CopyTemp from r1
									if (CopyTemp->previous != NULL)
									{
										//car behind CopyTemp
										CopyTemp->previous->next = CopyTemp->next;
										
										if (CopyTemp->next!=NULL)
										{
											//car in front of CopyTemp
											CopyTemp->next->previous = CopyTemp->previous;
											CopyTemp = CopyTemp->next;
										}
										else
										{
											r1_head = CopyTemp->previous;
											CopyTemp = CopyTemp->previous;
										}
									}
									else //was the first of its lane
									{
										if (CopyTemp->next!=NULL)
										{// set r->next is the r1
											CopyTemp->next->previous = NULL;
											r1 = CopyTemp->next;
											CopyTemp = CopyTemp->next;
											
											if (r1->next == NULL)
											{
												r1_head = r1;
											}
										}
										else
										{
											r1=NULL;
											r1_head = NULL;
											CopyTemp = NULL;
										}
									}
									//  
									// insert r into r2, q<>NULL now
																	
									// 
									if (origin->lane < 10 || (origin->lane >= 10 && destination->lane < 10))
									{
										if (origin->x >= VSLStartLocation && origin->x < VSLEndLocation)
											origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//(r->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//DriversSpeedLimit(DesiredSpeedLimit_temp, Compliance);
										else										
											origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeed) + DesiredSpeed;
									}
									// 

									origin->next = NULL;
									origin->previous = destination;
									origin->lane_prev = origin->lane;
									origin->lane = destination->lane;

									if (destination->previous!=NULL)
									{
										destination->next = origin;
										destination_head = origin;
									}
									else
									{
										destination->next = origin;
										destination_tail = destination;
										destination_head = origin ;
									}
									

									//  
									car *Tempr2q;
									Tempr2q = NULL;
									while (destination->previous != NULL)
									{
										destination = destination->previous;
									}
									Tempr2q = CopyStream(destination);
									destination_tail = NULL;
									destination_tail = CopyStream(Tempr2q);
																	
									car *Tempr2_headq;
									Tempr2_headq= NULL;
									Tempr2_headq = destination_tail;
									while (Tempr2_headq->next != NULL)
									{
										Tempr2_headq = Tempr2_headq->next;
									}
									destination_head = Tempr2_headq;

									
									if (r2 != NULL)
									{
										if (destination_tail->lane == r2->lane)
										{
											r2 = destination_tail;
											r2_head = destination_head;
										}
									}
									if (r3 != NULL)
									{
										if (destination_tail->lane == r3->lane)
										{
											if (r3->lane == 10)
											{
												r10 = destination_tail;
												r10_head = destination_head;
											}
											if (r3->lane == 11)
											{
												r11 = destination_tail;
												r11_head = destination_head;
											}
											if (r3->lane == 12)
											{
												r12 = destination_tail;
												r12_head = destination_head;
											}
											if (r3->lane == 13)
											{
												r13 = destination_tail;
												r13_head = destination_head;
											}
											if (r3->lane == 14)
											{
												r14 = destination_tail;
												r14_head = destination_head;
											}
											if (r3->lane == 15)
											{
												r15 = destination_tail;
												r15_head = destination_head;
											}
											if (r3->lane == 16)
											{
												r16 = destination_tail;
												r16_head = destination_head;
											}
											if (r3->lane == 17)
											{
												r17 = destination_tail;
												r17_head = destination_head;
											}
											if (r3->lane == 18)
											{
												r18 = destination_tail;
												r18_head = destination_head;
											}											
										}
									}

									//  

									//  
									if (CopyTemp != NULL)
									{
										while (CopyTemp->previous != NULL)
										{
											CopyTemp = CopyTemp->previous;
										};
										r1 = CopyStream(CopyTemp);
										
										if (r1->next == NULL)
										{
											r1_head = r1;
										}
									}
									else
									{
										r1 = NULL;
										r1_head = NULL;
									};

									if (r1 != NULL)
									{
										car *tempupdate;
										tempupdate = r1;
										while (tempupdate->next != NULL)
										{
											tempupdate = tempupdate->next;
										}
										r1_head = tempupdate;
									}
									//  
								}
							}
							
							origin->lanechangingflag = 0;

							//lane change
							if (b==1)
							{// lane change
								flag = 1;
								origin->changelaneinstance = 1;
								//   Ramp Case 1
								car *CopyTemp;
								CopyTemp = NULL;
								CopyTemp = CopyStream(r1);

								int compare;
								compare = CompareLinkedLists (CopyTemp, origin);
								while (compare == -1)
								{
									CopyTemp = CopyTemp->next;
									compare = CompareLinkedLists (CopyTemp , origin);
								};
								
								//  
										
								//  
								//CopyTemp->lane_prev = origin->lane;
								//CopyTemp->lane = destination->lane;
																							
								// remove CopyTemp from r1
								if (CopyTemp->previous != NULL)
								{
									//car behind CopyTemp
									CopyTemp->previous->next = CopyTemp->next;
									
									if (CopyTemp->next!=NULL)
									{
										//car in front of CopyTemp
										CopyTemp->next->previous = CopyTemp->previous;
										CopyTemp = CopyTemp->next;
									}
									else
									{
										r1_head = CopyTemp->previous;
										CopyTemp = CopyTemp->previous;
									}
								}
								else //was the first of its lane
								{
									if (CopyTemp->next!=NULL)
									{// set r->next is the r1
										CopyTemp->next->previous = NULL;
										r1 = CopyTemp->next;
										CopyTemp = CopyTemp->next;
										
										if (r1->next == NULL)
										{
											r1_head = r1;
										}
									}
									else
									{
										r1=NULL;
										r1_head = NULL;
										CopyTemp = NULL;
									}
								}
								// 

								// insert r into r2, q<>NULL now
																
								if (origin->lane < 10 || (origin->lane >= 10 && destination->lane < 10))
								{
									if (origin->x >= VSLStartLocation && origin->x < VSLEndLocation)
										origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//(r->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//DriversSpeedLimit(DesiredSpeedLimit_temp, Compliance);
									else										
										origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeed) + DesiredSpeed;
								}

								origin->next = destination;
								origin->previous = destination->previous;
								origin->lane_prev = origin->lane;
								origin->lane = destination->lane;

								if (destination->previous!=NULL)
								{
									destination->previous->next = origin;
								}
								else
								{
									destination->previous = origin;
									destination_tail = origin;
									
									if (destination_tail->next == NULL)
									{
										destination_head = destination_tail ;
									}
								}
								
								//  
								car *Tempr2q;
								Tempr2q = NULL;
								while (destination->previous != NULL)
								{
									destination = destination->previous;
								}
								Tempr2q = CopyStream(destination);
								destination_tail = NULL;
								destination_tail = CopyStream(Tempr2q);

								car *Tempr2_headq;
								Tempr2_headq= NULL;
								Tempr2_headq = destination_tail;
								while (Tempr2_headq->next != NULL)
								{
									Tempr2_headq = Tempr2_headq->next;
								}
								destination_head = Tempr2_headq;

								if (r2 != NULL)
								{
									if (destination_tail->lane == r2->lane)
									{
										r2 = destination_tail;
										r2_head = destination_head;
									}
								}
								if (r3 != NULL)
								{
									if (destination_tail->lane == r3->lane)
									{
										if (r3->lane == 10)
										{
											r10 = destination_tail;
											r10_head = destination_head;
										}
										if (r3->lane == 11)
										{
											r11 = destination_tail;
											r11_head = destination_head;
										}
										if (r3->lane == 12)
										{
											r12 = destination_tail;
											r12_head = destination_head;
										}
										if (r3->lane == 13)
										{
											r13 = destination_tail;
											r13_head = destination_head;
										}
										if (r3->lane == 14)
										{
											r14 = destination_tail;
											r14_head = destination_head;
										}
										if (r3->lane == 15)
										{
											r15 = destination_tail;
											r15_head = destination_head;
										}
										if (r3->lane == 16)
										{
											r16 = destination_tail;
											r16_head = destination_head;
										}
										if (r3->lane == 17)
										{
											r17 = destination_tail;
											r17_head = destination_head;
										}
										if (r3->lane == 18)
										{
											r18 = destination_tail;
											r18_head = destination_head;
										}
									}
								}							

								//  

								//  
								if (CopyTemp != NULL)
								{
									while (CopyTemp->previous != NULL)
									{
										CopyTemp = CopyTemp->previous;
									};
									r1 = CopyStream(CopyTemp);
									
									if (r1->next == NULL)
									{
										r1_head = r1;
									}
								}
								else
								{
									r1 = NULL;
									r1_head = NULL;
								};

								if (r1 != NULL)
								{
									car *tempupdate;
									tempupdate = r1;
									while (tempupdate->next != NULL)
									{
										tempupdate = tempupdate->next;
									}
									r1_head = tempupdate;
								}
								//  
							}
						}
						else
						{
							flag = 1;
							origin->changelaneinstance = 1;
							origin->lanechangingflag = 0;
							//    Ramp Case 1
							car *CopyTemp;
							CopyTemp = NULL;
							CopyTemp = CopyStream(r1);

							int compare;
							compare = CompareLinkedLists (CopyTemp, origin);
							while (compare == -1)
							{
								CopyTemp = CopyTemp->next;
								compare = CompareLinkedLists (CopyTemp , origin);
							};
							
							//  
									
							//  
							//CopyTemp->lane_prev = origin->lane;
							//CopyTemp->lane = destination->lane;
																						
							// remove CopyTemp from r1
							if (CopyTemp->previous != NULL)
							{
								//car behind CopyTemp
								CopyTemp->previous->next = CopyTemp->next;
								
								if (CopyTemp->next!=NULL)
								{
									//car in front of CopyTemp
									CopyTemp->next->previous = CopyTemp->previous;
									CopyTemp = CopyTemp->next;
								}
								else
								{
									r1_head = CopyTemp->previous;
									CopyTemp = CopyTemp->previous;
								}
							}
							else //was the first of its lane
							{
								if (CopyTemp->next!=NULL)
								{// set r->next is the r1
									CopyTemp->next->previous = NULL;
									r1 = CopyTemp->next;
									CopyTemp = CopyTemp->next;
									
									if (r1->next == NULL)
									{
										r1_head = r1;
									}
								}
								else
								{
									r1=NULL;
									r1_head = NULL;
									CopyTemp = NULL;
								}
							}
							//  

							// insert r into r2, q<>NULL now
															
							if (origin->lane < 10)
							{
								if (origin->x >= VSLStartLocation && origin->x < VSLEndLocation)
									origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//(r->overspeedingrate * DesiredSpeedLimit_temp) + DesiredSpeedLimit_temp;//DriversSpeedLimit(DesiredSpeedLimit_temp, Compliance);
								else										
									origin->Vd = (origin->Compliance * origin->overspeedingrate * DesiredSpeed) + DesiredSpeed;
							}
							// 

							//  
							if (CopyTemp != NULL)
							{
								while (CopyTemp->previous != NULL)
								{
									CopyTemp = CopyTemp->previous;
								};
								r1 = CopyStream(CopyTemp);
								
								if (r1->next == NULL)
								{
									r1_head = r1;
								}
							}
							else
							{
								r1 = NULL;
								r1_head = NULL;
							};

							if (r1 != NULL)
							{
								car *tempupdate;
								tempupdate = r1;
								while (tempupdate->next != NULL)
								{
									tempupdate = tempupdate->next;
								}
								r1_head = tempupdate;
							}
							//  

							if (origin->destlane == 2)
							{
								origin->next = NULL;
								origin->previous = NULL;
								origin->lane_prev = origin->lane;
								origin->lane = (origin->lane - 1);								
								r2 = r2_head = origin;
							}
							if (origin->destlane == 3)
							{
								origin->next = NULL;
								origin->previous = NULL;
								origin->lane_prev = origin->lane;
								for (int o = 0; o < ramp_information.size(); o++)
								{
									if (origin->x > ramp_information[o].Ramp_Location && origin->x < (ramp_information[o].Ramp_Location + ramp_information[o].Ramp_Merging_Length))
									{
										origin->lane = ramp_information[o].Ramp_Number;
										break;
									}
								}
								r3 = r3_head = origin;
								if (r3->lane == 10)
								{
									r10 = origin;
									r10_head = origin;
								}
								if (r3->lane == 11)
								{
									r11 = origin;
									r11_head = origin;
								}
								if (r3->lane == 12)
								{
									r12 = origin;
									r12_head = origin;
								}
								if (r3->lane == 13)
								{
									r13 = origin;
									r13_head = origin;
								}
								if (r3->lane == 14)
								{
									r14 = origin;
									r14_head = origin;
								}
								if (r3->lane == 15)
								{
									r15 = origin;
									r15_head = origin;
								}
								if (r3->lane == 16)
								{
									r16 = origin;
									r16_head = origin;
								}
								if (r3->lane == 17)
								{
									r17 = origin;
									r17_head = origin;
								}
								if (r3->lane == 18)
								{
									r18 = origin;
									r18_head = origin;
								}								
							}
						}
					}
					else
					{
						while (origin->previous != NULL)
							origin = origin->previous;
						r1 = origin;
						while (origin->next != NULL)
							origin = origin->next;
						r1_head = origin;
					}
				}
			}
		}
	}
	
	Lane_Changing_temp_RML->LtmpRML1 = r1;
	Lane_Changing_temp_RML->LtmpRML2 = r2;	
	Lane_Changing_temp_RML->LtmpRML10 = r10;
	Lane_Changing_temp_RML->LtmpRML11 = r11;
	Lane_Changing_temp_RML->LtmpRML12 = r12;
	Lane_Changing_temp_RML->LtmpRML13 = r13;
	Lane_Changing_temp_RML->LtmpRML14 = r14;
	Lane_Changing_temp_RML->LtmpRML15 = r15;
	Lane_Changing_temp_RML->LtmpRML16 = r16;
	Lane_Changing_temp_RML->LtmpRML17 = r17;
	Lane_Changing_temp_RML->LtmpRML18 = r18;
	

	Lane_Changing_temp_RML->LtmpRML1_head = r1_head;
	Lane_Changing_temp_RML->LtmpRML2_head = r2_head;	
	Lane_Changing_temp_RML->LtmpRML10_head = r10_head;
	Lane_Changing_temp_RML->LtmpRML11_head = r11_head;
	Lane_Changing_temp_RML->LtmpRML12_head = r12_head;
	Lane_Changing_temp_RML->LtmpRML13_head = r13_head;
	Lane_Changing_temp_RML->LtmpRML14_head = r14_head;
	Lane_Changing_temp_RML->LtmpRML15_head = r15_head;
	Lane_Changing_temp_RML->LtmpRML16_head = r16_head;
	Lane_Changing_temp_RML->LtmpRML17_head = r17_head;
	Lane_Changing_temp_RML->LtmpRML18_head = r18_head;
	
	


	
	/*destination = NULL;
	destination_tail = NULL;
	destination_head = NULL;
	origin = NULL;
	r1 = NULL;
	r2 = NULL;
	r3 = NULL;
	r10 = NULL;
	r1 = NULL;
	r12 = NULL;
	r13 = NULL;
	r14 = NULL;
	r15 = NULL;
	r16 = NULL;
	r17 = NULL;
	r18 = NULL;
	r19 = NULL;
	r20 = NULL;
	r1_head = NULL;
	r2_head = NULL;
	r3_head = NULL;
	r10_head = NULL;
	r1_head = NULL;
	r12_head = NULL;
	r13_head = NULL;
	r14_head = NULL;
	r15_head = NULL;
	r16_head = NULL;
	r17_head = NULL;
	r18_head = NULL;
	r19_head = NULL;
	r20_head = NULL;*/

	return (Lane_Changing_temp_RML);
	free(Lane_Changing_temp_RML);

};
//--------------------------------------End of Lane Changing-------------------------------------


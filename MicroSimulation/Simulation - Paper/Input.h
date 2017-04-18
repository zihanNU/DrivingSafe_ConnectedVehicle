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
#include "ControlPanel.h"
#include "Reaction.h"


struct insert_output
{
	car * lane_head;
	car * lane_tail;
	car * queue_end;
	car * queue_start;
};

insert_output * Insert_Vehicle (car * r, car * r1, car * r1_head, car * Lqe, car * Lqs)//struct insert_output * Insert_Vehicle (car * r, car * Lqe, car * Lqs)
{
	float f = 0;
	insert_output * Insert_Vehicle_temp;
	Insert_Vehicle_temp = (insert_output *) malloc(sizeof(Insert_Vehicle_temp));
	Insert_Vehicle_temp->queue_end = NULL;
	Insert_Vehicle_temp->queue_start = NULL;
	Insert_Vehicle_temp->lane_head = NULL;
	Insert_Vehicle_temp->lane_tail = NULL;

	if ( r1 == NULL)
	{
		if (Lqe == NULL)
		{// insert car to lane
			r->next = r->previous = NULL;
			r1 = r;

			r1_head = r1;
		}
		else
		{//insert car to queue
			Lqe->previous = r;
			r->next = Lqe;
			r->previous = NULL;
			Lqe = r;
		}	
	}
	else
	{
		if ( (r1->x - r1->s) < safeD) //car ahead didnt fully enter yet
		{	//insert car to queue;
			if (Lqe !=NULL)
			{
				Lqe->previous = r;
				r->next = Lqe;
				r->previous = NULL;
				Lqe = r;
			}
			else
			{
				r->previous = NULL;
				r->next = NULL;
				Lqs = Lqe = r;
			}
		}
		else  //insert car into lane
		{
			//ensure car has safe starting velocity
			//f = (p->decc * p->next->v * p->next->v / p->next->decc) - (2.0 * p->decc * (p->next->x - p->next->s));
			//f = sqrt(f);

			f = min(Average_Starting_Speed[r->lane], r->v);//r->next->v;//r->decc * 0.1 + sqrt ( r->decc * r->decc * 0.1 * 0.1 - (  ( r->next->v * r->next->v) / r->next->decc ) );

			//f = startV;

			if (f < r->v)
			{
				r->v = f;
				 
				if (r->v <=0)
				{
					r->v = 0.001;
				};
				 
			}
			r->next = r1;
			r->previous = NULL;
			r1->previous = r;
			r1 = r;
		}
	}

	car *tempq;
	tempq = r1;
	if (tempq != NULL)
	{
		while (tempq->next != NULL)
			tempq = tempq->next;
	}
	r1_head = tempq;

	if (Lqe != NULL)
	{
		tempq = NULL;
		tempq = Lqe;
		while (tempq->next != NULL)
			tempq = tempq->next;
		Lqs = tempq;
	}

	Insert_Vehicle_temp->lane_head = r1_head;
	Insert_Vehicle_temp->lane_tail = r1;
	Insert_Vehicle_temp->queue_end = Lqe;
	Insert_Vehicle_temp->queue_start = Lqs;
	

	/*
	if (Lqe == NULL)
	{
		r->next = r->previous = NULL;
		Lqe = Lqs = r;
	}
	else
	{
		Lqe->previous = r;
		r->next = Lqe;
		r->previous = NULL;
		Lqe = r;
		
		car *tempq;
		tempq = Lqe;
		while (tempq->next != NULL)
			tempq = tempq->next;
		Lqs = tempq;
	}

	Insert_Vehicle_temp->lane_head = r1_head;
	Insert_Vehicle_temp->lane_tail = r1;
	Insert_Vehicle_temp->queue_end = Lqe;
	Insert_Vehicle_temp->queue_start = Lqs;*/


	return(Insert_Vehicle_temp);
};

insert_output * Insert_Vehicle_II (car * r1, car * r1_head, car * Lqe, car * Lqs)
{
	car * r = NULL;
	float f = 0;
	insert_output * Insert_Vehicle_temp;
	Insert_Vehicle_temp = (insert_output *) malloc(sizeof(Insert_Vehicle_temp));
	Insert_Vehicle_temp->queue_end = NULL;
	Insert_Vehicle_temp->queue_start = NULL;
	Insert_Vehicle_temp->lane_head = NULL;
	Insert_Vehicle_temp->lane_tail = NULL;

	
	/*r = Lqs;
	
	//insert car to beginning of its lane
	if ( r1 == NULL) //lane is now empty (you never know!)
	{
		//take out the p
		if (Lqs->previous != NULL)
		{		
			Lqs = Lqs->previous;
			Lqs->next = NULL;
			car *tempq;
			tempq = Lqs;
			while (tempq->previous != NULL)
				tempq = tempq->previous;
			Lqe = tempq;
		}
		else
		{
			Lqs = Lqe = NULL;
		}

		r->departure = TIME;
		r->dt_prev = r->departure;
		r->previous = NULL;
		r1 = r;
		
		r1_head = r1;
	}
	else //L0 not empty
	{
		if ( (r1->x - r1->s) > safeD) //car ahead is clear
		{
			//take out the p
			if (Lqs->previous != NULL)
			{
				Lqs = Lqs->previous;
				Lqs->next = NULL;
				car *tempq;
				tempq = Lqs;
				while (tempq->previous != NULL)
					tempq = tempq->previous;
				Lqe = tempq;
			}
			else
			{
				Lqs = Lqe = NULL;
			}					

			r->departure = TIME;
			r->dt_prev = r->departure;
			r->next = r1;
			r->previous = NULL;

			r1->previous = r;

			//ensure car has safe starting velocity
			//f = (p->decc * p->next->v * p->next->v / p->next->decc) - (2.0 * p->decc * (p->next->x - p->next->s));
			//f = sqrt(f);
			//f = (r->next->x - r->x)/40;
			f = r->decc * 0.1 + sqrt ( r->decc * r->decc * 0.1 * 0.1 - (  ( r->next->v * r->next->v) / r->next->decc ) );
			//f = sqrt(2*sqrt(r->decc*r->decc)*(r->next->x - r->x + ((r->next->v * r->next->v)/(2 * r->next->decc))));
			//f= f*0.5;
			//f = startV;
			//if (L0->next != NULL)
			//	f = L0->next->v;

			if (f < r->v)
			{
				r->v = f;
				// added by ART 07/09/11
					if (r->v <=0)
					{
						r->v = 0.001;
					};
					// end of added by ART 07/09/11
			}
				

			car *tempL00;
			tempL00 = r;
			while (tempL00->next != NULL)
				tempL00 = tempL00->next;
			r1_head = tempL00;

			car *tempL0;
			tempL0 = r1_head;
			while (tempL0->previous != NULL)
				tempL0 = tempL0->previous;
			r1_head = tempL0;
		}
	}

	Insert_Vehicle_temp->lane_head = r1_head;
	Insert_Vehicle_temp->lane_tail = r1;
	Insert_Vehicle_temp->queue_end = Lqe;
	Insert_Vehicle_temp->queue_start = Lqs;*/


	
	r = Lqs;

	//insert car to beginning of its lane
	if ( r1 == NULL) //lane is now empty (you never know!)
	{
		//take out the p
		if (Lqs->previous != NULL)
		{
			Lqs = Lqs->previous;
			Lqs->next = NULL;
		}
		else
		{
			Lqs = Lqe = NULL;
		}

		r->departure = TIME;
		r->dt_prev = r->departure;
		r->previous = NULL;
		r1 = r;
		
		r1_head = r1;
	}
	else //r1 not empty
	{
		if ( (r1->x - r1->s) > safeD) //car ahead is clear
		{
			//take out the p
			if (Lqs->previous != NULL)
			{
				Lqs = Lqs->previous;
				Lqs->next = NULL;
			}
			else
			{
				Lqs = Lqe = NULL;
			}

			r->next = r1;
			r->previous = NULL;

			r1->previous = r;
			//r1 = r;

			r->departure = TIME;
			r->dt_prev = r->departure;

			//ensure car has safe starting velocity
			//f = (r->decc * r->next->v * r->next->v / r->next->decc) - (2.0 * r->decc * (r->next->x - r->next->s));
			//f = sqrt(f);
			
			f = min(Average_Starting_Speed[r->lane], r->v);//r->next->v;//r->decc * 0.1 + sqrt ( r->decc * r->decc * 0.1 * 0.1 - (  ( r->next->v * r->next->v) / r->next->decc ) );
			
			//f = startV;

			if (f < r->v)
			{
				r->v = f;
			 
					if (r->v <=0)
					{
						r->v = 0.001;
					};
					 
			}
		}
	}

	car *tempq;
	tempq = r1;
	while (tempq->next != NULL)
		tempq = tempq->next;
	r1_head = tempq;

	if (Lqs != NULL)
	{
		tempq = NULL;
		tempq = Lqs;
		while (tempq->previous != NULL)
			tempq = tempq->previous;
		Lqe = tempq;
	}

	Insert_Vehicle_temp->lane_head = r1_head;
	Insert_Vehicle_temp->lane_tail = r1;
	Insert_Vehicle_temp->queue_end = Lqe;
	Insert_Vehicle_temp->queue_start = Lqs;
	
	return (Insert_Vehicle_temp);
};
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************

insert_output * Insert_Vehicle_Ramp (car * r, car * r1, car * r1_head, car * Lqe, car * Lqs, short lane)//struct insert_output * Insert_Vehicle (car * r, car * Lqe, car * Lqs)
{
	float f = 0;
	insert_output * Insert_Vehicle_temp;
	Insert_Vehicle_temp = (insert_output *) malloc(sizeof(Insert_Vehicle_temp));
	Insert_Vehicle_temp->queue_end = NULL;
	Insert_Vehicle_temp->queue_start = NULL;
	Insert_Vehicle_temp->lane_head = NULL;
	Insert_Vehicle_temp->lane_tail = NULL;

	short counter_ramp = 0;
	while (lane != ramp_information[counter_ramp].Ramp_Number)
		counter_ramp++;

	short i = 0;
	while (metering_info[i].ramp != lane)
		i++;

	
	if ( r1 == NULL)
	{
		if (Lqe == NULL && TIME - metering_depart[i] >= metering_interarrivaltime[i])
		{// insert car to lane
			r->next = r->previous = NULL;
			r1 = r;

			r1_head = r1;
			metering_depart[i] = TIME;
		}
		else
		{//insert car to queue
			Lqe->previous = r;
			r->next = Lqe;
			r->previous = NULL;
			Lqe = r;
		}	
	}
	else
	{
		if ((r1->x - r1->s - ramp_information[counter_ramp].Ramp_StartPoint) < safeD) //car ahead didnt fully enter yet
		{	//insert car to queue;
			if (Lqe !=NULL)
			{
				Lqe->previous = r;
				r->next = Lqe;
				r->previous = NULL;
				Lqe = r;
			}
			else
			{
				r->previous = NULL;
				r->next = NULL;
				Lqs = Lqe = r;
			}
		}
		else  //insert car into lane
		{
			if (TIME - metering_depart[i] >= metering_interarrivaltime[i])
			{
				//ensure car has safe starting velocity
				//f = (p->decc * p->next->v * p->next->v / p->next->decc) - (2.0 * p->decc * (p->next->x - p->next->s));
				//f = sqrt(f);

				f = min(Average_Starting_Speed[r->lane], r->v);//r->decc * 0.1 + sqrt ( r->decc * r->decc * 0.1 * 0.1 - (  ( r->next->v * r->next->v) / r->next->decc ) );

				//f = startV;

				if (f < r->v)
				{
					r->v = f;
					// added by ART 07/09/11
					if (r->v <=0)
					{
						r->v = 0.001;
					};
					// end of added by ART 07/09/11
				}
				r->next = r1;
				r->previous = NULL;
				r1->previous = r;
				r1 = r;
				metering_depart[i] = TIME;
			}
			else
			{
				if (Lqe !=NULL)
				{
					Lqe->previous = r;
					r->next = Lqe;
					r->previous = NULL;
					Lqe = r;
				}
				else
				{
					r->previous = NULL;
					r->next = NULL;
					Lqs = Lqe = r;
				}
			}
		}
	}

	car *tempq;
	tempq = r1;
	while (tempq->next != NULL)
		tempq = tempq->next;
	r1_head = tempq;

	Insert_Vehicle_temp->lane_head = r1_head;
	Insert_Vehicle_temp->lane_tail = r1;
	Insert_Vehicle_temp->queue_end = Lqe;
	Insert_Vehicle_temp->queue_start = Lqs;
	

	/*
	if (Lqe == NULL)
	{
		r->next = r->previous = NULL;
		Lqe = Lqs = r;
	}
	else
	{
		Lqe->previous = r;
		r->next = Lqe;
		r->previous = NULL;
		Lqe = r;
		
		car *tempq;
		tempq = Lqe;
		while (tempq->next != NULL)
			tempq = tempq->next;
		Lqs = tempq;
	}

	Insert_Vehicle_temp->lane_head = r1_head;
	Insert_Vehicle_temp->lane_tail = r1;
	Insert_Vehicle_temp->queue_end = Lqe;
	Insert_Vehicle_temp->queue_start = Lqs;*/


	return(Insert_Vehicle_temp);
};

insert_output * Insert_Vehicle_Ramp_II (car * r1, car * r1_head, car * Lqe, car * Lqs, short lane)
{
	car * r = NULL;
	float f = 0;
	insert_output * Insert_Vehicle_temp;
	Insert_Vehicle_temp = (insert_output *) malloc(sizeof(Insert_Vehicle_temp));
	Insert_Vehicle_temp->queue_end = NULL;
	Insert_Vehicle_temp->queue_start = NULL;
	Insert_Vehicle_temp->lane_head = NULL;
	Insert_Vehicle_temp->lane_tail = NULL;

	short counter_ramp = 0;
	while (lane != ramp_information[counter_ramp].Ramp_Number)
		counter_ramp++;
	
	short i = 0;
	while (metering_info[i].ramp != lane)
		i++;
	
	/*r = Lqs;
	
	//insert car to beginning of its lane
	if ( r1 == NULL) //lane is now empty (you never know!)
	{
		//take out the p
		if (Lqs->previous != NULL)
		{		
			Lqs = Lqs->previous;
			Lqs->next = NULL;
			car *tempq;
			tempq = Lqs;
			while (tempq->previous != NULL)
				tempq = tempq->previous;
			Lqe = tempq;
		}
		else
		{
			Lqs = Lqe = NULL;
		}

		r->departure = TIME;
		r->dt_prev = r->departure;
		r->previous = NULL;
		r1 = r;
		
		r1_head = r1;
	}
	else //L0 not empty
	{
		if ( (r1->x - r1->s - ramp_information[counter_ramp].Ramp_StartPoint) > safeD) //car ahead is clear
		{
			//take out the p
			if (Lqs->previous != NULL)
			{
				Lqs = Lqs->previous;
				Lqs->next = NULL;
				car *tempq;
				tempq = Lqs;
				while (tempq->previous != NULL)
					tempq = tempq->previous;
				Lqe = tempq;
			}
			else
			{
				Lqs = Lqe = NULL;
			}					

			r->departure = TIME;
			r->dt_prev = r->departure;
			r->next = r1;
			r->previous = NULL;

			r1->previous = r;

			//ensure car has safe starting velocity
			//f = (p->decc * p->next->v * p->next->v / p->next->decc) - (2.0 * p->decc * (p->next->x - p->next->s));
			//f = sqrt(f);
			//f = (r->next->x - r->x)/40;
			f = r->decc * 0.1 + sqrt ( r->decc * r->decc * 0.1 * 0.1 - (  ( r->next->v * r->next->v) / r->next->decc ) );
			//f = sqrt(2*sqrt(r->decc*r->decc)*(r->next->x - r->x + ((r->next->v * r->next->v)/(2 * r->next->decc))));
			//f= f*0.5;
			//f = startV;
			//if (L0->next != NULL)
			//	f = L0->next->v;

			if (f < r->v)
			{
				r->v = f;
				// added by ART 07/09/11
					if (r->v <=0)
					{
						r->v = 0.001;
					};
					// end of added by ART 07/09/11
			}
				

			car *tempL00;
			tempL00 = r;
			while (tempL00->next != NULL)
				tempL00 = tempL00->next;
			r1_head = tempL00;

			car *tempL0;
			tempL0 = r1_head;
			while (tempL0->previous != NULL)
				tempL0 = tempL0->previous;
			r1_head = tempL0;
		}
	}

	Insert_Vehicle_temp->lane_head = r1_head;
	Insert_Vehicle_temp->lane_tail = r1;
	Insert_Vehicle_temp->queue_end = Lqe;
	Insert_Vehicle_temp->queue_start = Lqs;*/


	
	r = Lqs;

	if (TIME - metering_depart[i] >= metering_interarrivaltime[i])
	{
		//insert car to beginning of its lane
		if ( r1 == NULL) //lane is now empty (you never know!)
		{
			//take out the p
			if (Lqs->previous != NULL)
			{
				Lqs = Lqs->previous;
				Lqs->next = NULL;
			}
			else
			{
				Lqs = Lqe = NULL;
			}

			r->departure = TIME;
			r->dt_prev = r->departure;
			r->previous = NULL;
			r1 = r;
			
			r1_head = r1;
			metering_depart[i] = TIME;
		}
		else //r1 not empty
		{
			if ( (r1->x - r1->s - ramp_information[counter_ramp].Ramp_StartPoint) > safeD) //car ahead is clear
			{
				//take out the p
				if (Lqs->previous != NULL)
				{
					Lqs = Lqs->previous;
					Lqs->next = NULL;
				}
				else
				{
					Lqs = Lqe = NULL;
				}

				r->next = r1;
				r->previous = NULL;

				r1->previous = r;
				//r1 = r;

				r->departure = TIME;
				r->dt_prev = r->departure;
				metering_depart[i] = TIME;

				//ensure car has safe starting velocity
				//f = (r->decc * r->next->v * r->next->v / r->next->decc) - (2.0 * r->decc * (r->next->x - r->next->s));
				//f = sqrt(f);
				
				f = min(Average_Starting_Speed[r->lane], r->v);//r->decc * 0.1 + sqrt ( r->decc * r->decc * 0.1 * 0.1 - (  ( r->next->v * r->next->v) / r->next->decc ) );
				
				//f = startV;

				if (f < r->v)
				{
					r->v = f;
					 
						if (r->v <=0)
						{
							r->v = 0.001;
						};
						 
				}
			}
		}
	}

	car *tempq;
	tempq = r1;
	if (tempq != NULL)
	{
		while (tempq->next != NULL)
			tempq = tempq->next;
		r1_head = tempq;
	}
	else
		r1_head = NULL;

	Insert_Vehicle_temp->lane_head = r1_head;
	Insert_Vehicle_temp->lane_tail = r1;
	Insert_Vehicle_temp->queue_end = Lqe;
	Insert_Vehicle_temp->queue_start = Lqs;
	
	return (Insert_Vehicle_temp);
};
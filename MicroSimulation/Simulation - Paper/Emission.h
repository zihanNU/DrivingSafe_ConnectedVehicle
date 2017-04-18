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

using namespace std;

short Mode, Mode_Previous;
double Energy;
vector<double> emission_input;

/*double emission_rate[100];

emission_rate[100] = 43853.5;
emission_rate[1] = 40569.5;
emission_rate[11] = 63794.6;
emission_rate[12] = 88088.8;
emission_rate[13] = 122424;
emission_rate[14] = 154689;
emission_rate[15] = 184463;
emission_rate[16] = 222795;
emission_rate[21] = 86789;
emission_rate[22] = 98790.1;
emission_rate[23] = 120321;
emission_rate[24] = 154363;
emission_rate[25] = 205999;
emission_rate[27] = 271565;
emission_rate[28] = 366049;
emission_rate[29] = 501490;
emission_rate[30] = 629763;
emission_rate[33] = 123648;
emission_rate[35] = 198272;
emission_rate[37] = 258282;
emission_rate[38] = 336786;
emission_rate[39] = 448593;
emission_rate[40] = 571849;*/

int g_Readnum(FILE* f)
// read an integer from the current pointer of the file, skip all spaces
{
   char ch, buf[ 32 ];
   int i = 0;
   int flag = 1;
   /* returns -1 if end of file is reached */

   while(true)
   {
    ch = getc( f );
      if( ch == EOF ) return -1;
		if (isdigit(ch))
			break;
		if (ch == '-')
		  flag = -1;
		else
	      flag = 1;
   };
      if( ch == EOF ) return -1;
   while( isdigit( ch )) {
      buf[ i++ ] = ch;
      ch = fgetc( f );
   }
   buf[ i ] = 0;


   return atoi( buf ) * flag;

}

double g_Readdouble(FILE *f)
/* read a floating point number from the current pointer of the file,
   skip all spaces
*/
{
   char ch, buf[ 32 ];
   int i = 0;
   int flag = 1;

   /* returns -1 if end of file is reached */

   while(true)
   {
    ch = getc( f );
      if( ch == EOF ) return -1;
		if (isdigit(ch))
			break;

		if (ch == '-')
		  flag = -1;
		else
	      flag = 1;
			
   };
      if( ch == EOF ) return -1;
   while( isdigit( ch ) || ch == '.' ) {
      buf[ i++ ] = ch;
      ch = fgetc( f );
   }
   buf[ i ] = 0;

/* atof function converts a character string (char *) into a doubleing
   pointer equivalent, and if the string is not a floting point number,
   a zero will be return.
*/

   return atof( buf ) * flag;
}

int read_emission_file(int a)
{
	double temp;
	FILE *st;
//	st = fopen("C:\\Users\\Administrator\\Desktop\\Alireza Files\\ISTTT Paper Simulation Codes\\Simulation (07.27.14) - gap lanechanging\\I-O Files\\CO2.txt","r");
	st = fopen("..\\I-O Files\\CO2.txt","r");

	Mode_Previous = -1;
	while(!feof(st))
	{
		Mode = g_Readnum(st);
		Energy = g_Readdouble(st);
		for (int i = Mode_Previous + 1; i <= Mode; i++)
		{
			if (i < Mode)
				temp = 0;
			else
				temp = Energy;

			emission_input.push_back(temp);
		}
		Mode_Previous = Mode;
	}
	fclose(st);

	return (a);
}


int CO2 (double V, double A)
{
	double VSP=0;
	Mode = -10;

	VSP = ((0.156461 * V + 0.00200193 * V * V + 0.000492646 * V * V * V)/1.4788) + A * V;

	V = V * 2.23694;
	if (A < -2)
		Mode = 0;//this is equivalent to Mode = 0;

	if (V > -1 && V < 1)
		Mode = 1;
	
	if (VSP < 0 && V >= 1 && V < 25)
		Mode = 11;
	if (VSP >= 0 && VSP < 3 && V >= 1 && V < 25)
		Mode = 12;
	if (VSP >= 3 && VSP < 6 && V >= 1 && V < 25)
		Mode = 13;
	if (VSP >= 6 && VSP < 9 && V >= 1 && V < 25)
		Mode = 14;
	if (VSP >= 9 && VSP < 12 && V >= 1 && V < 25)
		Mode = 15;
	if (VSP >= 12 && V >= 1 && V < 25)
		Mode = 16;

	if (VSP < 0 && V >= 25 && V < 50)
		Mode = 21;
	if (VSP >= 0 && VSP < 3 && V >= 25 && V < 50)
		Mode = 22;
	if (VSP >= 3 && VSP < 6 && V >= 25 && V < 50)
		Mode = 23;
	if (VSP >= 6 && VSP < 9 && V >= 25 && V < 50)
		Mode = 24;
	if (VSP >= 9 && VSP < 12 && V >= 25 && V < 50)
		Mode = 25;
	if (VSP >= 12 && VSP < 18 && V >= 25 && V < 50)
		Mode = 27;
	if (VSP >= 18 && VSP < 24 && V >= 25 && V < 50)
		Mode = 28;
	if (VSP >= 24 && VSP < 30 && V >= 25 && V < 50)
		Mode = 29;
	if (VSP >= 30 && V >= 25 && V < 50)
		Mode = 30;

	if (VSP < 6 && V >= 50)
		Mode = 33;
	if (VSP >= 6 && VSP < 12 && V >= 50)
		Mode = 35;
	if (VSP >= 12 && VSP < 18 && V >= 50)
		Mode = 37;
	if (VSP >= 18 && VSP < 24 && V >= 50)
		Mode = 38;
	if (VSP >= 24 && VSP < 30 && V >= 50)
		Mode = 39;
	if (VSP >= 30 && V >= 50)
		Mode = 40;

	return (Mode);
}


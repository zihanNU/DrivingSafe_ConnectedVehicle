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
#include <deque>
//#include "mrand.h"

using namespace std;

vector<deque<car *>> Vehicle_History;
car * TempP_history;

vector<double> Average_Starting_Speed;
vector<double> Starting_Speed_Counter;
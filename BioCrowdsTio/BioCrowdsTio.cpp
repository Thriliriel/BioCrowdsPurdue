// BioCrowdsTio.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

int main(int argcp, char **argv)
{
	//configure scenario
	Simulation newSimulation(100, 100, 1, argcp, argv);
	//Simulation newSimulation(1000, 1000, 5);

	system("PAUSE");

    return 0;
}


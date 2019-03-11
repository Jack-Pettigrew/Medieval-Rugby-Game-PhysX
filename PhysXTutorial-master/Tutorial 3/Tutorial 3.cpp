#include <iostream>
#include "VisualDebugger.h"

using namespace std;

int main()
{
	try 
	{ 
		VisualDebugger::Init("PET15593452 - Physics Simulation - Medievil Rugby!", 1280, 720); 
	}
	catch (Exception exc) 
	{ 
		cerr << exc.what() << endl;
		return 0; 
	}

	VisualDebugger::Start();

	return 0;
}
#include <iostream>
#include <string>
#include <algorithm>
#include <conio.h>
#include <fstream>
#include <chrono>
using namespace std;

#include "olcConsoleGameEngine.h"
#include "Interface.h"

int main(){
	cout << "This is the simulation of Path Finding algorithm.. " << endl;
	cout << "use the following key to navigate through the visualisation:" << endl;
	cout << "keys input:\nNUMPAD '1' to run path finding on Dijkstra's algorithm"<<endl;
	cout << "NUMPAD '2' to run path finding on A* algorithm" << endl;
	cout << "NUMPAD '4' to change into 4 path mode" << endl;
	cout << "NUMPAD '8' to change into 8 path mode" << endl;
	cout << "NUMPAD '9' to export the run result" << endl;
	cout << "LMB to create or delete the obstacle" << endl;
	cout << "LMB + CTRL to move the end (red) point" << endl;
	cout << "LMB + SHIFT to move the start (green) point" << endl;
	system("PAUSE");

	Path_Visualizer simulation;
	simulation.ConstructConsole(160, 160, 4, 4);
	simulation.Start();
	return 0;
}
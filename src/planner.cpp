#include <DrogonControlInterfaceLibrary.h>
#include <iostream>
#include <fstream>

using namespace std;

void fillMap(map<string, double> &goal, string filename);
DrogonControlInterface* dci;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Planner");
	if (argc < 1) {
		printf("a file name is required./n");
	}
	dci = new DrogonControlInterface();
	dci->rosEnable();
	map<string, double> goal;
	fillMap(goal, argv[1]);
	moveit::planning_interface::MoveGroup::Plan plan = dci->getPlan(goal, drogon::LEFT);
	dci->executePlan(plan, drogon::LEFT);
}
void fillMap(map<string, double> &goal, string filename)
{
	ifstream goalInput;
	goalInput.open(filename);
	string line;
	while(!goalInput.eof) {
		getLine(goalInput, line);
		// do stuff
	}
	goalInput.close();
}

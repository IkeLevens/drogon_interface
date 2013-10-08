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
	cout << "creating DrogonControlInterface" << endl;
	dci = new DrogonControlInterface();
	cout << "created DrogonControlInterface" << endl;
	dci->rosEnable();
	map<string, double> goal;
	fillMap(goal, argv[1]);
	moveit::planning_interface::MoveGroup::Plan plan = dci->getPlan(goal, drogon::LEFT);
	dci->executePlan(plan, drogon::LEFT);
	ros::waitForShutdown();
}
void fillMap(map<string, double> &goal, string filename)
{
	ifstream goalInput;
	cout << filename.c_str() << endl;
	goalInput.open(filename.c_str());
	string line;
	while(!goalInput.eof()) {
		getline(goalInput, line);
		cout << line << endl;
		// do stuff
	}
	goalInput.close();
}

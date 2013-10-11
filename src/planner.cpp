#include <DrogonControlInterfaceLibrary.h>

using namespace std;

void fillMap(map<string, double> &goal, string filename);
DrogonControlInterface* dci;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Planner");
	if (argc < 2) {
		printf("a file name is required./n");
		return -1;
	}
	cout << "creating DrogonControlInterface" << endl;
	dci = new DrogonControlInterface();
	cout << "created DrogonControlInterface" << endl;
	dci->rosEnable();
	map<string, double> goal;
	fillMap(goal, argv[1]);
	sleep(5000);
	moveit::planning_interface::MoveGroup::Plan plan = dci->getPlan(goal, drogon::LEFT);
	cout << "plan received" << endl;
	dci->executePlan(plan, drogon::LEFT);
	cout << "plan executed" << endl;
	ros::spin();
}
void fillMap(map<string, double> &goal, string filename)
{
	ifstream goalInput;
	cout << filename.c_str() << endl;
	goalInput.open(filename.c_str());
	string line;
	while(getline(goalInput, line)) {
		cout << "outer loop:: " << endl;
		cout << line << endl;
		stringstream lineStream(line);
		string key;
		string valueString;
		double value;
		while (lineStream >> key) {
			lineStream >> valueString;
			value = atof(valueString.c_str());
			goal[key] = value;
		}
	}
	goalInput.close();
}

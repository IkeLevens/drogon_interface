#include <DrogonControlInterfaceLibrary.h>

using namespace std;

void fillMap(map<string, double> &goal, string filename);
DrogonControlInterface* dci;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Planner");
	if (argc < 2) {
		ROS_WARN("a file name is required.\n");
		return -1;
	}
	ROS_INFO("creating DrogonControlInterface\n");
	dci = new DrogonControlInterface();
	ROS_INFO("created DrogonControlInterface\n");
//	dci->waitForMoveit();
	dci->rosEnable();
	map<string, double> goal;
	fillMap(goal, argv[1]);
//	int count = 0;
	stringstream ss;
	typedef std::map<std::string, double>::iterator it_type;
	for(it_type iterator = goal.begin(); iterator != goal.end(); iterator++) {
//		ss << ++count;
//		ROS_INFO(ss.str().c_str());
		ROS_INFO(iterator->first.c_str());
//		ss.str("");
		ss << iterator->second;
		ROS_INFO(ss.str().c_str());
		ss.str("");
	}
	ROS_INFO("requesting plan\n");
	moveit::planning_interface::MoveGroup::Plan* plan = dci->getPlan(goal, drogon::LEFT);
	ROS_INFO("plan received\n");
	dci->executePlan(*plan, drogon::LEFT);
	ROS_INFO("plan executed\n");
	ros::spin();
	return 0;
}
void fillMap(map<string, double> &goal, string filename)
{
	ifstream goalInput;
	cout << filename.c_str() << endl;
	goalInput.open(filename.c_str());
	string line;
	while(getline(goalInput, line)) {
//		ROS_INFO("outer loop:: \n");
//		ROS_INFO(line.c_str());
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

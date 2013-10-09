#include <DrogonControlInterfaceLibrary.h>

using namespace std;
using namespace drogon;

int main (int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Web_Server");
	DrogonControlInterface dci;
	dci.rosEnable();
	dci.enableWebServer();
	ros::waitForShutdown();
	dci.rosDisable();
}

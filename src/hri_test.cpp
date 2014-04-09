/* 
 * File:   hri_test.cpp
 * Author: Isaac Yochelson
 *
 * Created on April 9, 2014, 11:05 AM
 */
#include <cstdlib>
#include <iostream>
#include <sstream>
int main(int argc, char** argv) {
	for (int i = 3; i<8; ++i) {
		std::stringstream ss;
		ss << i;
		std::cout << "straight: " << ss.str() << std::endl;
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f trj_straight_right_" + ss.str() + ".trj").c_str());
		std::cout << "curve: " << ss.str() << std::endl;
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f trj_curve_right_" + ss.str() + ".trj").c_str());
		std::cout << "claw: " << ss.str() << std::endl;
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f trj_claw_right_" + ss.str() + ".trj").c_str());
		std::cout << "short: " << ss.str() << std::endl;
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f trj_short_right_" + ss.str() + ".trj").c_str());
	};
	for (int i = 13; i>8; --i) {
		std::stringstream ss;
		ss << i;
		std::cout << "straight: " << ss.str() << std::endl;
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f trj_straight_left_" + ss.str() + ".trj").c_str());
		std::cout << "curve: " << ss.str() << std::endl;
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f trj_curve_left_" + ss.str() + ".trj").c_str());
		std::cout << "claw: " << ss.str() << std::endl;
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f trj_claw_left_" + ss.str() + ".trj").c_str());
		std::cout << "short: " << ss.str() << std::endl;
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f trj_short_left_" + ss.str() + ".trj").c_str());
	};
	return 0;
}

/* 
 * File:   trjparser.cpp
 * Author: Isaac Yochelson
 *
 * Created on February 20, 2014, 3:06 PM
 */
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/tokenizer.hpp>

using namespace std;

bool parse(string filename);

int main(int argc, char** argv)
{
	if (argc < 2) {
		cout << "Usage: trjparser <filename>\n";
		return -1;
	} else {
		string filename(argv[1]);
		if (parse(filename)) {
			return 0;
		} else {
			cout << "filename was invalid\n";
			cout << filename << "\n";
			return -1;
		}
	}
}
bool parse(string filename)
{
	if ((filename.substr(filename.length()-3, 3)) != "trj") {
		cout << (filename.substr(filename.length()-3, 3)) << " is not trj\n";
		return false;
	}
	int target;
	int digits;
	double x = .82;
	double z = .18;
	double y = .104 * (target - 8); 
	target = atoi((filename.substr(filename.length()-6, 2)).c_str());
	if (target > 9) {
		digits = 2;
	} else {
		target = atoi((filename.substr(filename.length()-5, 1)).c_str());
		digits = 1;
	}
	cout << "target: " << target << " digits: " << digits << " X: " << x << " Y: " << y << " Z: " << z << "\n";
	bool left;
	if ((filename.substr(filename.length()-11+digits, 4)) == "left") {
		left = true;
	} else {
		if ((filename.substr(filename.length()-12+digits, 5)) == "right") {
			left = false;
		} else {
			cout << "file name does not end in left or right\n";
			cout << "file ends in: " << filename.substr(filename.length()-12+digits, 5) << "\n";
			return false;
		}
	}
	ifstream trjInput;
	trjInput.open(filename.c_str());
	double time, s0, s1, e0, e1, w0, w1, w2;
	string trjLine;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> commaDelimited(",");
	int line = 0;
	while (getline(trjInput, trjLine)) {
		if (++line > 1) {
			tokenizer values(trjLine, commaDelimited);
			int count = 0;
			for (tokenizer::iterator value_iter = values.begin(); value_iter != values.end();
					++value_iter)
			{
				++count;
				if (count == 1) {
					time = atof((*value_iter).c_str());
				}
				if ((left && (count == 2)) || (!left && (count == 10))) {
					s0 = atof((*value_iter).c_str());
				}
				if ((left && (count == 3)) || (!left && (count == 11))) {
					s1 = atof((*value_iter).c_str());
				}
				if ((left && (count == 4)) || (!left && (count == 12))) {
					e0 = atof((*value_iter).c_str());
				}
				if ((left && (count == 5)) || (!left && (count == 13))) {
					e1 = atof((*value_iter).c_str());
				}
				if ((left && (count == 6)) || (!left && (count == 14))) {
					w0 = atof((*value_iter).c_str());
				}
				if ((left && (count == 7)) || (!left && (count == 15))) {
					w1 = atof((*value_iter).c_str());
				}
				if ((left && (count == 8)) || (!left && (count == 16))) {
					w2 = atof((*value_iter).c_str());
				}
			}
			stringstream ss;
			ss << "values for line " << line << ": s0: " << s0 << " s1: " << s1 << " e0: " << e0
				<< " e1: " << e1 << " w0: " << w0 << " w1: " << w1 << " w2: " << w2 << " time: " << time << "\n";
			cout << ss.str();
			// Add actual functionality here
		}
	}
	return true;
}

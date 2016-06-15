//CPP
#include <iostream>
#include <math.h>
//EIGEN
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
//local
#include "robothandler.h"

int trajectory() {
	//wanted position
	double radius_d = 1, //radius of the circle 1m
	 alpha=.1; //rotational speed along circle .1 rad/s
	Eigen::Vector3d p(1,2,3);
}
/*To launch program 
 * sudo chmod 777 -R /dev/ttyUSB0 (the name of the USB port attached to the robot (use dmesg))
 * ./pioneerax -robotPort /dev/ttyUSB0 
 **/
int main(int argc, char** argv) {    
	Robothandler rh(argc,argv);
	rh.connection();
	rh.getInitialData();
	//~ rh.wander();
	//~ rh.followSquare();
	rh.disconnection();
	Aria::exit(0);
	return 0;
}



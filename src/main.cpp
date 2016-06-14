//CPP
#include <iostream>
//EIGEN
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
//local
#include "robothandler.h"

int trajectory() {
}
/*To launch program 
 * sudo chmod 777 -R /dev/ttyUSB0 (the name of the USB port attached to the robot (use dmesg))
 * ./pioneerax -robotPort /dev/ttyUSB0 
 **/
int main(int argc, char** argv) {    
	//ariaSequence(argc,argv);
	Robothandler rh(argc,argv);
	int retCode = rh.connection();
	printf("connexion has returned %d/n",retCode);
	rh.getInitialData();
	rh.disconnection();
	return 0;
}



//ARIA
#include "Aria.h"
//CPP
#include <iostream>
//EIGEN
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

int trajectory() {
}

int ariaSequence(int argc, char** argv) {
	Aria::init();
	ArArgumentParser parser(&argc,argv);
	ArRobot robot;
	ArAnalogGyro gyro(&robot);
	ArSonarDevice sonar;
	ArRobotConnector robotConnector(&parser, &robot);
	ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
	
	//start connection sequence
	if(!robotConnector.connectRobot()) {
		ArLog::log(ArLog::Terse, "Ax-Example: Could not connect to the robot.");
	if(parser.checkHelpAndWarnUnparsed()) {
		Aria::logOptions();
		Aria::exit(1);
		}
	}
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
		Aria::logOptions();
		Aria::exit(1);
		}
	ArLog::log(ArLog::Normal, "Ax-Example: Connected to robot.");
	//end of connection sequence
	
	//declare sonar
	robot.addRangeDevice(&sonar);
	robot.runAsync(true);
	//Make a key handler, so that escape will shut down the program
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	printf("You may press escape to exit\n");
	
	//Collision avoidance (eventually) - may disrupt control
	
	//wander around
	//try to connect to laser. if fail, warn & continue
		if(!laserConnector.connectLasers()) {
			ArLog::log(ArLog::Normal, "Warning: unable to connect lasers. Using only sonar");
		}
	//turn on the motors
	robot.enableMotors();
	robot.comInt(ArCommands::SOUNDTOG,0);
	//Wander core: set of actions to make wander bahavior
	ArActionStallRecover revover;
	ArActionBumpers bumpers;
	ArActionAvoidFront avoidFrontNear("Avoid front near",255,0);
	ArActionAvoidFront avoidFrontFar;
	ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
}

int main(int argc, char** argv) {    
	ariaSequence(argc,argv);        
	return 0;
}



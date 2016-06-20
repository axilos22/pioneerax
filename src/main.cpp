/**
 * @author Axel JEANNE for ECN
 * @date 19/06/2016
 * @version 0.07
 * @note To launch program :
 * 		sudo chmod 777 -R /dev/ttyUSB0 (the name of the USB port attached to the robot (use dmesg)
 * 		./pioneerax -robotPort /dev/ttyUSB0
 */
//define
#define SUCCESSFUL_EXE_CODE 0
#define CONNECTION_FAILED_CODE 2
#define VERBOSE 1
#define LOOP 50

//CPP
#include <iostream>
#include <istream>

//local
#include "robothandler.h"
#include "trajectory.h"

const double circleRadius = 400,//mm
angularSpeed = .01,//rad/s
errorGain = 0;  //gain applied to error
/**
 * @brief makeRobotWander make the robot wander (normally)
 * @param rh the robot handler
 */
void makeRobotWander(Robothandler &rh) {
    int retCode = rh.connection();
    if(!retCode) {
        rh.makeKeyHandler();
        rh.prepareToMove();
        rh.wander();
        rh.disconnection();
    } else {
        ArLog::log(ArLog::Normal, "Ax-Example@main: Unable to connect to robot. Abort");
    }
}
/**
 * @brief squareTrajectory make the robot follow a square trajectory
 * @param rh the robot handler
 */
void squareTrajectory(Robothandler &rh) {
    int retCode = rh.connection();
    if(!retCode) {
        rh.makeKeyHandler();
        rh.prepareToMove();
        rh.followSquare();
        rh.disconnection();
    } else {
        ArLog::log(ArLog::Normal, "Ax-Example@main: Unable to connect to robot. Abort");
    }
}
/**
 * @brief main main of the program. Intenally it use a Robothandler and a Trajectory. Both classes have linkages
 * to comunicate.
 * @param argc
 * @param argv
 * @return return code (check defines).
 */
int main(int argc, char** argv) {    		
    Robothandler rh(argc,argv);
    Trajectory tr(circleRadius,angularSpeed,errorGain); //radius (mm) ,angular speed (rad/s) and error gain
    int retCode = rh.connection();
    if(!retCode) { //if we connected
        rh.makeKeyHandler();
        rh.prepareToMove();
        tr.setInitialPose(0,0,0);
        int loop = 0;
        ArLog::log(ArLog::Normal,"Ax-example@main : Begin control in 3s");
        ArUtil::sleep(3000);
        rh.resetTime(); //reset time for the control algorithm
        while(Aria::getRunning() && loop < LOOP) {
            std::cout << "############### @main " << "[" << loop << "] ###############" << std::endl;
            Eigen::Vector2d v_w = tr.trajectorySequence(rh.getTime()->secSince(),rh.getPoseEigen());
            rh.setCommand(v_w(0,0),v_w(1,0));
            loop++;
        }
        rh.disconnection();
    } else {
        ArLog::log(ArLog::Normal, "Ax-Example@main: Unable to connect to robot. Abort");
        Aria::exit(CONNECTION_FAILED_CODE);
        return CONNECTION_FAILED_CODE;
    }
    Aria::exit(SUCCESSFUL_EXE_CODE);
    return SUCCESSFUL_EXE_CODE;
}

/* Test functions 
* get some initial data after connection
rh.getInitialData();
*/

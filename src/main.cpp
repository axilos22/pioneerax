/**
 * @author Axel JEANNE for ECN
 * @date 15/06/2016
 * @note To launch program :
 * 		sudo chmod 777 -R /dev/ttyUSB0 (the name of the USB port attached to the robot (use dmesg)
 * 		./pioneerax -robotPort /dev/ttyUSB0
 */
//CPP
#include <iostream>
#include <istream>

//local
#include "robothandler.h"
#include "trajectory.h"

#define SUCCESSFUL_EXE_CODE 0
#define CONNECTION_FAILED_CODE 2
/**
 * @brief makeRobotWander make the robot wander (normally)
 * @param rh the robot handler
 */
void makeRobotWander(Robothandler &rh) {
    int retCode = rh.connection();
    if(!retCode) {
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
        rh.prepareToMove();
        rh.followSquare();
        rh.disconnection();
    } else {
        ArLog::log(ArLog::Normal, "Ax-Example@main: Unable to connect to robot. Abort");
    }
}

int main(int argc, char** argv) {    
    Robothandler rh(argc,argv);
    Trajectory tr(150,.1); //radius (mm) and angular speed (rad/s)
    int retCode = rh.connection();
    int loop = 0;
    //~ ArLog::log(ArLog::Normal, "Ax-Example@main: Connection OK");
    rh.prepareToMove();
    rh.resetTime();
    tr.setInitialPose(0,0,0);
    if(!retCode) {
        while(Aria::getRunning() && loop < 10) {
            std::cout << "##### @main " <<"[" << loop << "] #####" << std::endl;
            //Start of traj sequence
            Eigen::Vector2d v_w = tr.trajectorySequence(rh.getTime()->secSince(),rh.getPose());
            //end of traj seq
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

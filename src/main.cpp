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

void trajectorySequence(Robothandler &rh) {	
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
		std::cout << "[" << loop << "]" << std::endl;
        //Start of traj sequence
        //~ std::cout << "robot time= " << rh.getTime()->secSince() << "s" << std::endl;
        //~ std::cout << "robot time= " << rh.getTime()->mSecSince() << "ms" << std::endl;
        tr.updateTime(rh.getTime()->secSince());
        tr.computeDesired();
        std::cout <<"@main-Desired Pos vector \n "<< tr.getDesiredPosition() << std::endl;
        tr.updateRobotPose(rh.getPose());        
        std::cout <<"@main Updated robot pose \n "<< tr.getRobotPose() << std::endl;
        tr.computeError();        
        std::cout <<"@main  ERROR vector \n "<< tr.getErrorPosition() << std::endl;
        Eigen::Vector2d v_w= tr.computeCommands();
        std::cout <<"@main CMD= \n "<< v_w << std::endl;
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
* make the robot wander (normally)
rh.wander();
* make the robot follow a square trajectory
rh.followSquare();
* */

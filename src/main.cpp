/**
 * @author Axel JEANNE for ECN
 * @date 20/06/2016
 * @version 0.08
 * @note To launch program :
 * 		sudo chmod 777 -R /dev/ttyUSB0 (the name of the USB port attached to the robot (use dmesg)
 * 		./pioneerax -robotPort /dev/ttyUSB0
 */
//CPP
#include <ctime>
#include <iostream>
#include <istream>
#include <fstream>
//local
#include "../include/robothandler.h"
#include "../include/trajectory.h"
#include "../include/controller.h"
#include "../include/circulartrajectory.h"
//define
#define SUCCESSFUL_EXE_CODE 0
#define CONNECTION_FAILED_CODE 2
#define VERBOSE 1
#define RECORD_CSV 1
#define LOOP 300

const double circleRadius = 1000,//mm
angularSpeed = .3,//rad/s
errorGain = 0.2;  //gain applied to error
const std::string trajFilePath = "../traj.csv";
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
void circularTrajectory(Robothandler& rh, double& radius, double& angularSpeed) {
    CircularTrajectory ct(radius,angularSpeed);
    Controller controller(.2,150);
    rh.makeKeyHandler();
    rh.prepareToMove();
    controller.setInitialPose(0,0,0);
    int loop =0;
    ArLog::log(ArLog::Normal,"Ax-example@main : Begin control in 3s");
    ArUtil::sleep(3000);
    while(Aria::getRunning() && loop < LOOP) {
        double time = rh.getTime()->mSecSince()/1000.0;
        ct.computeDesired(time);
        controller.updateRobotPose(rh.getPoseEigen());
        controller.computeError(ct.desiredPosition());
        Eigen::Vector2d v_w = controller.computeCommands();
        rh.setCommand(v_w(0),v_w(1));
        loop++;
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
    #if RECORD_CSV == 1
		std::ofstream csvFile;
		csvFile.open(trajFilePath);
		csvFile << "loop;time;x_d;y_d;x;y;x_err;y_err;V;w\n"; 		
    #endif
    int retCode = rh.connection();
    if(!retCode) { //if we connected
        rh.makeKeyHandler();
        rh.prepareToMove();
        tr.setInitialPose(0,0,0);
        int loop = 0;

        rh.resetTime(); //reset time for the control algorithm
        while(Aria::getRunning() && loop < LOOP) {
            std::cout << "############### @main " << "[" << loop << "] ###############" << std::endl;
            double time = rh.getTime()->mSecSince()/1000.0;
            Eigen::Vector2d v_w = tr.trajectorySequence(time,rh.getPoseEigen());
            Eigen::Vector2d desired = tr.getDesiredPosition();
		    Eigen::Vector2d err = tr.getErrorPosition();
		    Eigen::Vector3d robotPose = tr.getRobotPose();
		    #if RECORD_CSV == 1
			    csvFile << loop << ";" << time << ";" << desired(0) << ";" << desired(1) 
					<< ";" << robotPose(0) << ";" << robotPose(1) << ";" << err(0) 
					<< ";" << err(1) << ";" << v_w(0) << ";" << v_w(1) << "\n";				
		    #endif
            rh.setCommand(v_w(0),v_w(1));
            loop++;
        }
        csvFile.close();
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

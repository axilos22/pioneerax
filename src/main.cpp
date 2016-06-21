/**
 * @author Axel JEANNE for ECN
 * @date 21/06/2016
 * @version 0.083
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
#include "../include/controller.h"
#include "../include/circulartrajectory.h"
#include "../include/camhandler.h"
//lib
#include "../lib/odometry/include/odometry.h"
//define
#define SUCCESSFUL_EXE_CODE 0
#define CONNECTION_FAILED_CODE 2
#define VERBOSE 1
#define RECORD_CSV 1
#define LOOP 150
#define CAM_DEFAULT_ID 0

const double circleRadius = 1000,//mm
angularSpeed = .3,//rad/s
errorGain = 0.2,  //gain applied to error
distance2center = 105, //distance to the controller point of the robot
Kp = .2; //proportional gain
const std::string recordFilePath = "record.txt";
std::ofstream recordFile;
const std::vector<double> camParam = { 0.734966,1.01564,239.228,238.882,319.815,237.017};

void recordData(std::vector<double> data) {
	for(int rank=0;rank<data.size();rank++) {
		recordFile << data.at(rank) << " ";
	}
	recordFile << "\n";
}
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
void doCircularTrajectory(Robothandler& rh,const double& radius,const double& angularSpeed) {
	//traj & control
    CircularTrajectory ct(radius,angularSpeed);
    Controller controller(Kp,distance2center);
    //robot
    rh.makeKeyHandler();
    rh.prepareToMove();        
    //vision
    CamHandler cam(CAM_DEFAULT_ID);
    MonocularOdometry odom(camParam);
    cv::namedWindow("camera");
    //exe
    int loop =0;
    ArLog::log(ArLog::Normal,"Ax-example@main : Begin control in 3s");
    ArUtil::sleep(3000);
    while(Aria::getRunning() && loop < LOOP) {
		//control part
		std::vector<double> recordedData;
        double time = rh.getTime()->mSecSince()/1000.0;        
        Eigen::Vector3d robotPose = rh.getPoseEigen();
        ct.computeDesired(time);
        Eigen::Vector2d desiredPosition = ct.desiredPosition(),
        desiredPositionDot = ct.desiredPositionDot();        
        controller.updateRobotPose(robotPose);
        controller.computeError(desiredPosition);
        Eigen::Vector2d v_w = controller.computeCommands(desiredPositionDot);
        rh.setCommand(v_w(0),v_w(1));
        //vision part
        cv::Mat frame = cam.read();
        Transformation<double> transf = odom.feedImage(frame);
        Vector3<double> camTrans = transf.trans();
        Vector3<double> camRot = transf.rot();
        cv::waitKey(100);
        //record part
        Eigen::Vector2d positionErr = controller.getPositionError();
        recordedData.push_back(time);
        recordedData.push_back(robotPose(2));
        recordedData.push_back(camRot(2));
        recordData(recordedData);
        /*RECORD TRAJ 
        recordedData.push_back(robotPose(0));
        recordedData.push_back(robotPose(1));
        recordedData.push_back(desiredPosition(0));
        recordedData.push_back(desiredPosition(1));
        recordedData.push_back(positionErr(0));
        recordedData.push_back(positionErr(1));
        */
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
	recordFile.open(recordFilePath);
	Robothandler rh(argc,argv);	
	int retCode = rh.connection();
    if(!retCode) { //if we connected
		doCircularTrajectory(rh,circleRadius,angularSpeed);
        rh.disconnection();
    } else {
        ArLog::log(ArLog::Normal, "Ax-Example@main: Unable to connect to robot. Abort");
        Aria::exit(CONNECTION_FAILED_CODE);
        return CONNECTION_FAILED_CODE;
    }
    Aria::exit(SUCCESSFUL_EXE_CODE);
    return SUCCESSFUL_EXE_CODE;
    recordFile.close();
}

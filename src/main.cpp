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
#include <iomanip>
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
#define LOOP 500
#define CAM_DEFAULT_ID 0

using namespace std;
//odometry parameters
const double displacementThreshold = 200;//mm
const double turningThreshold = .5;//rad
//circular traj parameters
const double circleRadius = 1200;//mm
const double angularSpeed = .3;//rad/s
//Robot control parameter
const double distance2center = 134; //distance to the controller point of the robot
const double d = 200;
const double Kp = .3; //proportional gain
const std::string recordFilePath = "record.txt",
recordFileHeader = "time x y th x_d y_d x_e y_e camRx camRy camRz\n";
//~ recordFileHeader = "time th camRy err\n";
std::ofstream recordFile;
const std::vector<double> camParam = {0.734966,1.01564,239.228,238.882,319.815,237.017};

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
	//recorder
	recordFile.open("recordCircularTraj.txt");
	recordFile << "R=" << radius << " w=" << angularSpeed << " d=" << distance2center  <<" Kp=" << Kp <<"\n";
	recordFile << "# time x y th x_d y_d x_e y_e\n";
	//traj & control
    CircularTrajectory ct(radius,angularSpeed);      
    Controller controller(Kp, d, distance2center);
    //robot
    rh.makeKeyHandler();
    rh.prepareToMove();        
    //vision
    CamHandler cam(CAM_DEFAULT_ID);
    MonocularOdometry odom(camParam);
    //exe
    int loop =0;
    ArLog::log(ArLog::Normal,"Ax-example@main : Begin control in 1s");
    ArUtil::sleep(1000);
    ct.time0 = rh.getTime()->mSecSince()/1000.0;    
    // integrated VO
    Transformation<double> voPose;
    Vector3d woPose;
    const int voLoopCount = 5;
    while(Aria::getRunning() && loop < LOOP) {
        //ODOM
        //no odometry for now
		//CONTROL PART
        //~ cout << "### iteration number " << loop << endl;
		std::vector<double> recordedData;
        double time = rh.getTime()->mSecSince()/1000.0;        
        Eigen::Vector3d robotPose = rh.getPoseEigen();
        //~ cout << "robot pose" << robotPose.transpose() << endl; 
        ct.computeDesired(time);
        Eigen::Vector2d desiredPosition = ct.desiredPosition();
        //~ cout << "desired pose " << desiredPosition << endl;
		Eigen::Vector2d desiredPositionDot = ct.desiredPositionDot();        
		controller.updateRobotPose(robotPose);
		controller.computeError(desiredPosition);
		Eigen::Vector2d positionErr = controller.getPositionError();
		Eigen::Vector2d v_w = controller.computeCommands(desiredPositionDot);
		rh.setCommand(v_w(0),v_w(1));
		//RECCORD
		recordedData.push_back(loop);
	    recordedData.push_back(time);
	    recordedData.push_back(robotPose(0));
		recordedData.push_back(robotPose(1));
	    recordedData.push_back(robotPose(2));
	    recordedData.push_back(desiredPosition(0));
	    recordedData.push_back(desiredPosition(1));
	    recordedData.push_back(positionErr(0));
		recordedData.push_back(positionErr(1));
		recordData(recordedData);	
        loop++;
    }
}

void odometryTeleop(Robothandler& rh) {
	//Odometry init
    CamHandler cam(CAM_DEFAULT_ID);
    MonocularOdometry odom(camParam);
	rh.teleop();
	recordFile.open("recordOdometry.txt");
	recordFile << "# time x y th -camRY\n";
	Transformation<double> voPose;
	Vector3d woPose;
	int loop=0;
    while(1) {		
		cv::Mat frame = cam.read();//get frame from the camera
		Transformation<double> deltaVoPose = odom.feedImage(frame);//feed frame to the odometry
		Eigen::Vector3d robotPose = rh.getPoseEigen();//get the pose of the robot
		Vector3d deltaWoPose = robotPose - woPose;//compute delta between ref pose of ref frame and current pose
		deltaWoPose(2) = 0;//discard orientation in norm
		if(deltaVoPose.rot().norm() > turningThreshold or deltaWoPose.norm() > displacementThreshold) {
			std::vector<double> recordedData;
			woPose = robotPose;//update pose of the ref frame
			odom.pushImage();
			voPose = voPose.compose(deltaVoPose);
	        voPose.normalize();			
		    Vector3<double> camTrans = voPose.trans();
		    Vector3<double> camRot = voPose.rot();
		    recordedData.push_back(loop);
		    recordedData.push_back(rh.getTime()->mSecSince()/1000.0 );
		    recordedData.push_back(robotPose(0) );
		    recordedData.push_back(robotPose(1) );
		    recordedData.push_back(robotPose(2) );
		    recordedData.push_back(-camRot(1));
		    recordData(recordedData);
		    loop++;
		}
		ArUtil::sleep(10);//10ms tempo
    }
    cout << "Odom terminated, " << loop << " executed.";
}
/**
 * @brief main main of the program. Intenally it use a Robothandler and a Trajectory. Both classes have linkages
 * to comunicate.
 * @param argc
 * @param argv
 * @return return code (check defines).
 */
int main(int argc, char** argv) {    

	//~ recordFile.open(recordFilePath);
	//~ std::cout << "recording into: " << recordFilePath << std::endl;
	//~ recordFile << recordFileHeader;
	Robothandler rh(argc,argv);	
	int retCode = rh.connection();
    if(!retCode) { //if we connected
		doCircularTrajectory(rh,circleRadius,angularSpeed);
		//odometryTeleop(rh);		       
        rh.disconnection();
    } else {
        ArLog::log(ArLog::Normal, "Ax-Example@main: Unable to connect to robot. Abort");
        Aria::exit(CONNECTION_FAILED_CODE);
        return CONNECTION_FAILED_CODE;
    }
    recordFile.close();
    Aria::exit(SUCCESSFUL_EXE_CODE);
    return SUCCESSFUL_EXE_CODE;    
}

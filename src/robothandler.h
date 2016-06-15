#ifndef ROBOTHANDLER_H
#define ROBOTHANDLER_H
//CPP
#include <vector>
//ARIA
#include "Aria.h"

class Robothandler
{
public:
	//parameter
    ArArgumentParser* m_parser;
    ArRobot* m_robot;
    ArAnalogGyro* m_gyro;
    ArSonarDevice* m_sonar;
    ArRobotConnector* m_robotConnector;
    ArLaserConnector* m_laserConnector;
    Robothandler(int argc, char** argv);
    //functions
    int connection();
    int disconnection();
    void getInitialData();
    void activateSonar();
    void activateLaser();
    void makeKeyHandler();    
    int wander();
    std::vector<double> getPose();
    ~Robothandler();
    void followSquare();
};

#endif // ROBOTHANDLER_H


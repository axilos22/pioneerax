#ifndef ROBOTHANDLER_H
#define ROBOTHANDLER_H
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
    ~Robothandler();
};

#endif // ROBOTHANDLER_H


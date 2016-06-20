#ifndef ROBOTHANDLER_H
#define ROBOTHANDLER_H
//CPP
#include <vector>
#include <math.h>
//ARIA
#include "Aria.h"
//EIGEN
#include <Eigen/Dense>

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
    ArTime* m_time;
    ArKeyHandler* m_keyHandler;
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
    Eigen::Vector2d getPositionEigen();
    Eigen::Vector3d getPoseEigen();
    ~Robothandler();
    void followSquare();
    const ArTime* getTime();
    void resetTime();
    void setCommand(double v, double w);
    void prepareToMove();
    double rad2degree(double radValue);
    double degree2rad(double degValue);
};

#endif // ROBOTHANDLER_H


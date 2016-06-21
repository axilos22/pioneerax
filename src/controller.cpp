#include "../include/controller.h"

Controller::Controller(double Kp, double d)
{
    m_d = d;
    m_kp = Kp;
}

void Controller::updateRobotPose(Eigen::Vector3d robotPose)
{
    m_robotPose = robotPose;
}

void Controller::computeError(Eigen::Vector2d desiredPosition)
{
    Eigen::Vector2d robotPosition(m_robotPose(0),m_robotPose(1));
    m_positionError = desiredPosition - robotPosition;
    m_positionError = m_positionError*m_kp;
}

void Controller::setInitialPose(double x0, double y0, double th0)
{
    m_initialPose = Eigen::Vector3d(x0,y0,th0);
}

Eigen::Vector2d Controller::computeCommands(Eigen::Vector2d desiredPositionDot)
{
    double th = m_robotPose(2);
    Eigen::Vector2d u;
    u = /*m_initialPose+*/ m_positionError + desiredPositionDot;
    Eigen::Matrix2d K, invK;
    K << cos(th),-m_d*sin(th),
            sin(th), m_d*cos(th);
    invK = K.inverse();
    return invK*u;
}

Controller::~Controller(){}

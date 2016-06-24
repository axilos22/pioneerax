#include "../include/controller.h"

Controller::Controller(double Kp, double d, double center) :
m_kp(Kp),
m_d(d),
m_center(center),
m_robotPose(Eigen::Vector3d(0,0,0)),
m_initialPose(Eigen::Vector3d(0,0,0)),
m_controlledPosition(Eigen::Vector2d(0,0))
{}

void Controller::updateRobotPose(Eigen::Vector3d robotPose)
{
    m_robotPose = robotPose + m_initialPose;
    m_controlledPosition(0)= m_robotPose(0)+(m_d + m_center)*cos(m_robotPose(2));
    m_controlledPosition(1)= m_robotPose(1)+(m_d + m_center)*sin(m_robotPose(2));
}

void Controller::computeError(Eigen::Vector2d desiredPosition)
{
    //~ Eigen::Vector2d robotPosition(m_robotPose(0),m_robotPose(1));
    //~ robotPosition(0) += (m_d + m_center)*cos(m_robotPose(2));
    //~ robotPosition(1) += (m_d + m_center)*sin(m_robotPose(2));    
    m_positionError = desiredPosition - m_controlledPosition;
    m_positionError = m_positionError;
}

void Controller::setInitialPose(double x0, double y0, double th0)
{
    m_initialPose = Eigen::Vector3d(x0,y0,th0);
}

Eigen::Vector2d Controller::computeCommands(Eigen::Vector2d desiredPositionDot)
{
    double th = m_robotPose(2);
    Eigen::Vector2d u, initialPosition;
    initialPosition << m_initialPose(0), m_initialPose(1);
    u = m_positionError*m_kp + desiredPositionDot; //initial position not working
    Eigen::Matrix2d K, invK;
    K << cos(th),-m_d*sin(th),
            sin(th), m_d*cos(th);
    invK = K.inverse();
    return invK*u;
}
Eigen::Vector2d Controller::getPositionError() {
	return m_positionError;
}

Controller::~Controller(){}

Eigen::Vector2d Controller::getControlledPosition() {
	return m_controlledPosition;
}

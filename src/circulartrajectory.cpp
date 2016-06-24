#include "../include/circulartrajectory.h"

CircularTrajectory::CircularTrajectory(double radius, double angularSpeed)
{
    m_radius = radius;
    m_angularSpeed = angularSpeed;
}

void CircularTrajectory::computeDesired(const double actualTime)
{
	double time = actualTime - time0;
    //compute position
    //~ m_desiredPosition(0)= m_radius*(cos(m_angularSpeed*time));
    //test turn trajectory
    //~ m_desiredPosition(0)= 0;
    //~ m_desiredPosition(1)= 200*time;
    m_desiredPosition(0)= m_radius*(cos(m_angularSpeed*time) - 1); //so that the trajectory go through 0    
    m_desiredPosition(1)= m_radius*sin(m_angularSpeed*time);
    //compute derivative
    //test turn traj
    //~ m_desiredPositionDot(0)= 0;
    //~ m_desiredPositionDot(1)= 200;    
    m_desiredPositionDot(0)= -m_angularSpeed*m_radius*sin(m_angularSpeed*time);
    m_desiredPositionDot(1)= m_angularSpeed*m_radius*cos(m_angularSpeed*time);
}

const double CircularTrajectory::radius()
{
    return m_radius;
}

const double CircularTrajectory::angularSpeed()
{
    return m_angularSpeed;
}

const Eigen::Vector2d CircularTrajectory::desiredPosition()
{
    return m_desiredPosition;
}

const Eigen::Vector2d CircularTrajectory::desiredPositionDot()
{
    return m_desiredPositionDot;
}

CircularTrajectory::~CircularTrajectory(){}

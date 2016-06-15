#include "trajectory.h"

Trajectory::Trajectory(double radius, double w)
{    
	//wanted position
	m_radius = radius; //radius of the circle 1m
	m_angularSpeed = w; //rotational speed along circle .1 rad/s	
}

Trajectory::~Trajectory()
{    
}

void Trajectory::updateTime(const double time_s, const double time_ms) {
	//m_time_s = arTime->getSec();
	//m_time_ms = arTime->getMSec();
	m_time_s = time_s;
	m_time_ms = time_ms;
}

void Trajectory::setInitialPose(double x0, double y0, double th0) {
	m_initialPose(0,0)=x0;
	m_initialPose(1,0)=y0;
	m_initialPose(2,0)=th0;
}

void Trajectory::computeDesired() {
	double th0 = m_initialPose(2,0);
	m_desiredPosition(0,0)= m_radius*cos(m_angularSpeed*m_time_s+th0);
	m_desiredPosition(1,0)= m_radius*sin(m_angularSpeed*m_time_s+th0);
}

void Trajectory::updateRobotPose(const std::vector<double> pos) {
	m_pose(0,0) = pos.at(0);
	m_pose(1,0)= pos.at(1);
	m_pose(2,0)= pos.at(2);
}

void Trajectory::computeError() {
	Eigen::Vector2f currentPosition;
	currentPosition(m_pose(0,0));
	currentPosition(m_pose(1,0));
	m_errorPosition = m_desiredPosition - currentPosition;
}

const Eigen::Vector2f Trajectory::getErrorPosition() {
	return m_errorPosition;
}

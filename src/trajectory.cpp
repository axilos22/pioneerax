#include "trajectory.h"

Trajectory::Trajectory(double radius, double w, double errGain)
{    
    //wanted position
    m_radius = radius; //radius of the circle 1m
    m_angularSpeed = w; //rotational speed along circle .1 rad/s
    m_gain = errGain;
    m_d = 50; //50mm
}

Trajectory::~Trajectory()
{    
}

void Trajectory::updateTime(const double time_s, const double time_ms) {
    m_time_s = time_s;
    m_time_ms = time_ms;
}

void Trajectory::setInitialPose(double x0, double y0, double th0) {
    m_initialPose(0,0)=x0;
    m_initialPose(1,0)=y0;
    m_initialPose(2,0)=th0;
}

void Trajectory::setInitialPose(const std::vector<double> pos)
{
    m_initialPose(0,0)=pos.at(0);
    m_initialPose(1,0)=pos.at(1);
    m_initialPose(2,0)=pos.at(2);
}
/**
 * @brief Trajectory::computeDesired compute the desired position and its derivative.
 */
void Trajectory::computeDesired() {   
    m_desiredPosition(0,0)= m_radius*cos(m_angularSpeed*m_time_s);
    m_desiredPosition(1,0)= m_radius*sin(m_angularSpeed*m_time_s);
    m_desiredPositionDot(0,0)=-m_angularSpeed*m_radius*sin(m_angularSpeed*m_time_s);
    m_desiredPositionDot(1,0)=m_angularSpeed*m_radius*cos(m_angularSpeed*m_time_s);
}
/**
 * @brief Trajectory::updateRobotPose get the robot pose from the phisical robot for the control.
 * @param pos the position of the robot (x,y)
 */
void Trajectory::updateRobotPose(const std::vector<double> pos) {
    m_pose(0,0) = pos.at(0);
    m_pose(1,0)= pos.at(1);
    m_pose(2,0)= pos.at(2);
}

void Trajectory::computeError() {	
    Eigen::Vector2d currentPosition;
    //~ std::cout << "m_pose =" << m_pose << std::endl;
    std::cout << "@computeErr m_pose X =" << m_pose(0,0) << std::endl;
    std::cout << "@computeErr m_pose Y =" << m_pose(1,0) << std::endl;
    currentPosition(m_pose(0,0));
    currentPosition(m_pose(1,0));
    std::cout << "@computeErr desiredPos =" << m_desiredPosition << std::endl;
    m_errorPosition = m_desiredPosition-currentPosition;
    std::cout << "@computeErr SUBBED" << std::endl;
    m_errorPosition = m_errorPosition+m_gain*m_errorPosition;
    std::cout << "@computeErr MULED" << std::endl;
}

const Eigen::Vector2d Trajectory::getErrorPosition() {
    return m_errorPosition;
}

const Eigen::Vector2d Trajectory::getDesiredPosition()
{
    return m_desiredPosition;
}

const Eigen::Vector3d Trajectory::getRobotPose()
{
    return m_pose;
}

void Trajectory::setGain(const double gain)
{
    m_gain = gain;
}

Eigen::Vector2d Trajectory::computeCommands() {
    Eigen::Matrix2d invK(2,2);
    double th = m_pose(2,0);
    //~ std::cout << "THETA =" << th << std::endl;
    invK << cos(th), sin(th),(-1/m_d)*sin(th), (1/m_d)*cos(th);
    //~ std::cout <<"TRAJECTORY --- INV K mat" << invK << std::endl;
    //~ std::cout <<"TRAJECTORY --- ERR POS" << m_errorPosition << std::endl;
    Eigen::Vector2d v_w = invK*m_errorPosition;
    //~ v_w = invK*m_desiredPositionDot;
    return v_w;

}
/**
 * @brief Trajectory::trajectorySequence executes the full sequence to compute commands
 * @param time_s the current time for the robot (s)
 * @param pos the current position of the robot (mm)
 * @return
 */
Eigen::Vector2d Trajectory::trajectorySequence(const double time_s, std::vector<double> pos)
{
    //Start of traj sequence
    //~ std::cout << "robot time= " << rh.getTime()->secSince() << "s" << std::endl;
    //~ std::cout << "robot time= " << rh.getTime()->mSecSince() << "ms" << std::endl;
    updateTime(time_s);
    computeDesired();
    std::cout <<"@trajSeq Desired Pos vector \n "<< getDesiredPosition() << std::endl;
    updateRobotPose(pos);
    std::cout <<"@trajSeq Updated robot pose \n "<< getRobotPose() << std::endl;
    computeError();
    std::cout <<"@trajSeq  ERROR vector \n "<< getErrorPosition() << std::endl;
    Eigen::Vector2d v_w= computeCommands();
    std::cout <<"@trajSeq CMD= \n "<< v_w << std::endl;
    return v_w;
}

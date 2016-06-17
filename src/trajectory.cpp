#include "trajectory.h"

Trajectory::Trajectory(double radius, double w, double errGain)
{    
    //wanted position
    m_radius = radius; //radius of the circle (mm)
    m_angularSpeed = w; //rotational speed along circle (rad/s)
    m_gain = errGain;
    m_d = 50; //50mm
    std::cout << "New trajectory generated. type=circular R=" << m_radius
		<< " W=" << m_angularSpeed 
		<< "Kerr="<< m_gain 
		<< "d=" << m_d << std::endl;
}

Trajectory::~Trajectory(){    
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
	if(pos.size() != 3) {
		std::cout << "WARNING: Unexpected size of the pose. " << std::endl;
	}
    m_pose(0,0) = pos.at(0);
    m_pose(1,0)= pos.at(1);
    m_pose(2,0)= degree2rad(pos.at(2));
}

//~ void Trajectory::updateRobotPose(const Eigen::Vector2d position) {
	//~ m_pose(0,0)=position(0,0);
	//~ m_pose(1,0)=position(1,0);
	//~ //no information about orientation here.
	//~ m_pose(2,0)=0;
//~ }

void Trajectory::updateRobotPose(const Eigen::Vector3d pose) {
	m_pose = pose;	
	m_pose(2,0)= degree2rad(pose(2,0));
}

void Trajectory::computeError() {	    
    //~ std::cout << "m_pose =" << m_pose << std::endl;
    //~ std::cout << "@computeErr m_pose X =" << m_pose(0,0) << std::endl;
    //~ std::cout << "@computeErr m_pose Y =" << m_pose(1,0) << std::endl;    
    Eigen::Vector2d currentPosition(m_pose(0,0),m_pose(1,0));
    //~ std::cout << "@computeErr desiredPos =" << m_desiredPosition << std::endl;
    m_errorPosition = m_desiredPosition-currentPosition;
    //~ std::cout << "@computeErr SUBBED" << std::endl;
    m_errorPosition+=m_gain*m_errorPosition;
    //~ std::cout << "@computeErr MULED" << std::endl;
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
    double th = m_pose(2,0);
    //~ std::cout << "THETA =" << th << std::endl;
    Eigen::Matrix2d invK(2,2);
    invK << cos(th),(-1/m_d)*sin(th), sin(th), (1/m_d)*cos(th);
    //~ std::cout <<"TRAJECTORY --- INV K mat" << invK << std::endl;
    //~ std::cout <<"TRAJECTORY --- ERR POS" << m_errorPosition << std::endl;
    //~ Eigen::Vector2d v_w = invK*m_errorPosition;
    //~ v_w = invK*m_desiredPositionDot;
    std::cout << "invK =" << invK << std::endl;
    std::cout << "W =" << m_W << std::endl;
    Eigen::Vector2d v_w = invK*m_W;
    v_w(1,0) = v_w(1,0)*180/M_PI; // convert w in degree/sec
    return v_w;

}
/**
 * @brief Trajectory::trajectorySequence executes the full sequence to compute commands
 * @param time_s the current time for the robot (s)
 * @param pos the current position of the robot (mm)
 * @return
 */
Eigen::Vector2d Trajectory::trajectorySequence(const double time_s, Eigen::Vector3d pose)
{
    //Start of traj sequence
    std::cout << "robot time= " << time_s << "s" << std::endl;
    //~ std::cout << "robot time= " << rh.getTime()->mSecSince() << "ms" << std::endl;
    updateTime(time_s);
    computeDesired();
    std::cout <<"@trajSeq Desired Pos vector \n "<< getDesiredPosition() << std::endl;
    updateRobotPose(pose);
    //~ addDPart(); // add the part used to control the point P of the robot.
    std::cout <<"@trajSeq Updated robot pose \n "<< getRobotPose() << std::endl;
    computeError();
    std::cout <<"@trajSeq  ERROR vector (after gain) \n "<< getErrorPosition() << std::endl;
    addDesiredDerivatives(); //take the desired derivatives for control
    Eigen::Vector2d v_w = computeCommands();
    std::cout <<"@trajSeq CMD= \n "<< v_w << std::endl;
    //convert lateral speed to rad/s
    v_w(1,0) = rad2degree(v_w(1,0));
    return v_w;
}

void Trajectory::addDPart() {
	m_pose(0,0)+=m_d*cos(m_pose(2,0)); // Hx = x + cos(th)
	m_pose(1,0)+=m_d*sin(m_pose(2,0)); // Hy = y + sin(th)
}

void Trajectory::addDesiredDerivatives() {
	m_W = m_errorPosition + m_desiredPositionDot;
}

double Trajectory::rad2degree(double radValue) {
	return (radValue*180.0)/M_PI;
}

double Trajectory::degree2rad(double degValue) {
	return (degValue*M_PI)/180.0;
}

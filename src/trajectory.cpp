#include "trajectory.h"
/**
 * @brief Trajectory::Trajectory create a circular trajectory.
 * @param radius radius of the circle (mm)
 * @param w angular speed along the circle
 * @param errGain error gain.
 */
Trajectory::Trajectory(double radius, double w, double errGain)
{    
    //wanted position
    m_radius = radius; //radius of the circle (mm)
    m_angularSpeed = w; //rotational speed along circle (rad/s)
    m_errorGain = errGain;
    m_d = 50; //50mm
    std::cout << "New trajectory generated. type=circular R=" << m_radius
              << " W=" << m_angularSpeed
              << "Kerr="<< m_errorGain
              << "d=" << m_d << std::endl;
}

Trajectory::~Trajectory(){    
}
/**
 * @brief Trajectory::updateTime update the time of the control loop
 * @param time_s time in s
 * @param time_ms time in ms
 */
void Trajectory::updateTime(const double time_s, const double time_ms) {
    m_time_s = time_s;
    m_time_ms = time_ms;
}
/**
 * @brief Trajectory::setInitialPose set the initial pose of the control loop
 * @param x0 initial x
 * @param y0 initial y
 * @param th0 initial th
 */
void Trajectory::setInitialPose(double x0, double y0, double th0) {
    m_initialPose(0,0)=x0;
    m_initialPose(1,0)=y0;
    m_initialPose(2,0)=th0;
}
/**
 * @brief Trajectory::setInitialPose with a standart vector (need 3 values)
 * @param pos
 */
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
/**
 * @brief Trajectory::updateRobotPose for the error computation
 * @param pose
 */
void Trajectory::updateRobotPose(const Eigen::Vector3d pose) {
    m_pose = pose;
    m_pose(2,0)= degree2rad(pose(2,0));
}
/**
 * @brief Trajectory::computeError compute the error between desired position and current position
 * and apply error gain
 */
void Trajectory::computeError() {	    
    //~ std::cout << "m_pose =" << m_pose << std::endl;
    //~ std::cout << "@computeErr m_pose X =" << m_pose(0,0) << std::endl;
    //~ std::cout << "@computeErr m_pose Y =" << m_pose(1,0) << std::endl;
    Eigen::Vector2d currentPosition(m_pose(0,0),m_pose(1,0));
    //~ std::cout << "@computeErr desiredPos =" << m_desiredPosition << std::endl;
    m_errorPosition = m_desiredPosition-currentPosition;
    //~ std::cout << "@computeErr SUBBED" << std::endl;
    m_errorPosition+=m_errorGain*m_errorPosition;
    //~ std::cout << "@computeErr MULED" << std::endl;
}
/**
 * @brief Trajectory::getErrorPosition
 * @return the error in position vector
 */
const Eigen::Vector2d Trajectory::getErrorPosition() {
    return m_errorPosition;
}
/**
 * @brief Trajectory::getDesiredPosition
 * @return the desired position vector
 */
const Eigen::Vector2d Trajectory::getDesiredPosition()
{
    return m_desiredPosition;
}
/**
 * @brief Trajectory::getRobotPose
 * @return  the robot pose vector
 */
const Eigen::Vector3d Trajectory::getRobotPose()
{
    return m_pose;
}
/**
 * @brief Trajectory::setGain sets the error gain.
 * @param gain
 */
void Trajectory::setGain(const double gain)
{
    m_errorGain = gain;
}
/**
 * @brief Trajectory::computeCommands compute the command with all the current values of
 * position, error and desired position
 * @return commands as vector
 */
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
/**
 * @brief Trajectory::addDPart as we control the point P, distant from d to the axle, we add the d part to the pose
 */
void Trajectory::addDPart() {
    m_pose(0,0)+=m_d*cos(m_pose(2,0)); // Hx = x + cos(th)
    m_pose(1,0)+=m_d*sin(m_pose(2,0)); // Hy = y + sin(th)
}
/**
 * @brief Trajectory::addDesiredDerivatives compute m_W which is the sum of
 * error postion and desired position derived.
 */
void Trajectory::addDesiredDerivatives() {
    m_W = m_errorPosition + m_desiredPositionDot;
}
/**
 * @brief Trajectory::rad2degree convert a values from rad to degree
 * @param radValue
 * @return
 */
double Trajectory::rad2degree(double radValue) {
    return (radValue*180.0)/M_PI;
}
/**
 * @brief Trajectory::degree2rad convert a values from degree to rad
 * @param degValue
 * @return
 */
double Trajectory::degree2rad(double degValue) {
    return (degValue*M_PI)/180.0;
}

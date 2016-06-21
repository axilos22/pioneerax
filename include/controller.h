/** @author Axel J. for ECN
  */
#ifndef CONTROLLER_H
#define CONTROLLER_H
//cpp
#include <iostream>
//eigen
#include <Eigen/Dense>
/**
 * @brief The Controller class allows to perform control computation to control the robot.
 */
class Controller
{
private:
    double m_kp,
    m_d;
    Eigen::Vector2d m_positionError;
    Eigen::Vector3d m_robotPose, m_initialPose;
public:
    /**
     * @brief Controller construtor create a new controller with these parameter
     * @param Kp Propotional gain
     * @param d distance to the controlled point
     */
    Controller(double Kp, double d=0);
    /**
     * @brief updateRobotPose update the pose of the robot
     * @param robotPose pose used for the update
     */
    void updateRobotPose(Eigen::Vector3d robotPose);
    /**
     * @brief computeError compute the error between current position and desired position
     * @param desiredPosition
     */
    void computeError(Eigen::Vector2d desiredPosition);
    /**
     * @brief computeCommands compute the commands to input to the robot
     * @return the command (v,w)
     */
    Eigen::Vector2d computeCommands(Eigen::Vector2d desiredPositionDot);
    void setInitialPose(double x0,double y0, double th0);
    Eigen::Vector2d getPositionError();
    ~Controller();
};

#endif // CONTROLLER_H

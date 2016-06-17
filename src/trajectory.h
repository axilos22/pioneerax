/** @author Axel JEANNE
 *
 */
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
//CPP
#include <vector>
#include <math.h>
#include <iostream>
//EIGEN
//~ #include <eigen3/Eigen/Dense>
//~ #include <eigen3/Eigen/Core>
//~ #include <Eigen/Eigen>
#include <Eigen/Dense>

class Trajectory
{
    enum Type {circle,square,other};
public:
    //parameter
    double m_gain, // the error gain
    m_d; //the small distance between the axle and the controlled point
    double m_radius, m_angularSpeed, m_time_s, m_time_ms;
    Eigen::Vector3d m_pose, m_initialPose;
    Eigen::Vector2d m_desiredPosition, m_errorPosition, m_desiredPositionDot, m_W;
    //functions
    Trajectory(double radius=1.0, double w=.1,double errGain=.1);
    ~Trajectory();
    void updateTime(const double time_s, const double time_ms=0);
    void setInitialPose(double x0, double y0, double th0);
    void setInitialPose(const std::vector<double> pos);
    void computeDesired();
    void updateRobotPose(const std::vector<double> pos);
    //~ void updateRobotPose(const Eigen::Vector2d position);
    void updateRobotPose(const Eigen::Vector3d pose);
    void computeError();
    const Eigen::Vector2d getErrorPosition();
    const Eigen::Vector2d getDesiredPosition();
    const Eigen::Vector3d getRobotPose();
    void setGain(const double gain);
    Eigen::Vector2d computeCommands();
    Eigen::Vector2d trajectorySequence(const double time_s, Eigen::Vector3d position);
    void addDPart();
    void addDesiredDerivatives();
    double rad2degree(double radValue);
    double degree2rad(double degValue);
};

#endif // ROBOTHANDLER_H


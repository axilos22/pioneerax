#ifndef TRAJECTORY_H
#define TRAJECTORY_H
//CPP
#include <vector>
#include <math.h>
//EIGEN
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

class Trajectory
{
    enum Type {circle,square,other};
public:
    //parameter
    double m_gain;
    double m_radius, m_angularSpeed, m_time_s, m_time_ms;
    Eigen::Vector3f m_pose, m_initialPose;
    Eigen::Vector2f m_desiredPosition, m_errorPosition;
    //functions
    Trajectory(double radius=1.0, double w=.1);
    ~Trajectory();
    void updateTime(const double time_s, const double time_ms=0);
    void setInitialPose(double x0, double y0, double th0);
    void setInitialPose(const std::vector<double> pos);
    void computeDesired();
    void updateRobotPose(const std::vector<double> pos);
    void computeError();
    const Eigen::Vector2f getErrorPosition();
    const Eigen::Vector2f getDesiredPosition();
    void setGain(const double gain);
};

#endif // ROBOTHANDLER_H


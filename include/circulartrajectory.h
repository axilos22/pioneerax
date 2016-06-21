/** @author Axel JEANNE
 */
#ifndef CIRCULARTRAJECTORY_H
#define CIRCULARTRAJECTORY_H
//CPP
#include <vector>
#include <math.h>
#include <iostream>
//EIGEN
#include <Eigen/Dense>
/**
 * @brief The CircularTrajectory class allows to create a circular trajectory for the robot to follow.
 * This class is used in the control loop to provide the desired position and its derivative.
 */
class CircularTrajectory
{
public:
    double m_radius, m_angularSpeed;
    Eigen::Vector2d m_desiredPosition,m_desiredPositionDot;
    CircularTrajectory(double radius, double angularSpeed);
    void computeDesired(const double time);
    const double radius();
    const double angularSpeed();
    const Eigen::Vector2d desiredPosition();
    const Eigen::Vector2d desiredPositionDot();
    ~CircularTrajectory();
};

#endif // ROBOTHANDLER_H

#pragma once

//STL
#include <vector>

#include "geometry/geometry.h"

class MotionEstimator
{
public:
    MotionEstimator () : thresh(3e-3), maxIter(300) {}
    MotionEstimator (double threshold) : thresh(threshold) {}
    virtual ~MotionEstimator () {}
    
    // 8 point algorithm
    Matrix3d computeEssential(const vector<Vector3d> & xVec1,
            const vector<Vector3d> & xVec2) const;
    
    // statistical method using as many points as possible
    Matrix3d estimateEssential(const vector<Vector3d> & xVec1,
            const vector<Vector3d> & xVec2) const;

    void estimateEssential(const vector<Vector3d> & xVec1,
            const vector<Vector3d> & xVec2,
            const vector<bool> & inlierMask,
            Matrix3d & E) const;
            
    void ransac(const std::vector<Vector3d> & cloud1,
            const vector<Vector3d> & cloud2,
            vector<bool> & inlierMask, Matrix3d & E) const;
    
    void extractMotion(const Matrix3d & E, vector<Transformation<double>> & xiVec) const;
    
    int pickGoodMotion(vector<Transformation<double>> & xiVec,
            const vector<Vector3d> & xVec1, const vector<Vector3d> & xVec2) const;
    
    double thresh;
    int maxIter;
};


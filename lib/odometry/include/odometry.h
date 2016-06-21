#pragma once

//STL
#include <vector>

#include <opencv2/opencv.hpp>

#include "geometry/geometry.h"
#include "camera/eucm.h"
#include "epipolar.h"

class MonocularOdometry
{
    enum Status {EMPTY, READY};
    
public:    
    MonocularOdometry (vector<double> cameraParams) : 
    camera(cameraParams.data()), status(EMPTY) {}
    virtual ~MonocularOdometry () {}
    Transformation<double> feedImage(const cv::Mat & img);
    
private:
    Status status;
    EnhancedCamera camera;
    cv::Mat desc1, img1;
    vector<Vector3d> cloud1;
    vector<cv::KeyPoint> keyPointVec1;
    MotionEstimator motion;
};


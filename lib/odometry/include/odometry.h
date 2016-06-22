#pragma once

//STL
#include <vector>

#include <opencv2/opencv.hpp>

#include "geometry/geometry.h"
#include "camera/eucm.h"
#include "epipolar.h"

#define UPDATE_EVERYTIME 0

//TODO add replace function to replace the ref image

class MonocularOdometry
{
    enum Status {EMPTY, READY};
    
public:    
    MonocularOdometry (vector<double> cameraParams) : 
    camera(cameraParams.data()), status(EMPTY) {}
    virtual ~MonocularOdometry () {}
    Transformation<double> feedImage(const cv::Mat & img);
    void pushImage();    
    
private:
    Status status;
    EnhancedCamera camera;
    cv::Mat desc1, desc2, img1, img2;
    vector<Vector3d> cloud1, cloud2;
    vector<cv::KeyPoint> keyPointVec1, keyPointVec2;
    vector<Vector2d> pointVec2;
    MotionEstimator motion;
};


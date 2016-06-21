#ifndef CAMHANDLER_H
#define CAMHANDLER_H
//CPP
#include <ctype.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cassert>
//Open CV
#include <opencv2/core/core.hpp>  //-- Contains structures and classes for holding and manipulating images
#include <opencv2/highgui/highgui.hpp> //-- Contains functions for displaying images on screen
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

class CamHandler
{
private:
    int m_cameraId;
    cv::VideoCapture m_vc;
    cv::Mat m_frame;
public:
    CamHandler(const int cameraId);
    ~CamHandler();
    void release();
    int changeResolution(double width,double height);
    int displayVideoStream();
    cv::VideoCapture getVideoCapture();
    int getCameraId();
    cv::Mat getFrame();
    double getParameter(std::string param);
    bool setParameter(std::string param, double value);
    bool grab();
    bool retrieve();
    bool isOpened();
    cv::Mat read();
};

#endif // CAMHANDLER_H

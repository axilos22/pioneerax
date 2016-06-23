#include <array>
#include <iostream>

#include <opencv2/opencv.hpp>
//#include <opencv2/features2d.hpp>

#include "../include/odometry.h"
#include "camera/eucm.h"

using namespace std;
using cv::Mat;
using cv::KeyPoint;
using cv::DMatch;

Transformation<double> MonocularOdometry::feedImage(const Mat & i_img2)
{
    cv::ORB orb;
    i_img2.copyTo(img2);
    orb(img2, Mat(), keyPointVec2, desc2);
    if (keyPointVec2.size() < 25) return Transformation<double>();

    vector<Vector2d> pointVec2;
    for (auto kpt : keyPointVec2)
    {
        pointVec2.emplace_back(kpt.pt.x, kpt.pt.y);
    }
    camera.reconstructPointCloud(pointVec2, cloud2);
    Transformation<double> result;
    switch (status)
    {
    case EMPTY:        
        status = READY;
        result = Transformation<double>();
        pushImage();
        break;
    case READY:
        cv::BFMatcher bfm(cv::NORM_HAMMING, true);
        vector<DMatch> matchVec;
        bfm.match(desc1, desc2, matchVec);
//        cout << cloud1.size() << " " << cloud2.size() << endl;
        vector<Vector3d> xVec1, xVec2;
        double thresh = 50;
        for (auto match : matchVec)
        {
            //~ cout << match.distance << endl;
            if (match.distance > thresh) continue;
            xVec1.push_back(cloud1[match.queryIdx]);
            xVec2.push_back(cloud2[match.trainIdx]);
        }
        /*Display the image matching*/
        //~ cv::Mat matchImg;
        //~ cv::drawMatches(img1, keyPointVec1, img2, keyPointVec2, matchVec, matchImg);
        //~ imshow("matchImg", matchImg);
       
        vector<bool> mask;
        Matrix3d E;
        motion.ransac(xVec1, xVec2, mask, E);
        motion.estimateEssential(xVec1, xVec2, mask, E);
        vector<Transformation<double>> xiVec2;
        motion.extractMotion(E, xiVec2);
        /*take the smallest norm among the transformations*/
        if (xiVec2[0].rot().norm() < xiVec2[1].rot().norm()) result = xiVec2[0];
        else result = xiVec2[1];
//        cout << result << endl;
        break;
    }
    return result;
}

void MonocularOdometry::pushImage() {
	cv::imwrite("newRef"+to_string(newRefFrameCounter)+".jpg",img2);
	newRefFrameCounter++;
	cloud1 = cloud2;
    desc2.copyTo(desc1);
    img2.copyTo(img1);
    keyPointVec1 = keyPointVec2;
}

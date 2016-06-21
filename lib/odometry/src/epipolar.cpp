#include <array>
#include <random>
#include <algorithm>
#include <iostream>
	
#include <Eigen/SVD>

#include "../include/epipolar.h"

using namespace std;
using Eigen::JacobiSVD;
using Eigen::ComputeFullU;
using Eigen::ComputeFullV;

Matrix3d MotionEstimator::computeEssential(const vector<Vector3d> & xVec1,
        const vector<Vector3d> & xVec2) const
{
    assert(xVec1.size() == xVec2.size());
    Matrix<double, 8, 9> A;
    
    for (int i = 0; i < 8; i++)
    {
        const double & x = xVec1[i][0];
        const double & y = xVec1[i][1];
        const double & z = xVec1[i][2];
        const double & u = xVec2[i][0];
        const double & v = xVec2[i][1];
        const double & w = xVec2[i][2];
        A.row(i) << x*u, x*v, x*w, y*u, y*v, y*w, z*u, z*v, z*w;
    }
    Eigen::FullPivLU<Matrix<double, 8, 9>> lu(A);
    Matrix<double, 9, 1> H = lu.kernel().col(0);
    Matrix3d E;
    E << H(0), H(1), H(2), H(3), H(4), H(5), H(6), H(7), H(8);
    return E;
}


Matrix3d MotionEstimator::estimateEssential(const vector<Vector3d> & xVec1,
        const vector<Vector3d> & xVec2) const
{
    assert(xVec1.size() == xVec2.size());
    Matrix<double, 9, 9> AtA = Matrix<double, 9, 9>::Zero();
    
    for (int i = 0; i < xVec1.size(); i++)
    {
        const double & x = xVec1[i][0];
        const double & y = xVec1[i][1];
        const double & z = xVec1[i][2];
        const double & u = xVec2[i][0];
        const double & v = xVec2[i][1];
        const double & w = xVec2[i][2];
        Matrix<double, 9, 1> A;
        A << x*u, x*v, x*w, y*u, y*v, y*w, z*u, z*v, z*w;
        AtA += A * A.transpose();
    }
    JacobiSVD<Matrix<double, 9, 9>> svd(AtA, ComputeFullV);
    //~ cout << "singular value : " << svd.singularValues()[8] << endl; 
    Matrix<double, 9, 1> H = svd.matrixV().col(8);
    Matrix3d E;
    E << H(0), H(1), H(2), H(3), H(4), H(5), H(6), H(7), H(8);
    return E;
}

void MotionEstimator::estimateEssential(const vector<Vector3d> & xVec1,
        const vector<Vector3d> & xVec2,
        const vector<bool> & inlierMask,
        Matrix3d & E) const 
{
    vector<Vector3d> xVecMasked1, xVecMasked2;
    for (int idx = 0; idx < xVec1.size(); idx++)
    {
        if (inlierMask[idx])
        {
            xVecMasked1.push_back(xVec1[idx]);
            xVecMasked2.push_back(xVec2[idx]);
        }
    }
    E = estimateEssential(xVecMasked1, xVecMasked2);
}

void MotionEstimator::ransac(const std::vector<Vector3d> & cloud1,
        const vector<Vector3d> & cloud2,
        vector<bool> & inlierMask, Matrix3d & Efinal) const
{
    int inlierCount = 8;
    
    vector<int> indexVec;
    for (int idx = 0; idx < cloud1.size(); idx++)
    {
        indexVec.push_back(idx);
    }
    inlierMask.assign(cloud1.size(), false);
    mt19937 g(0);
    for (int iter = 0; iter < maxIter; iter++)
    {
        //generate 6 random indices
        shuffle(indexVec.begin(), indexVec.end(), g);
        
        vector<Vector3d> xVec1, xVec2;
        for (auto iter = indexVec.begin(); iter != indexVec.begin() + 8; ++iter)
        {
            xVec1.push_back(cloud1[*iter]);
            xVec2.push_back(cloud2[*iter]);
        }
        
        // fit the model
        Matrix3d E = computeEssential(xVec1, xVec2);
        
        // count inliers
        vector<bool> inlierMaskEstim;
        int inlierCountEstim = 0;
        
        for (int idx = 0; idx < cloud1.size(); idx++)
        {
            const Vector3d & v1 = cloud1[idx];
            const Vector3d & v2 = cloud2[idx];
            double res = cloud1[idx].dot(E*cloud2[idx]);
            if (abs(res) < thresh)
            {
                inlierCountEstim++;
                inlierMaskEstim.push_back(true);
            }
            else
            {
                inlierMaskEstim.push_back(false);
            }
        }
        // refresh the best hypothesis
        if (inlierCountEstim > inlierCount)
        {
            inlierCount = inlierCountEstim;
            inlierMask = inlierMaskEstim;
            Efinal = E;
//            if (inlierCount > 30) break;
        }
    }
    //~ cout << "inlierCount : " << inlierCount << endl;
}

void MotionEstimator::extractMotion(const Matrix3d & E,
        vector<Transformation<double>> & xiVec) const
{
    JacobiSVD<Matrix3d> svd(E, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU(), V = svd.matrixV();
    
    // Motion vector estimation up to scale and sign
    Vector3d t = U.col(2);
    
    // Rotation estimation
    // Two rotations are possible: U R+-90 V^T
    Matrix3d R90;
    R90 <<  0,     -1,      0, 
            1,      0,      0,
            0,      0,      1;
    
    // to make sure that det(R) = 1
    double d = U.determinant() *  V.determinant();      
    Matrix3d R1 = d * U * R90 * V.transpose();
    Matrix3d R2 = d * U * R90.transpose() * V.transpose();
    Vector3d u1 = rotationVector(R1), u2 = rotationVector(R2);
    
    // store all possible motion hypotheses
    xiVec.emplace_back(t, u1);
    xiVec.emplace_back(t, u2);
    xiVec.emplace_back(-t, u1);
    xiVec.emplace_back(-t, u2);
}

// TODO put elswhere
// X minimizes the distance between two rays
// q*v1 and t + q*v2 for q > 0
bool triangulate(const Vector3d & v1, const Vector3d & v2,
        const Vector3d & t, Vector3d & X)
{
    //Vector3d v1n = v1 / v1.norm(), v2n = v2 / v2.norm();
    double v1v2 = v1.dot(v2);
    double v1v1 = v1.dot(v1);
    double v2v2 = v2.dot(v2);
    double tv1 = t.dot(v1);
    double tv2 = t.dot(v2);
    double delta = -v1v1 * v2v2 + v1v2 * v1v2;
    if (abs(delta) < 1e-4) // TODO the constant to be revised
    {
        X << 0, 0, 0;
        return false;
    }
    double l1 = (-tv1 * v2v2 + tv2 * v1v2)/delta;
    double l2 = (tv2 * v1v1 - tv1 * v1v2)/delta;
    if (l1 < 0 or l2 < 0)
    {
        X << 0, 0, 0;
        return false;
    }
    else
    {
        X = (v1*l1 + t + v2*l2)*0.5;
        return true;
    }
}

// TODO use the outlier mask
int MotionEstimator::pickGoodMotion(vector<Transformation<double>> & xiVec,
        const vector<Vector3d> & xVec1, const vector<Vector3d> & xVec2) const
{
    int idxBest = -1;
    int inlierBest = 0;
    for (int idx = 0; idx < xiVec.size(); idx++)
    {
        vector<Vector3d> xRotVec2;
        xiVec[idx].rotate(xVec2, xRotVec2);
        int inlierCount = 0;
        for (int i = 0; i < xVec1.size(); i++)
        {
            Vector3d X;
            if (triangulate(xVec1[i], xRotVec2[i], xiVec[idx].trans(), X))
            {
                inlierCount++;        
            }
        }
        if (inlierCount > inlierBest)
        {
            idxBest = idx;
            inlierBest = inlierCount;
        }
    }
    return idxBest;
}







#pragma once

#include <Eigen/Dense>
#include <string>
#include <map>

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;


struct LegScanComponent
{
    Matrix3D scan;
    Matrix3D master;

    std::map<std::string, Eigen::Vector3f> master_landmarks;
    std::map<std::string, Eigen::Vector3f> scan_landmarks;
};


struct LegScan
{
    std::string title;
    std::string date;
    
    LegScanComponent left_leg;
    LegScanComponent right_leg;
    LegScanComponent kneeling;
};
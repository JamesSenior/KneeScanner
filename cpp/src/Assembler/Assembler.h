#pragma once

#include <map>
#include <string>
#include <functional>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "common/StatusEvent.h"

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;


class Assembler
{
public:
    explicit Assembler(Matrix3D leftLeg, Matrix3D rightLeg, Matrix3D kneeling, std::map<std::string, Eigen::Vector3f> leftLandmarks, std::map<std::string, Eigen::Vector3f> rightLandmarks, std::map<std::string, Eigen::Vector3f> kneelingLandmarks, std::function<void(StatusEvent)> callback);

    void combine();
    void customize();
    
    Matrix3D getCombined(){return combined;}

private:
    // Scans
    Matrix3D m_left_leg;
    Matrix3D m_right_leg;
    Matrix3D m_kneeling;
    
    Matrix3D combined;

    // Landmarks for each scan
    std::map<std::string, Eigen::Vector3f> m_left_landmarks;
    std::map<std::string, Eigen::Vector3f> m_right_landmarks;
    std::map<std::string, Eigen::Vector3f> m_kneeling_landmarks;

    // Callback for status updates
    std::function<void(StatusEvent)> m_callback;
};

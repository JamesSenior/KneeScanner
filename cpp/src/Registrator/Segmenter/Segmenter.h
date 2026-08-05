#pragma once

#include <Eigen/Dense>
#include <functional>
#include <string>
#include <common/StatusEvent.h>
#include "nanoflann.hpp"

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;





class Segmenter 
{
public:
    //constructors
    Segmenter(Matrix3D points, std::function<void(StatusEvent)> callback);
    ~Segmenter();


    //setters and getters
    Matrix3D get_points(){return m_points;}


    //methods
    void ransac(float threshold = 0.01, int iterations = 2000);
    void euclidean_cluster(float threshold = 0.02);

private:

    //properties
    Matrix3D m_points;
    std::function<void(StatusEvent)> m_callback;


};
#pragma once

#include <Eigen/Dense>
#include <string>

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;


//function declirations
void writeToPLY(Matrix3D& points, std::string fileName);

Matrix3D readPLY(std::string fileName);
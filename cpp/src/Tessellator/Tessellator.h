#pragma once

#include <Eigen/Core>
#include <functional>

#include "common/StatusEvent.h"

// A point cloud with one XYZ point per row.
using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;

class Tessellator
{
public:
    explicit Tessellator(std::function<void(StatusEvent)> callback);

    // Supplies the scan points that a future reconstruction algorithm will use.
    void setPointCloud(const Matrix3D& points);
    
    void tessellate(float alpha, float beta);

private:
    Matrix3D m_points;
    std::function<void(StatusEvent)> m_callback;
};

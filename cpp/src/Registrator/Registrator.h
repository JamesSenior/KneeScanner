#pragma once

#include <common/StatusEvent.h>
#include <Eigen/Dense>
#include <functional>
#include <string>
#include "app/leg_scan.h"

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;


class Registrator {
public:
    Registrator(Matrix3D scan, Matrix3D master, std::map<std::string, Eigen::Vector3f> landmarks,std::function<void(StatusEvent)> callback);
    ~Registrator();

    Matrix3D get_scan(){return m_scan;}
    Matrix3D get_master(){return m_master;}
    std::map<std::string, Eigen::Vector3f> get_landmarks(){return m_landmarks;}
    void set_scan(Matrix3D scan){m_scan = scan;}
    void set_master(Matrix3D master){m_master = master;}
    void set_landmarks(std::map<std::string, Eigen::Vector3f> landmarks){m_landmarks = landmarks;}

    //methods
    void segment();
    void align();

private:
    //properties
    Matrix3D m_scan; //one we dont modify (target)
    Matrix3D m_master; //one we modify (source)
    std::map<std::string, Eigen::Vector3f> m_landmarks;
    std::function<void(StatusEvent)> m_callback;

    //helper methods
};

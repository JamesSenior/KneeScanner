#pragma once

#include <Eigen/Dense>
#include <functional>
#include <map>
#include <string>
#include <common/StatusEvent.h>
#include "nanoflann.hpp"

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;





class Aligner {
public:
    Aligner(Matrix3D scan, Matrix3D master, std::map<std::string, Eigen::Vector3f> landmarks, std::function<void(StatusEvent)> callback);
    ~Aligner();

    //setters and getters
    Matrix3D get_scan(){return m_scan;}
    Matrix3D get_master(){return m_master;}
    std::map<std::string, Eigen::Vector3f> get_landmarks(){return m_landmarks;}
    void set_scan(Matrix3D scan){m_scan = scan;}
    void set_master(Matrix3D master){m_master = master;}
    void set_landmarks(std::map<std::string, Eigen::Vector3f> landmarks){m_landmarks = landmarks;}

    //methods
    void segment();
    void initial_alignment();
    void icp_alignment(int max_iterations = 50, float convergence_threshold = 0.0169);
    void cpd_alignment(float alpha = 5.0f, float beta = 0.2f, float w = 0.05f, int max_iterations = 300, float tolarence = 2e-6f, int targetSize = 1500); //alpha = 0.2
    void Library_cpd();

private:
    //properties
    Matrix3D m_scan; //one we dont modify (target)
    Matrix3D m_master; //one we modify (source)
    std::map<std::string, Eigen::Vector3f> m_landmarks;
    std::function<void(StatusEvent)> m_callback;

    //helper funcs
    Matrix3D center(Matrix3D points);
    void pca();
    Matrix3D ransac(Matrix3D points, float threshold, int iterations);
    Matrix3D euclidean_cluster(Matrix3D points, float threshold = 0.02);
};

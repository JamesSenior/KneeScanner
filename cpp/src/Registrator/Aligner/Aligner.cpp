#include "Registrator/Aligner/Aligner.h"
#include <iostream>
#include <functional>
#include <string>
#include <unordered_map>
#include <tuple>
#include <cmath>
#include "nanoflann.hpp"
#include <common/StatusEvent.h>
#include "common/kd_tree.h"

//#include <cpd/src/nonrigid.hpp>


//constructors:
Aligner::Aligner(Matrix3D scan, Matrix3D master, std::map<std::string, Eigen::Vector3f> landmarks, std::function<void(StatusEvent)> callback)
{
    m_scan = scan;
    m_master = master;
    m_landmarks = landmarks;
    m_callback = callback;
}
Aligner::~Aligner(){}






//helper functions
Matrix3D get_pca_frame(Matrix3D points)
{
    //calculate covariance matrix
    Matrix3D cov = (points.transpose() * points) / (points.rows() - 1);

    //Compute the eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Matrix3D> solver(cov);

    Eigen::Vector3f eigvals = solver.eigenvalues();
    Matrix3D eigvecs = solver.eigenvectors();

    //reorder them
    Eigen::Matrix3f eigvecs_sorted;

    eigvecs_sorted.col(0) = eigvecs.col(2); // largest eigenvalue
    eigvecs_sorted.col(1) = eigvecs.col(1);
    eigvecs_sorted.col(2) = eigvecs_sorted.col(0).cross(eigvecs_sorted.col(1)); //do this instead to ensure it is not negitive

    return eigvecs_sorted;
}



float registrationError(const Matrix3D& source, const Matrix3D& target)
{
    //build kd tree
    PointCloudAdaptor adaptor(target);
    KDTree tree(3, adaptor);
    tree.buildIndex();


    float sum = 0.0f;

    for (int i = 0; i < source.rows(); i++)
    {
        const float query_pt[3] = {
        source(i,0),
        source(i,1),
        source(i,2)
        };

        uint32_t ret_index;
        float out_dist_sqr;

        tree.knnSearch(query_pt, 1, &ret_index, &out_dist_sqr);
        
        sum += out_dist_sqr;
    }

    return std::sqrt(sum / source.rows());
}







//helper methods
Matrix3D Aligner::center(Matrix3D points)
{
    Eigen::RowVector3f centroid;

    centroid = points.colwise().mean();
    points.rowwise() -= centroid;

    return points;
}


void Aligner::pca()
{
    const std::array<Eigen::Vector3i, 4> flips =   
    {{
    { 1,  1,  1},
    { 1, -1, -1},
    {-1,  1, -1},
    {-1, -1,  1}
    }};

    Matrix3D R_scan   = get_pca_frame(m_scan);
    Matrix3D R_master = get_pca_frame(m_master);

    float bestError = std::numeric_limits<float>::max();
    Matrix3D bestMaster;

    for (const auto& flip : flips)
    {
        Eigen::Matrix3f Rm = R_master;

        Rm.col(0) *= flip(0);
        Rm.col(1) *= flip(1);
        Rm.col(2) *= flip(2);

        Eigen::Matrix3f R = R_scan * Rm.transpose();

        Matrix3D rotated = m_master * R.transpose();

        float error = registrationError(rotated, m_scan);

        if (error < bestError)
        {
            bestError = error;
            bestMaster = rotated;
        }
    }

    m_master = bestMaster;

}







//***METHODS ***//



//INITIAL ALIGNMENT//
//center, pca
void Aligner::initial_alignment()
{
    //callback
    StatusEvent event;
    event.component = Component::Registrator;
    event.subcomponent = Component::Aligner;
    event.algorithm = Algorithm::PCA;
    event.level = LogLevel::NewUpdate;
    m_callback(event);

    
    //center
    m_master = center(m_master); //centers both scan and master to 0,0,0 on their respective com
    m_scan = center(m_scan);

    //pca
    pca(); //roughly alignes based on finding the axis of the clouds

    event.component = Component::Registrator;
    event.subcomponent = Component::Aligner;
    event.algorithm = Algorithm::PCA;
    event.level = LogLevel::Update;
    event.iteration = 1;
    event.max_iterations = 1;
    m_callback(event);
}



/*

//LIBRARY CPD//
void Aligner::Library_cpd()
{
    std::cout << "Starting library cpd" << std::endl;

    cpd::Matrix fixed = m_scan.cast<double>();
    cpd::Matrix moving = m_master.cast<double>();

    cpd::Nonrigid nonrigid;
    nonrigid.max_iterations(20);
    nonrigid.add_callback([](const cpd::Result& result)
    {
        std::cout << "CPD update iterations: " << result.iterations << std::endl;
    });

    cpd::NonrigidResult result = nonrigid.run(fixed, moving);

    m_master = result.points.cast<float>();

    std::cout << "ending library cpd" << std::endl;
}


*/

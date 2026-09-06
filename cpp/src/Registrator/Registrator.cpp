#include "Registrator/Registrator.h"
#include "Registrator/Aligner/Aligner.h"
#include "Registrator/Segmenter/Segmenter.h"
#include <common/StatusEvent.h>
#include <iostream>

//constructors:
Registrator::Registrator(Matrix3D scan, Matrix3D master, std::map<std::string, Eigen::Vector3f> landmarks ,std::function<void(StatusEvent)> callback)
    : m_scan(scan), m_master(master), m_landmarks(landmarks), m_callback(callback)
{}

Registrator::~Registrator(){}



//helper functions


//methods

void Registrator::segment()
{
    Segmenter scan_segmenter(m_scan, m_callback);
    scan_segmenter.ransac();
    scan_segmenter.euclidean_cluster();
    m_scan = scan_segmenter.get_points();

    Segmenter master_segmenter(m_master, m_callback);
    master_segmenter.ransac();
    master_segmenter.euclidean_cluster();
    m_master = master_segmenter.get_points();
}


void Registrator::align(float alpha, float beta, float w, int max_iterations, float tolarence, int targetSize, bool twoStep){
    
    //put landmark points at the end so they are accessable
    const int originalRows = m_master.rows();
    int row = originalRows;
     
    m_master.conservativeResize(originalRows + m_landmarks.size(), 3);

    for (const auto& [name, point] : m_landmarks) {
        m_master.row(row++) = point.transpose();
    }
    

    //create aligner object and run initiual, icp, and cpd alignments
    Aligner aligner(m_scan, m_master, m_landmarks, m_callback);
    aligner.initial_alignment();
    aligner.icp_alignment();
    aligner.cpd_alignment(alpha, beta, w, max_iterations, tolarence, targetSize);
    
    //if twostep is true run CPD twice, now run it again with a more localised deformation parameter 1/3rd Beta
    if(twoStep){
        aligner.cpd_alignment(alpha, beta/3, w, max_iterations, tolarence, targetSize);
    }

    //retrieve results from aligner
    m_scan = aligner.get_scan();
    m_master = aligner.get_master();
    
    //read landmark points from the end of master
    row = m_master.rows() - m_landmarks.size();
    for (auto& [name, point] : m_landmarks) {
        point = m_master.row(row++).transpose();
    }
}

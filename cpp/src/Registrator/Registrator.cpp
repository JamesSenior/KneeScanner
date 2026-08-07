#include "Registrator/Registrator.h"
#include "Registrator/Aligner/Aligner.h"
#include "Registrator/Segmenter/Segmenter.h"
#include <common/StatusEvent.h>
#include <iostream>

//constructors:
Registrator::Registrator(Matrix3D scan, Matrix3D master, std::function<void(StatusEvent)> callback)
    : m_scan(scan), m_master(master), m_callback(callback)
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


void Registrator::align(){

    Aligner aligner(m_scan, m_master, m_callback);
    aligner.initial_alignment();
    aligner.icp_alignment();
    //aligner.cpd_alignment();

    m_scan = aligner.get_scan();
    m_master = aligner.get_master();
}


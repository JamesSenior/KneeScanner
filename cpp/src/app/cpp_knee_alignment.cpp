#include "cpp_knee_alignment.h"
#include "Registrator/Registrator.h"
#include "io/PLYFile.h"
#include "common/StatusEvent.h"

#include <string>
#include <iostream>
#include <Eigen/Dense>
#include "common/leg_scan.h"

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;
using namespace std;

//constructors:
cpp_knee_alignment::cpp_knee_alignment(){}
cpp_knee_alignment::~cpp_knee_alignment(){}





//helper functions:

#include <random>

Eigen::Matrix3f randomRotation()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    // random Euler angles
    float rx = dist(gen) * 2.0f * M_PI;
    float ry = dist(gen) * 2.0f * M_PI;
    float rz = dist(gen) * 2.0f * M_PI;

    Eigen::AngleAxisf roll(rx, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch(ry, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw(rz, Eigen::Vector3f::UnitZ());

    return (yaw * pitch * roll).toRotationMatrix();
}

void applyTransform(Matrix3D& cloud,
                    const Eigen::Matrix3f& R,
                    const Eigen::Vector3f& t)
{
    cloud = cloud * R.transpose();   // rotation (row vectors)
    cloud.rowwise() += t.transpose(); // translation
}





//run function:

void cpp_knee_alignment::run()
{
    cout << "Starting Program\n";

    
    //get clouds:
    Matrix3D master = readPLY("rosieV2.ply");
    Matrix3D scan = readPLY("jamesV2.ply");  

    //combine and write
    Matrix3D test11(scan.rows() + master.rows(), 3);
    test11 << scan, master;
    writeToPLY(test11, "test11.ply");

    //segment
    Registrator registor(scan, master, callback);
    registor.segment();
    scan = registor.get_scan();
    master = registor.get_master();

    //combine and write
    Matrix3D test12(scan.rows() + master.rows(), 3);
    test12 << scan, master;
    writeToPLY(test12, "test12.ply");

    //align
    registor.align();
    scan = registor.get_scan();
    master = registor.get_master();

    //combine and write
    Matrix3D test13(scan.rows() + master.rows(), 3);
    test13 << scan, master;
    writeToPLY(test13, "test13.ply");


    cout << "\nProgram Finished\n";
}



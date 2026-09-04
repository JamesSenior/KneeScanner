#include "cpp_knee_alignment.h"
#include "Registrator/Registrator.h"
#include "io/PLYFile.h"
#include "common/StatusEvent.h"

#include <string>
#include <iostream>
#include <Eigen/Dense>
#include "leg_scan.h"

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

    
    //get all data:
    
    //name:
    std::string title = "Rosie Test";
    std::string date = "03/12/2008";
    
    //get clouds:
    Matrix3D leftLegScan = readPLY("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/LeftLeg_rosie.ply");
    Matrix3D rightLegScan = readPLY("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/RightLeg_rosie.ply");
    Matrix3D kneelingScan = readPLY("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/Kneeling_rosie.ply");
    
    //set up scan:
    LegScan leg_scan(title, date, callback);
    leg_scan.readInMasters(); //reads in master clouds and landmark points

    leg_scan.getLeftLeg().scan = leftLegScan;
    leg_scan.getRightLeg().scan = rightLegScan;
    leg_scan.getKneeling().scan = kneelingScan;
    
    
    //run process:
    leg_scan.create();
    
    
    //write output file:
    if(leg_scan.isCreated())
    {
        Matrix3D output = leg_scan.getCombined();
        writeToPLY(output, "output.ply");
        
        writeToPLY(leg_scan.getKneeling().master, "source.ply");
        writeToPLY(leg_scan.getKneeling().scan, "target.ply");
    }
    else
    {
        cout << "\nno output has been created";
    }
    
    
    
    cout << "\nProgram Finished\n";
    
}



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
    
    //cutomize scans:
    //leg_scan.customize();
    
    
    
    
    
    //write output file:
    if(leg_scan.isCreated())
    {
        Matrix3D output = leg_scan.getCombined();
        
        writeToPLY(output, "output.ply");
        
        /*
        writeToPLY(leg_scan.getLeftLeg().master, "LeftLegMaster.ply");
        writeToPLY(leg_scan.getLeftLeg().scan, "LeftLegScan.ply");
        
        writeToPLY(leg_scan.getRightLeg().master, "RightLegMaster.ply");
        writeToPLY(leg_scan.getRightLeg().scan, "RightLegScan.ply");
        
        writeToPLY(leg_scan.getKneeling().master, "KneelingMaster.ply");
        writeToPLY(leg_scan.getKneeling().scan, "KneelingScan.ply");
         */
    }
    else
    {
        cout << "\nno output has been created";
    }
    
    
    
    cout << "\nProgram Finished\n";
    
}



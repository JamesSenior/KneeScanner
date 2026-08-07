#include "common/StatusEvent.h"
#include "app/leg_scan.h"
#include "io/PLYFile.h"
#include "Registrator/Registrator.h"

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;




//helper functios:


std::map<std::string, Eigen::Vector3f> readLandmarks(const std::string& filename)
{
    std::map<std::string, Eigen::Vector3f> landmarks;

    // Open JSON file
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error("Could not open landmark file: " + filename);
    }

    // Parse JSON
    json data;
    file >> data;


    // Read each landmark
    for (const auto& item : data)
    {
        std::string label = item["label"];

        auto point = item["point"];

        Eigen::Vector3f position(
            point[0].get<float>(),
            point[1].get<float>(),
            point[2].get<float>()
        );

        landmarks[label] = position;
    }

    return landmarks;
}





//methods

void LegScan::create()
{
    //REGISTER:
    Registrator leftLegRegistor(left_leg.scan, left_leg.master, callback);
    //Registrator rightLegRegistor(right_leg.scan, right_leg.master, callback);
    //Registrator kneelingRegistor(kneeling.scan, kneeling.master, callback);
    
    //segment
    leftLegRegistor.segment();
    //rightLegRegistor.segment();
    //kneelingRegistor.segment();

    //align
    leftLegRegistor.align();
    //rightLegRegistor.align();
    //kneelingRegistor.align();
    
    //transfer
    
    
    
    //ASSEMBLE:
    
    
    //set output:
    Matrix3D temp(leftLegRegistor.get_scan().rows() + leftLegRegistor.get_master().rows(), 3);
    temp << leftLegRegistor.get_scan(), leftLegRegistor.get_master();
    combined = temp;
    
    
    std::cout << leftLegRegistor.get_scan().rows() << std::endl << leftLegRegistor.get_master().rows();
    
    //set created to true
    created = true;
}



void LegScan::readInMasters()
{
    //clouds:
    left_leg.master = readPLY("/Users/jamessenior/Desktop/Coding/KneeScanner/cpp/resources/masters/jamesV2.ply");
    //right_leg.master = readPLY("jamesV2.ply");
    //kneeling.master = readPLY("jamesV2.ply");
    
    //landmarks:
    left_leg.master_landmarks = readLandmarks("/Users/jamessenior/Desktop/Coding/KneeScanner/cpp/resources/masters/jamesV2.json");
    //right_leg.master_landmarks = readLandmarks("jamesV2.json");
    //kneeling.master_landmarks = readLandmarks("jamesV2.json");
    
    //call back
    StatusEvent event;
    event.component = Component::None;
    event.subcomponent = Component::None;
    event.algorithm = Algorithm::None;
    event.level = LogLevel::Info;
    event.message = "Read in 1 Master cloud and 1 Master landmark file";
    callback(event);
}








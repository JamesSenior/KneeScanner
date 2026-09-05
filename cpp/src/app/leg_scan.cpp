#include "common/StatusEvent.h"
#include "app/leg_scan.h"
#include "io/PLYFile.h"
#include "Registrator/Registrator.h"

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>

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
    
    //REGISTERS:
    Registrator leftLegRegistor(left_leg.scan, left_leg.master, left_leg.master_landmarks, callback);
    Registrator rightLegRegistor(right_leg.scan, right_leg.master, right_leg.master_landmarks, callback);
    Registrator kneelingRegistor(kneeling.scan, kneeling.master, kneeling.master_landmarks, callback);
    
    //each scan is run on seperate thread:
    //each thread runs segmentation and alignment algerithms
    std::thread t1([&]() {
        leftLegRegistor.segment();
        leftLegRegistor.align();
    });

    std::thread t2([&]() {
        rightLegRegistor.segment();
        rightLegRegistor.align();
    });

    std::thread t3([&]() {
        kneelingRegistor.segment();
        kneelingRegistor.align(0.3f, 3.0f, 0.01f, 100, 2e-9f, 4000, true);
    });

    // Wait for all three threads to finish
    t1.join();
    t2.join();
    t3.join();
    
    
    //get outputs:
    left_leg.master = leftLegRegistor.get_master();
    left_leg.scan = leftLegRegistor.get_scan();
    left_leg.scan_landmarks = leftLegRegistor.get_landmarks();
    
    right_leg.master = rightLegRegistor.get_master();
    right_leg.scan = rightLegRegistor.get_scan();
    right_leg.scan_landmarks = rightLegRegistor.get_landmarks();
    
    kneeling.master = kneelingRegistor.get_master();
    kneeling.scan = kneelingRegistor.get_scan();
    kneeling.scan_landmarks = kneelingRegistor.get_landmarks();
    
    
    //ASSEMBLE:
    
    
    
    
    
    
    
    //Combine the 2 to print out:
    //Matrix3D temp(leftLegRegistor.get_scan().rows() + leftLegRegistor.get_master().rows(), 3);
    //temp << leftLegRegistor.get_scan(), leftLegRegistor.get_master();
    
    
    //make the landmaerk points the combined matrix:
    Matrix3D temp(left_leg.scan_landmarks.size(), 3);

    int row = 0;
    for (const auto& [name, point] : left_leg.scan_landmarks) {
        temp.row(row++) = point.transpose();
    }
    
    combined = temp;
    
    
    //set created to true
    created = true;
}







void LegScan::readInMasters()
{
    //clouds:
    left_leg.master = readPLY("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/LeftLeg_james.ply");
    right_leg.master = readPLY("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/RightLeg_james.ply");
    kneeling.master = readPLY("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/Kneeling_james.ply");
    
    //landmarks:
    left_leg.master_landmarks = readLandmarks("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/LeftLeg_james.json");
    //right_leg.master_landmarks = readLandmarks("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/RightLeg_james.json");
    //kneeling.master_landmarks = readLandmarks("/Users/jamessenior/github/KneeScanner/cpp/resources/masters/Kneeling_james.json");
    
    //call back
    StatusEvent event;
    event.component = Component::None;
    event.subcomponent = Component::None;
    event.algorithm = Algorithm::None;
    event.level = LogLevel::Info;
    event.message = "Read in 3 Master cloud and 3 Master landmark file";
    callback(event);
}








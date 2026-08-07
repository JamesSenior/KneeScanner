#pragma once

#include <Eigen/Dense>
#include <string>
#include <map>
#include "common/StatusEvent.h"
#include <functional>

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;


struct LegScanComponent
{
    Matrix3D scan;
    Matrix3D master;

    std::map<std::string, Eigen::Vector3f> master_landmarks;
    std::map<std::string, Eigen::Vector3f> scan_landmarks;
};



class LegScan
{
public:
    
    //constructor:
    LegScan(
        const std::string& title,
        const std::string& date,
        std::function<void(StatusEvent)> callback
    )
        : title(title), date(date), m_callback(callback)
    {
    }
    
    
    //methods:
    void create();
    
    void readInMasters();
    
    
    //geters and setters
    LegScanComponent& getLeftLeg(){return left_leg;}
    LegScanComponent& getRightLeg(){return right_leg;}
    LegScanComponent& getKneeling(){return kneeling;}
    
    Matrix3D& getCombined(){return combined;}
    std::map<std::string, Eigen::Vector3f> combinedLandmarks(){return combined_landmarks;}
    
    std::string getTitle(){return title;}
    std::string getDate(){return date;}
    bool isCreated(){return created;}
    
    void setName(std::string text){title = text;}
    void setDate(std::string text){date = text;}
    
    
    
private:
    std::string title;
    std::string date;
    bool created = false;
    
    LegScanComponent left_leg;
    LegScanComponent right_leg;
    LegScanComponent kneeling;
    
    Matrix3D combined;
    std::map<std::string, Eigen::Vector3f> combined_landmarks;
    
    std::function<void(StatusEvent)> m_callback;
};

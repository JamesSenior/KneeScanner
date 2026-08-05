#include "StatusEvent.h"
#include <iostream>



using namespace std;

//callback
void callback(StatusEvent message)
{
    switch (message.level)
    {
        case LogLevel::Info:
        {
        // Handle info message
            cout << message;
            break;
        }

        case LogLevel::Warning:
        {
        // Handle warning message
            cout << message;
            break;
        }

        case LogLevel::Error:
        {
        // Handle error message
            cout << message;
            break;
        }

        case LogLevel::Update:
        {
        // Handle error message
            std::cout << "\rIteration: " << message.iteration << " / " << message.max_iterations << "  | " << message.value << std::flush;
            break;
        }

        case LogLevel::NewUpdate:
        {
        // Handle error message
            cout << "\nStarting: " << message.component << "/" << message.subcomponent << "/" << message.algorithm << std::endl;
            break;
        }

        default:
        {
        // Unknown log level
            cout << "Warning: unknown callback logged\n";
            cout << message;
            break;
        }
    }
}






std::ostream& operator<<(std::ostream& os, LogLevel level)
{
    switch (level)
    {
        case LogLevel::Info:
            return os << "Info";

        case LogLevel::Warning:
            return os << "Warning";

        case LogLevel::Error:
            return os << "Error";

        case LogLevel::Update:
            return os << "Update";

        case LogLevel::NewUpdate:
            return os << "New Update";
    }

    return os << "Unknown Log Level";
}


std::ostream& operator<<(std::ostream& os, Component component)
{
    switch (component)
    {
        case Component::None:
            return os << "None";

        case Component::Registrator:
            return os << "Registrator";

        case Component::Aligner:
            return os << "Aligner";

        case Component::Segmentor:
            return os << "Segmentor";

        case Component::Assembler:
            return os << "Assembler";

        case Component::IO:
            return os << "IO";
    }

    return os << "Unknown Component";
}





std::ostream& operator<<(std::ostream& os, Algorithm algorithm)
{
    switch (algorithm)
    {
        case Algorithm::None:
            return os << "None";

        case Algorithm::RANSAC:
            return os << "RANSAC";

        case Algorithm::EuclideanCluster:
            return os << "Euclidean Cluster";

        case Algorithm::PCA:
            return os << "PCA";

        case Algorithm::ICP:
            return os << "ICP";

        case Algorithm::CPD:
            return os << "CPD";
    }

    return os << "Unknown Algorithm";
}



std::ostream& operator<<(std::ostream& os, const StatusEvent& event)
{
    os << "[" << event.level << "] "
       << event.component << event.subcomponent;

    if (event.algorithm != Algorithm::None)
        os << " | " << event.algorithm;

    if (event.iteration >= 0)
        os << " | Iteration: " << event.iteration << " / " << event.max_iterations;

    if (event.value >= 0)
        os << " | Error: " << event.value;

    os << "Message: " << event.message << std::endl;

    return os;
}

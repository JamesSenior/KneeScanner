#pragma once

#include <string>
#include <ostream>


enum class LogLevel
{
    Info,
    Warning,
    Error,
    Update,
    NewUpdate
};

enum class Component
{
    None,
    Registrator,
    Aligner,
    Segmentor,
    Assembler,
    IO
};

enum class Algorithm
{
    None,
    RANSAC,
    EuclideanCluster,
    PCA,
    ICP,
    CPD
};

struct StatusEvent
{
    Component component = Component::None;
    Component subcomponent = Component::None;
    Algorithm algorithm = Algorithm::None;
    int iteration = -1;
    int max_iterations = -1;
    float value = -1.0f;
    std::string message = "";
    LogLevel level = LogLevel::Info;
};



void callback(StatusEvent message);

std::ostream& operator<<(std::ostream& os, LogLevel level);
std::ostream& operator<<(std::ostream& os, Component component);
std::ostream& operator<<(std::ostream& os, Algorithm algorithm);
std::ostream& operator<<(std::ostream& os, const StatusEvent& event);
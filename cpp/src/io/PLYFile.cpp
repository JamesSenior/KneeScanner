#include "io/PLYFile.h"
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <cstring>

using Matrix3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;



//read helper functions:

struct PlyInfo {
    size_t vertex_count;
    bool big_endian;
    size_t bytes_to_ignore;
};

PlyInfo parse_header(std::ifstream& file) {
    PlyInfo info{};
    std::string line;

    info.bytes_to_ignore = 0;

    while (std::getline(file, line)) {

        if (line.find("element vertex") != std::string::npos) {
            info.vertex_count = std::stoul(line.substr(15));
        }

        if (line == "format binary_big_endian 1.0")
            info.big_endian = true;

        if (line == "format binary_little_endian 1.0")
            info.big_endian = false;

        // Count bytes after x,y,z
        if (line.rfind("property ", 0) == 0) {

            static int property_count = 0;
            property_count++;

            if (property_count > 3) {

                if (line.find("float") != std::string::npos)
                    info.bytes_to_ignore += 4;
                else if (line.find("uchar") != std::string::npos)
                    info.bytes_to_ignore += 1;
                else if (line.find("char") != std::string::npos)
                    info.bytes_to_ignore += 1;
                else if (line.find("ushort") != std::string::npos)
                    info.bytes_to_ignore += 2;
                else if (line.find("short") != std::string::npos)
                    info.bytes_to_ignore += 2;
                else if (line.find("uint") != std::string::npos)
                    info.bytes_to_ignore += 4;
                else if (line.find("int") != std::string::npos)
                    info.bytes_to_ignore += 4;
                else if (line.find("double") != std::string::npos)
                    info.bytes_to_ignore += 8;
            }
        }

        if (line == "end_header")
            break;
    }

    return info;
}


static float read_float(std::ifstream& file, bool big_endian) {
    uint32_t bytes;
    file.read(reinterpret_cast<char*>(&bytes), sizeof(uint32_t));

    if (big_endian) {
        bytes =
            ((bytes & 0x000000FF) << 24) |
            ((bytes & 0x0000FF00) << 8)  |
            ((bytes & 0x00FF0000) >> 8)  |
            ((bytes & 0xFF000000) >> 24);
    }

    float value;
    std::memcpy(&value, &bytes, sizeof(float));
    return value;
}


Matrix3D read_vertices(std::ifstream& file, size_t N)
{
    Matrix3D points(N, 3);

    for (size_t i = 0; i < N; i++)
    {
        float x = read_float(file, false);
        float y = read_float(file, false);
        float z = read_float(file, false);

        points(i,0) = x;
        points(i,1) = y;
        points(i,2) = z;

        // ALWAYS skip RGB (3 bytes)
        //file.ignore(3);
    }

    return points;
}







//write helper functions:

static uint32_t float_to_big_endian(float value)
{
    uint32_t bytes;
    std::memcpy(&bytes, &value, sizeof(float));

    bytes =
        ((bytes & 0x000000FF) << 24) |
        ((bytes & 0x0000FF00) << 8)  |
        ((bytes & 0x00FF0000) >> 8)  |
        ((bytes & 0xFF000000) >> 24);

    return bytes;
}





//write to file
void writeToPLY(Matrix3D& points, std::string fileName)
{
    std::ofstream file("../resources/output/" + fileName, std::ios::binary);

    if (!file)
        throw std::runtime_error("Cannot create PLY file");

    size_t N = points.rows();

    // -------------------
    // HEADER
    // -------------------
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";   // CHANGED
    file << "element vertex " << N << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    // -------------------
    // DATA
    // -------------------
    for (size_t i = 0; i < N; i++)
    {
        float x = points(i, 0);   // CHANGED (no uint32_t)
        float y = points(i, 1);
        float z = points(i, 2);

        file.write(reinterpret_cast<char*>(&x), sizeof(float)); // CHANGED
        file.write(reinterpret_cast<char*>(&y), sizeof(float));
        file.write(reinterpret_cast<char*>(&z), sizeof(float));
    }
}




//read from file
Matrix3D readPLY(std::string fileName)
{
    std::ifstream file("../resources/masters/" + fileName, std::ios::binary);

    if (!file)
        throw std::runtime_error("Cannot open PLY file");

    auto info = parse_header(file);

    return read_vertices(file,
                         info.vertex_count);
}
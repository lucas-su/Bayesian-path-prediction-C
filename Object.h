#pragma once

class Object
{
private:
    Eigen::Matrix3d poscov;
    // Eigen::Vector3d velcov;
    // Eigen::Vector3d acccov;
public:
    int id;
    float prior;
    Eigen::Vector3d size;
    Eigen::Vector3d pos;
    Object(std::vector<float> position, std::vector<int> size, int id, float prior);
    ~Object();

    float posterior;
    
    double likelihood_pos(Eigen::Vector3d newpos);
};
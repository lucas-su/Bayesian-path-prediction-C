#pragma once

class Object
{
private:
    int id;
    
    Eigen::Vector3d poscov;
    Eigen::Vector3d velcov;
    Eigen::Vector3d acccov;
public:
    float prior = prior;
    Eigen::Vector3d size = size;
    Eigen::Vector3d pos;
    Object(std::vector<float> position, std::vector<int> size, int id, float prior);
    ~Object();

    float posterior = prior;

    float likelihood_pos(Eigen::Vector3d newpos);
};
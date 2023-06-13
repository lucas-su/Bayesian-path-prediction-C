#include "BPP.h"
#include "Object.h"

Object::Object(std::vector<float>position,  std::vector<int> size, int id, float prior)
{   
    pos << position[0], position[1], position[2];
    size = size;
    this->id = id;
    // pos(position.data());
    // std::cout << "object position from obj object: " << pos << std::endl;
    prior = prior;
    posterior = prior;
    std::cout << "posterior set as: " << posterior << std::endl;
    object_likelihoods.push_back(0); // add entry to object_likelihoods for every object
}

Object::~Object()
{
}

double Object::likelihood_pos(Eigen::Vector3d newpos){
    poscov << 2, 0, 0,
           0, 2, 0,
           0, 0, 2;
    return gaussnd(newpos, pos, poscov);
}
#include "BPP.h"
#include "Object.h"

Object::Object( std::vector<float> position,  std::vector<int> size, int id, float prior)
{
}

Object::~Object()
{
}

float Object::likelihood_pos(Eigen::Vector3d newpos){
    // return multivariate_normal.pdf(posx, self.pos, self.poscov)
}
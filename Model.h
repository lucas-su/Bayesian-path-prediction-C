#pragma once
#include "BPP.h"
#include "Object.h"

class Model 
{
private:

    
    Eigen::Vector3d robot_pos;
    Eigen::Vector3d robot_vel;
    Eigen::Matrix3d cov_Fo;
    // Cholesky chol;

    std::vector<Object*> objects;
    float m;

    std::vector<Eigen::Vector3d> Fo = std::vector<Eigen::Vector3d>(Nsteps+1);
    Eigen::Vector3d propFo;

    std::vector<Eigen::Vector3d> pos = std::vector<Eigen::Vector3d>(Nsteps+1);
    std::vector<Eigen::Vector3d> vel = std::vector<Eigen::Vector3d>(Nsteps+1);
    std::vector<float> logp = std::vector<float>(Nsteps+1);

    std::vector<Eigen::Vector3d> best_pos;
    std::vector<Eigen::Vector3d> best_vel;
    std::vector<float> best_logp = std::vector<float>(Nsteps+1);
    
    int K {10};
    int B {2};

    bool collision;
    
public:
    Model(Eigen::Vector3d robot_pos, Eigen::Vector3d robot_vel, int operator_m, std::vector<Object*> objects);
    ~Model();
    
    double likelihood_vel(Eigen::Vector3d);
    
    // float Model::multivariate_normal(Eigen::Vector3d x, Eigen::Vector3d mean, Eigen::Vector3d cov);
    float MVN_p(Eigen::Vector3d x, Eigen::Vector3d mean, Eigen::Vector3d cov);
    
    
    void step(int t, int p);
    Eigen::Vector3d Fo_t;

    Eigen::Vector3d get_acc(Eigen::Vector3d Fe, Eigen::Vector3d Fo, int m);

    void set_Fe(int t);
    Eigen::Vector3d Fe;

    void set_Fo(Eigen::Matrix3d);
    std::normal_distribution<> Fo_dist {0, 1};
    

    Eigen::Vector3d newacc;
    Eigen::Vector3d newpos;
    Eigen::Vector3d newvel;

    float logp_ObjPosterior;
    float logp_Pos;
    float logp_Vel;
    float logp_newpos;

    std::vector<float> object_likelihoods;
    
};
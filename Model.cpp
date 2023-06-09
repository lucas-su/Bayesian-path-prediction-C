#include "BPP.h"
#include "Object.h"
#include "Model.h"
// #include "cholesky.cpp"

Model::Model(Eigen::Vector3d initial_pos, Eigen::Vector3d initial_vel, int operator_m, std::vector<Object*> object_list)
{
    cov_Fo <<  0.3, 0, 0, 
               0, 0.3, 0,
               0, 0, 0.3;
    pos.at(0) = initial_pos;    
    vel.at(0) = initial_vel;
    objects = object_list;
    for (int i=0;i<Niters;i++){
        std::cout << "iter " << i << std::endl;
        for (int j=0;j<Nsteps;j++){
            set_Fo(cov_Fo);
            for (int k=0;k<Nsteps;k++){
                step(k, j);
            }
            // calculate posterior for every object
            if ((std::accumulate(logp.begin(), logp.end(),0))>(std::accumulate(best_logp.begin(), best_logp.end(),0.))){
                std::cout << "found new best path with log prob " << std::accumulate(logp.begin(), logp.end(),0) << std::endl;
                Fo[j]=propFo;
                best_logp =logp;
                best_pos = pos;
                best_vel = vel;

                float obj_pos_sum = 0; //-1.e-038; // this represents -infinity
                for (auto obj_ptr:objects)   {
                    Object obj = *obj_ptr;
                    for (auto pos_t:pos)   {
                        obj_pos_sum = obj_pos_sum + std::log(obj.likelihood_pos(pos_t));
                    }
                    object_likelihoods.push_back(std::exp(std::log(obj.prior)+obj_pos_sum));
                }
                // char mlo = ;
                std::cout << "Most likely object: " << *std::max_element(object_likelihoods.begin(), object_likelihoods.end()) << std::endl;
            }
        }
    }
}


Model::~Model()
{
}


double Model::likelihood_vel(Eigen::Vector3d newvel){
    // std::cout<<"like_vel"<<std::endl;
    Eigen::Vector3d mean;
    mean << 0.,0.,0.;
    Eigen::Matrix3d cov;
    cov << 2, 0, 0,
           0, 2, 0,
           0, 0, 2;
    Eigen::Vector3d mu;
    mu << 0,0,0;
    // std::cout << newvel.size() << mu.size() << cov.size() << std::endl;
    return gaussnd(newvel, mu, cov);
    // return 1.;
}



// float Model::MVN_p(Eigen::Vector3d x, Eigen::Vector3d mean, Eigen::Vector3d cov)
// {
    
    
// }


void Model::step(int t, int p){

    if (t==p){
        Fo_t = propFo;
    } else {
        Fo_t = Fo.at(t);
    }
    Model::set_Fe(t);
    newacc = Model::get_acc(Fe, Fo_t, operator_m);
    newpos = pos.at(t) + vel.at(t) + newacc;
    newvel = newpos - pos[t];
    logp_newpos = -1.e-038; // this represents -infinity
    
    for (auto it:objects)   {
        Object obj = *it;
        logp_ObjPosterior = obj.posterior;
        logp_Pos = obj.likelihood_pos(newpos);
        logp_Vel = likelihood_vel(newvel);
        logp_newpos = logaddexpf(logp_newpos, logp_ObjPosterior + logp_Pos + logp_Vel);
    }
    logp.at(t + 1) = logp_newpos;
    pos.at(t + 1) = newpos;
}

void Model::set_Fe(int t){
    Fe << 0,0,0;
    
    for (auto it:objects){
        Object obj = *it;
        collision = 1; 
        for (int i=0; i<3; i++){
            if (((pos[t]+vel[t])(i) < (obj.pos+(0.5*obj.size))(i))&&((pos[t]+vel[t])(i) > (obj.pos-(0.5*obj.size))(i))){
                collision = 0;
            }
        }
        if (collision){
            for (int i=0; i<3; i++){
                Fe(i) = -K * vel[t](i) - B;
            }
        }
    }
}

void Model::set_Fo(Eigen::Matrix3d cov){
    
    Eigen::Vector3d x;
    for (int i=0; i<3;i++){
        x[i]= Fo_dist(randflt);
    }
    Eigen::LLT<Eigen::MatrixXd> lltOfA(cov);
    Eigen::MatrixXd L = lltOfA.matrixL();
    propFo = L * x;
    // std::cout << "propFo" << propFo << std::endl;
}

Eigen::Vector3d Model::get_acc(Eigen::Vector3d Fe, Eigen::Vector3d Fo, int m){
    Eigen::Vector3d acc = (Fe+Fo)/m;
    for (int i=0; i<3; i++){
        acc(i) = std::min(acc(i),max_acc);
    }
    return acc;
}
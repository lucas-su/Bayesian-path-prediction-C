#include "BPP.h"
#include "Object.h"
#include "Model.h"
// #include "cholesky.cpp"

Model::Model(Eigen::Vector3d initial_pos, Eigen::Vector3d initial_vel, int operator_m, std::vector<Object*> object_list)
{
    cov_Fo <<  5., 0, 0, 
               0, 5., 0,
               0, 0, 5.;
    pos.at(0) = initial_pos;    
    vel.at(0) = initial_vel;
    objects = object_list;
    for (int i=0;i<Niters;i++){
        std::cout << "iteration " << i << std::endl;
        for (int j=0;j<Nsteps;j++){
            // std::cout << "updating force in step: " << j << std::endl;
            set_Fo(cov_Fo);
            // std::cout << "propFo" << propFo << std::endl;
            for (int k=j;k<Nsteps;k++){
                // std::cout << "calculating new position in step: " << k << std::endl;
                step(k, j);
            }
            // calculate posterior for every object

            // std::cout << "new logp" << (std::accumulate(logp.begin(), logp.end(),init)) << " old logp "<< (std::accumulate(best_logp.begin(), best_logp.end(),init)) << std::endl;
            if ((std::accumulate(logp.begin(), logp.end(),init))>(std::accumulate(best_logp.begin(), best_logp.end(),init))){
                std::cout << "found new best path with log prob " << std::accumulate(logp.begin(), logp.end(),init) << std::endl;
                Fo[j]=propFo;
                best_logp = logp;
                std::cout << "best logp now set to " << std::accumulate(best_logp.begin(), best_logp.end(),init) << std::endl;
                best_pos = pos;
                best_vel = vel;

                
                for (auto obj_ptr:objects)   {
                    float obj_pos_sum = -1.e38; // this represents -infinity
                    Object obj = *obj_ptr;
                    for (auto pos_t:pos)   {
                        obj_pos_sum = logaddexpf(obj_pos_sum, std::log(obj.likelihood_pos(pos_t)));
                    }
                    object_likelihoods[obj.id] = std::exp(std::log(obj.prior)+obj_pos_sum);
                }
                
                std::cout << "Most likely object: " << std::distance(object_likelihoods.begin(),std::max_element(object_likelihoods.begin(), object_likelihoods.end())) << std::endl;
                } else { // if updated force is not an improvement, revert back to previous best
                // std::cout << "Did not find new best route, reverting" << std::endl;
                logp = best_logp;
                pos = best_pos;
                vel = best_vel;
            }
        }
    }
}


Model::~Model()
{
}


double Model::likelihood_vel(Eigen::Vector3d newvel){
    // std::cout<<"like_vel"<<std::endl;
    
    mean_vel << 0.,0.,0.;
    cov_vel << 2, 0, 0,
           0, 2, 0,
           0, 0, 2;

    // std::cout << newvel.size() << mean_vel.size() << cov_vel.size() << std::endl;
    return gaussnd(newvel, mean_vel, cov_vel);
}

void Model::step(int t, int p){

    if (t==p){
        Fo_t = propFo;
    } else {
        Fo_t = Fo.at(t);
    }
    
    Model::set_Fe(t);
    newacc = Model::get_acc(Fe, Fo_t, operator_m);
    // std::cout << "newacc " << newacc << std::endl;
    // std::cout << "t: " << t << std::endl;
    
    // std::cout << "pos, vel, newacc " << pos.at(t) << vel.at(t) << newacc << std::endl;


    newpos = pos.at(t) + vel.at(t) + newacc; 
    // std::cout << "newpos " << newpos << std::endl;

    newvel = (newpos - pos.at(t))*deltaT;
    logp_newpos = -1.e38; // this represents -infinity
    
    for (auto it:objects) {
        Object obj = *it;
        logp_ObjPosterior = log(obj.posterior);
        // std::cout << "logp_obj: " << logp_ObjPosterior << std::endl;
        logp_Pos = log(obj.likelihood_pos(newpos));
        // std::cout << "robot position: " << newpos << "  object position: " << obj.pos << "  logp_pos: " << logp_Pos << std::endl;
        // std::cout << "logp_pos: " << logp_Pos << std::endl;
        logp_Vel = log(likelihood_vel(newvel));
        // std::cout << "logp_vel: " << logp_Vel << std::endl;
        logp_newpos = logaddexpf(logp_newpos, logp_ObjPosterior + logp_Pos + logp_Vel);
        // std::cout << "logp_newpos: " << logp_newpos << std::endl;
    }
    logp.at(t + 1) = logp_newpos;
    pos.at(t + 1) = newpos;
    vel.at(t + 1) = newvel;
}

void Model::set_Fe(int t){
    Fe << 0,0,0;
    
    for (auto it:objects){
        Object obj = *it;
        collision = 1; 
        for (int i=0; i<3; i++){
            if (((pos[t]+vel[t])(i) < (obj.pos+(0.5*obj.size))(i)) && ((pos[t]+vel[t])(i) > (obj.pos-(0.5*obj.size))(i))){
                collision = 0;
            }
        }
        if (collision){
            for (int i=0; i<3; i++){
                Fe(i) = (-K * vel[t](i)) - B;
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
    acc = ((Fe+Fo)/m)*deltaT;
    for (int i=0; i<3; i++){
        if (acc(i)>0){
            acc(i) = std::min(acc(i),max_acc);
        } else {
            acc(i) = std::max(acc(i),-max_acc);
        }
        
    }
    return acc; // m/s/step
}
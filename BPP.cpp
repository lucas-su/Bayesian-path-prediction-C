#include "BPP.h"
#include "Model.cpp"
#include "Object.cpp"
#include "plot_figs.cpp"

int main(){
    std::vector<Object*> object_list;

    for (int i=0; i < object_size.size();i++){
        object_list.push_back(new Object(object_positions[i], object_size, i, object_priors[i]));
        
        // std::cout << "object location: " << object_list[i]->pos << std::endl;
    }
    initial_pos << 0,0,0;
    initial_vel << 0,0,0;
    Model* model = new Model(initial_pos, initial_vel, operator_m, object_list);
    
}


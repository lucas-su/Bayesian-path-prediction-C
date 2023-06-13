#include "../BPP.h"

double gaussnd(Eigen::Vector3d x, Eigen::Vector3d mu, Eigen::Matrix3d cov) {
    
    Eigen::Vector3d diff = x - mu;
    Eigen::LLT<Eigen::MatrixXd> lltOfA(cov);
    int k = lltOfA.cols();
    // std::cout << "llt size: " << lltOfA.cols() << std:: endl;
    
    double sum=0;
    Eigen::MatrixXd L = lltOfA.matrixL();
    Eigen::MatrixXd LLT = lltOfA.matrixLLT();
    for (int i=0; i<k; ++i){
        // std::cout << "sum: " << sum << std::endl;
        sum += log(L.coeff(i,i));
    }
    // std::cout << "matrix L: " << L << std::endl;
    double logdet = 2*sum;
    // std::cout << "logdet : " << logdet << std::endl;
    // std::cout << "diff: "<< diff << std::endl;
    // std::cout << "LTT.inv: "<< LLT.inverse() << std::endl;
    // std::cout << "dot 1:" << diff.adjoint()*(LLT.inverse()) << std::endl;
    // std::cout << "dot 2:" << (diff.adjoint()*LLT.inverse())*diff << std::endl;
    // std::cout << (diff.transpose().dot(LLT.inverse())).dot(diff) << std::endl;
    // double y = (diff.adjoint()*LLT.inverse()).adjoint()*diff;    
    
    // std::cout << "gaussnd term 1: "  << 1/(pow((2*M_PI),(k/2))*exp(.5*logdet)) << std::endl;
    // std::cout << "gaussnd term 2: " << exp(-.5*(diff.transpose()*LLT.inverse())*diff) << std::endl;

    return 1/(pow((2*M_PI),(k/2))*exp(.5*logdet)) * exp(-.5*(diff.transpose()*LLT.inverse())*diff);
    // return 1.;
}


// exp(-.5*
    // inprod(
    //     multiply(
    //         diff,c.inverse()),
    //     diff
    // )
// )
#include "../BPP.h"


struct Cholesky {
  int n;
  std::vector<std::vector<double>> el;


  // Constructor: decompose a into el' x el
  int decompose(std::vector<std::vector<double>> a) {
    int rows = a.size(), cols=a[0].size();
    assert(rows == cols);
      
    n = rows;
    el = a;

    int i,j,k;
    std::vector<double> tmp;
    double sum;

    for (i=0; i<n; ++i) {
      for (j=0; j<n; ++j) {
        for (sum=el[i][j], k=i-1; k>=0; --k)
          sum -= el[i][k]*el[j][k];
        if (i==j) {
          if (sum<=0)
            return 1;           // Decomposition failed, not posdef
          el[i][j] = sqrt(sum);
        } else
          el[j][i] = sum/el[i][i];
      }
    }
    for (i=0; i<n; ++i)
      for (j=0; j!=i; ++j)
        el[j][i] = 0.0;

    return 0;                   // Success.
  }

  // Solve A x = b, returning x
  std::vector<double> solve(std::vector<double> b) {
    int i,k;
    double sum;

    if (b.size() != n) {
      std::cout<< "Cholesky solve: b is of wrong length" << std::endl;
      std::vector<double> ret = {0};
      return ret;
    }

    std::vector<double> x(n);
    
    for (i=0; i<n; ++i) {
      for (sum=b[i],k=i-1; k>=0; --k)
        sum -= el[i][k]*x[k];
      x[i] = sum/el[i][i];
    }
    for (i=n-1; i>=0; --i) {
      for (sum=x[i], k=i+1; k<n;++k)
        sum -= el[k][i]*x[k];
      x[i] = sum/el[i][i];
    }
    return x;    
  }
  
  // For L y = b, return y (where L'L=A)
  std::vector<double> elSolve(std::vector<double> b) {
    int i,j;
    double sum;

    if (b.size() != n)
      std::cout << "Error: vector length does not match!" << std::endl;

    std::vector<double> y(n);
    
    for (i=0; i<n; ++i) {
      for (sum=b[i],j=0; j<i; ++j)
        sum -= el[i][j] * y[j];
      y[i] = sum/el[i][i];
    }
    
    return y;
  }

  // For L' y = b, return y (where L'L=A)
  std::vector<double> elTransSolve(std::vector<double> b) {
    int i,j;
    double sum;

    if (b.size() != n)
      std::cout << "Error: vector length does not match!" << std::endl;

    std::vector<double> y(n);
    
    for (i=n-1; i>=0; --i) {
      // write("i=",i);
      for (sum=b[i],j=i+1; j<n; ++j) {
        std::cout << ("j=",j)<< std::endl;
        // std::cout << ("el=", el)<< std::endl;
        // std::cout << ("y=", y)<< std::endl;
        sum -= el[j][i] * y[j];
      }
      y[i] = sum/el[i][i];
    }
    
    return y;
  }

  // Return A^{-1}
  std::vector<std::vector<double>> inverse() {
    int i,j,k;
    double sum;
    std::vector<std::vector<double>> ainv = std::vector<std::vector<double>>(n);
    

    for (i=0; i<n; ++i)
      for (j=0; j<n; ++j) {
        sum = (i == j ? 1. : 0. );
        for (k = i-1; k>= j; --k)
          sum -= el[i][k] * ainv[j][k];
        ainv[j][i] = sum/el[i][i];
      }
    for (i=n-1; i>=0; --i)
      for (j=0; j<=i; ++j) {
        sum = (i<j ? 0. : ainv[j][i]);
        for (k=i+1; k<n; ++k)
          sum -= el[k][i] * ainv[j][k];
        ainv[i][j] = ainv[j][i] = sum/el[i][i];
      }
    return ainv;
  }  

  //  return log(|A|)
  double logdet() {
    double sum=0;
    for (int i=0; i<n; ++i)
      sum += log(el[i][i]);
    return 2.*sum;
  }
};

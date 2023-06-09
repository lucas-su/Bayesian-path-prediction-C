import matrix;
import cholesky;

real lgamma(real z) {
     if (z>171)
          return (z-.5)*log(z) - z + .5*log(2*pi);
     else
          return log(gamma(z));
}

real lbeta(real x, real a, real b) {
     return lgamma(a+b) - lgamma(a) - lgamma(b) + (a-1) * log(x) + (b-1) * log(1-x);
}

real beta(real x, real a, real b) {
     // real v1 = (gamma(a+b)/gamma(a) * ((x ^ (a-1)) * ((1-x)^(b-1)))/ gamma(b)) ;
     // real v2 = exp(lbeta(x,a,b));
     // write("Beta");
     // write(v1);
     // write("log beta");
     // write(lbeta(x,a,b));
     // write("exp log beta");
     // write(v2);
     return exp(lbeta(x,a,b));
}

real gauss(real x, real mu, real s) {
  return exp(-(x-mu)^2/(2*s^2)) / (sqrt(2*pi)*s);
}

real dgauss(real x, real mu, real sigma) {
     return gauss(x,mu,sigma) * (-(x-mu)/sigma^2);
}

real gaussCDF(real x, real mu, real s) {
     return .5*(1+erf((x-mu)/sqrt(2*s^2)));
}

real det2(real[][] C) {
  return C[0][0]*C[1][1] - C[0][1]*C[1][0];
} 

real gauss2d(pair x, real[] mu, real [][] C) {
  real [] xv = {x.x, x.y}; 
  real [] diff = difference(xv,mu);
  return 1/(2*pi*sqrt(det2(C))) * exp(-.5*inprod(multiply(diff,inverse(C)),diff));
}

real gaussnd(real[] x, real[] mu, real[][] C) {
  real [] diff = difference(x,mu);
  Cholesky c;
  c.decompose(C);
  real k = x.length;
  
  return 1/((2*pi)^(k/2)*exp(.5*c.logdet())) * exp(-.5*inprod(multiply(diff,c.inverse()),diff));
}

real gaussnd(real[] x, real[] mu, Cholesky c) {
  real [] diff = difference(x,mu);
  
  return 1/((2*pi)^(c.n/2)*exp(.5*c.logdet())) * exp(-.5*inprod(multiply(diff,c.inverse()),diff));
}


real loggaussnd(real[] x, real[] mu, Cholesky c) {
  real [] diff = difference(x,mu);
  
  return -.5 * (c.n*log(2*pi) + c.logdet() + inprod(multiply(diff,c.inverse()),diff));
}


real expdist(real x, real lambda) {
     if (x>=0)
          return lambda*exp(-lambda*x);
     else
          return 0;
}

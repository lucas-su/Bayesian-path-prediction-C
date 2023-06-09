import cholesky;
import matrix;

int rand_x = 0, rand_y = 1, RAND_MAX = 4294967295;

void rand_init(int seedx, int seedy)
{
     rand_x = seedx;
     if (seedy == 0)
          rand_y = seedy;
     else
          rand_y = 1;
}

int myrandom()
{
     rand_x = AND((rand_x * 69069 + 1),RAND_MAX);

     rand_y = AND(XOR(rand_y,rand_y * 8192),RAND_MAX);
     rand_y = AND(XOR(rand_y,quotient(rand_y,131072)),RAND_MAX);
     rand_y = AND(XOR(rand_y,rand_y * 32),RAND_MAX);

     return XOR(rand_x,rand_y);
}

int rand_int(int n)
{
     int s;
     if (n<2)
          return 0;
     s = quotient(RAND_MAX,n);
     while (true) {
          int r = quotient(myrandom(),s);
          if (r < n) return r;
     }
     return n;                  // fuck you.
}


/* No precomputed value yet */
bool _norm_fast = false;
real _norm_next;
real _norm_max = RAND_MAX / 2.0;

real rand_normal()
{
     /* Precomputed value ready */
     if (_norm_fast)
     {
	  /* Forget this value */
	  _norm_fast = false;

	  /* Done */
	  return (_norm_next);
     }

     /* New value available */
     _norm_fast = true;

     while (true)
     {
	  /* Maximum distance */

//           uint32_t
//                u1 = myrandom(),
//                u2 = myrandom();
          
// 	  double u = ((double)myrandom() - max + 0.5) / max; //works.
// 	  double v = ((double)myrandom() - max + 0.5) / max;
//           real u = myrandom(), v = myrandom();
          
	  real u = (myrandom() - _norm_max + 0.5) / _norm_max; //works.
	  real v = (myrandom() - _norm_max + 0.5) / _norm_max;

// 	  double u = ((int32_t)myrandom() + 0.5) / max; original.
// 	  double v = ((int32_t)myrandom() + 0.5) / max;

	  /* Find the distance from the origin */
	  real x, w = u * u + v * v + realMin;

	  /* Reject locations outside the unit circle */
	  if (w >= 1) continue;

	  /* Get normalized value */
	  x = sqrt(-2 * log(w) / w);

	  /* Save the next value */
	  _norm_next = u * x;

	  /* Done */
	  return (v * x);
     }

     return 12345.789;
}

real rand_normal(real mean, real stdev)
{
  return stdev*rand_normal() + mean;
}


real rand_range(real min, real max)
{
  real scale = max-min, r = myrandom();
  r = (r*scale)/RAND_MAX;
  
  return r + min;
}
  
real [] sampleGaussND(real [] mu, real [][] C) {
  Cholesky c;
  if (c.decompose(C) != 0)
    write ("sampleGaussND: C is not positive semidefinite!");
  int N = mu.length;
  real [] z = new real [N];

  for (int i=0; i!=N; ++i)
    z[i] = rand_normal();

  return sum(mu, multiply(c.el,z));
}

real [] sampleGaussND(real [] mu, Cholesky c) {
  int N = mu.length;
  real [] z = new real [N];

  for (int i=0; i!=N; ++i)
    z[i] = rand_normal();

  return sum(mu, multiply(c.el,z));
}


#include <stdio.h>
#include <math.h>

static double vmax;
static double amax;
static double dt;

void tprof_init (double vm, double am, double dt_)
{
  vmax = vm;
  amax = am;
  dt = dt_;
}


double tprof_calc (double pos, double vel, double pdes)
{
  double dpmin, dp, vdes;

  dpmin = pow (vel, 2.0) / 2.0 / amax;
  dp = pdes - pos;
  
  if (vel >= 0.0) {
    if (dpmin < dp) {
      vdes = vel + amax * dt;
      if (vdes < vmax) {
	return vdes;
      }
      return vmax;
    }
    return vel - amax * dt;
  }
  
  if ( - dpmin < dp) {
    return vel + amax * dt;
  }
  vdes = vel - amax * dt;
  if (vdes < - vmax) {
    return - vmax;
  }
  return vdes;
}


int main (int argc, char ** argv)
{
  struct {
    double p0, v0, p1, vm, am, dt, tt;
  } test[] = {
    {  0.0,  0.0,  1.5,  1.0, 3.0, 0.001,  3.0 },
    {  0.0,  0.0,  1.5,  1.0, 3.0, 0.01,   3.0 },
    {  2.0,  0.0, -1.0,  0.5, 2.0, 0.001, 10.0 },
    {  0.0, 10.0,  1.0, 15.0, 5.0, 0.001, 10.0 },
    //
    {  0.0,  1.0,  1.5,  1.0, 3.0, 0.001,  3.0 },
    {  0.0,  1.0,  1.5,  1.0, 3.0, 0.01,   3.0 },
    {  0.0,  1.0, -1.5,  1.0, 3.0, 0.001,  3.0 },
    {  0.0,  1.0, -1.5,  1.0, 3.0, 0.01,   3.0 },
    {  0.0, -1.0, -1.5,  1.0, 3.0, 0.001,  3.0 },
    {  0.0, -1.0, -1.5,  1.0, 3.0, 0.01,   3.0 }
  };
  
  double pos, vel;
  int ii, jj;

  for (ii = 0; ii < (sizeof test) / (sizeof *test); ++ii) {
    tprof_init (test[ii].vm, test[ii].am, test[ii].dt);
    pos = test[ii].p0;
    vel = test[ii].v0;
    for (jj = 0; jj < test[ii].tt / test[ii].dt; ++jj) {
      printf ("%f  %f  %f  %f  %f  %f\n",
	      jj * test[ii].dt,
	      test[ii].p0, test[ii].v0, test[ii].p1,
	      pos, vel);
      vel = tprof_calc (pos, vel, test[ii].p1);
      pos += vel * dt;
    }
    printf ("%f  %f  %f  %f  %f  %f\n"
	    "\n\n",
	    jj * test[ii].dt,
	    test[ii].p0, test[ii].v0, test[ii].p1,
	    pos, vel);
  }
  
  for (ii = 0; ii < (sizeof test) / (sizeof *test); ++ii) {
    printf ("# idx: %d  vm: %f  am: %f  dt: %f\n",
	    ii, test[ii].vm, test[ii].am, test[ii].dt);
  }
  
  return 0;
}

#include <math.h>
#include <stdio.h>


struct polar_s {
  double rho, gamma, delta;
};

struct command_s {
  double v, w;
};


struct polar_s ptctrl_calc_polar (double rx, double ry, double rth,
				  double gx, double gy, double gth)
{
  struct polar_s pp;
  double dx, dy, eps;
  
  dx = gx - rx;
  dy = gy - ry;
  pp.rho = sqrt (dx*dx + dy*dy);
  
  eps = atan2 (dy, dx);
  pp.gamma = eps - rth;
  pp.delta = gth - pp.gamma - rth;
  
  return pp;
}


struct command_s ptctrl_calc_com (struct polar_s ee,
				  struct polar_s kk)
{
  struct command_s cc;
  cc.v = kk.rho * ee.rho;
  cc.w = kk.gamma * ee.gamma + kk.delta * ee.delta;
  return cc;
}


static void step (double * rx, double * ry, double * rth,
		  struct command_s com,
		  double dt)
{
  double dx, dy, dth, mag;
  
  dth = com.w * dt;
  if (fabs (dth) > 1.0e-6) {
    /* use circular movement */
    mag = com.v / com.w;
    dx = mag * sin (dth);
    dy = mag * (1.0 - cos (dth));
  }
  else {
    /* linear movement */
    mag = com.v * dt;
    dx = mag * cos (0.5 * dth);
    dy = mag * sin (0.5 * dth);
  }
  
  (*rx) += dx * cos (*rth) - dy * sin (*rth);
  (*ry) += dy * cos (*rth) + dx * sin (*rth);
  (*rth) += dth;
}


int main (int argc, char ** argv)
{
  struct {
    double x0, y0, th0, gx, gy, gth;
  } test[] = {
    {  0.0,  0.0,         0.0,  1.0,  1.0,         0.0 },
    {  0.0,  0.0,  M_PI / 2.0, -1.0,  1.0,  M_PI / 2.0 },
    { -1.0,  2.0,  M_PI / 4.0,  1.0, -2.0,         0.0 },
    { -1.0,  2.0, -M_PI / 4.0, -1.0, -2.0,       -M_PI },
    {  1.0, -1.0, 0.75 * M_PI,  1.0, -1.0, 2.75 * M_PI }
  };
  
  double rx, ry, rth;
  int ii, jj;
  struct polar_s ee, kk;
  struct command_s com;
  
  kk.rho = 3.0;
  kk.gamma = 8.0;
  kk.delta = -1.5;
  
  for (ii = 0; ii < (sizeof test) / (sizeof *test); ++ii) {
    rx = test[ii].x0;
    ry = test[ii].y0;
    rth = test[ii].th0;
    
    for (jj = 0; jj < 200; ++jj) {
      ee = ptctrl_calc_polar (rx, ry, rth,
			      test[ii].gx, test[ii].gy, test[ii].gth);
      com = ptctrl_calc_com (ee, kk);
      printf ("%f  %f  %f  %f  %f  %f  %f  %f\n",
	      rx, ry, rth,
	      ee.rho, ee.gamma, ee.delta,
	      com.v, com.w);
      step (&rx, &ry, &rth, com, 0.01);
    }
    
    printf ("\n\n");
  }
  
  return 0;
}

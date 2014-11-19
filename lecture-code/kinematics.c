#include <math.h>
#include <stdio.h>

static double rl;
static double rr;
static double bb;
static double px;
static double py;
static double th;


void kinematics_init (void)
{
  rl = 0.1;
  rr = 0.1;
  bb = 0.2;
  px = 0.0;
  py = 0.0;
  th = 0.0;
}


void kinematics_step (double wl, double wr, double dt)
{
  double trans = (wl * rl + wr * rr) / 2.0;
  double rot = (wr * rr - wl * rl) / bb;
  double dth = rot * dt;
  double dx, dy, mag;
  if (fabs (dth) > 1.0e-6) {
    /* use circular movement */
    mag = trans / rot;
    dx = mag * sin (dth);
    dy = mag * (1.0 - cos (dth));
  }
  else {
    /* linear movement */
    mag = trans * dt;
    dx = mag * cos (0.5 * dth);
    dy = mag * sin (0.5 * dth);
  }
  
  px += dx * cos (th) - dy * sin (th);
  py += dy * cos (th) + dx * sin (th);
  th += dth;
}


int main (int argc, char ** argv)
{
  int ii, jj, kk;
  kinematics_init ();
  
  /* CCW */
  for (kk = -1; kk <= 1; ++kk) {
    rl = 0.1 * (1.0 + kk * 0.01);
    rr = 0.1 * (1.0 - kk * 0.01);
    bb = 0.2;
    px = 0.0;
    py = 0.0;
    th = 0.0;
    for (jj = 0; jj < 4; ++jj) {
      for (ii = 0; ii < 100; ++ii) {
	kinematics_step (   M_PI / 20.0,   M_PI / 20.0, 0.1);
	printf ("%f  %f  %f\n", px, py, th);
      }
      for (ii = 0; ii < 100; ++ii) {
	kinematics_step ( - M_PI / 20.0,   M_PI / 20.0, 0.1);
	printf ("%f  %f  %f\n", px, py, th);
      }
    }
    printf("\n\n");
  }
  
  /* CW */
  for (kk = -1; kk <= 1; ++kk) {
    rl = 0.1 * (1.0 + kk * 0.01);
    rr = 0.1 * (1.0 - kk * 0.01);
    bb = 0.2;
    px = 0.0;
    py = 0.0;
    th = 0.0;
    for (jj = 0; jj < 4; ++jj) {
      for (ii = 0; ii < 100; ++ii) {
	kinematics_step (   M_PI / 20.0,   M_PI / 20.0, 0.1);
	printf ("%f  %f  %f\n", px, py, th);
      }
      for (ii = 0; ii < 100; ++ii) {
	kinematics_step (   M_PI / 20.0, - M_PI / 20.0, 0.1);
	printf ("%f  %f  %f\n", px, py, th);
      }
    }
    printf("\n\n");
  }
  return 0;
}

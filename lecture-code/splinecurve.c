#include <stdio.h>
#include <math.h>


void base (double tt, double * bb)
{
  bb[0] = (  -pow(tt,3) + 3*pow(tt,2) - 3*tt + 1) / 6.0;
  bb[1] = ( 3*pow(tt,3) - 6*pow(tt,2)        + 4) / 6.0;
  bb[2] = (-3*pow(tt,3) + 3*pow(tt,2) + 3*tt + 1) / 6.0;
  bb[3] = (   pow(tt,3)                         ) / 6.0;
}


int main (int argc, char ** argv)
{
  struct {
    double x, y;
  } point[] = {
    {0.0, 0.0},
    {1.0, 0.0},
    {2.0, 0.0},
    {2.0, 1.0},
    {1.0, 1.0},
    {0.0, 1.0},
    {0.0, 2.0},
    {1.0, 2.0},
    {1.0, 1.0},
    {0.0, 0.0},
    {-1.0, -1.0}
  };
  
  double tt;
  double bb[4];
  int ii, jj;
  double xx, yy;
  
  for (ii = 0; ii < (sizeof point) / (sizeof *point) - 3; ++ii) {
    for (tt = 0.0; tt < 1.0; tt += 0.01) {
      base (tt, bb);
      xx = 0.0;
      yy = 0.0;
      for (jj = 0; jj < 4; ++jj) {
	xx += point[ii+jj].x * bb[jj];
	yy += point[ii+jj].y * bb[jj];
      }
      printf ("%f  %f\n", xx, yy);
    }
  }
  
  printf ("\n\n");
  
  for (ii = 0; ii < (sizeof point) / (sizeof *point); ++ii) {
    printf ("%f  %f\n", point[ii].x, point[ii].y);
  }
  
  return 0;
}

#include <stdio.h>
#include <math.h>

int main (int argc, char ** argv)
{
  double tt;
  for (tt = 0.0; tt < 1.0; tt += 0.01)
    printf ("%f  %f  %f  %f  %f\n",
	    tt,
	    (  -pow(tt,3) + 3*pow(tt,2) - 3*tt + 1) / 6.0,
	    ( 3*pow(tt,3) - 6*pow(tt,2)        + 4) / 6.0,
	    (-3*pow(tt,3) + 3*pow(tt,2) + 3*tt + 1) / 6.0,
	    (   pow(tt,3)                         ) / 6.0);
  return 0;
}

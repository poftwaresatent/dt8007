#include "wheel.h"
#include <stdio.h>

int main (int argc, char ** argv)
{
  int seq[] = { 127, 270, -20, 270, 127 };
  //  int seq[] = { 32, 66, -5, 66, 32 };
  int ii;
  wheel_init (1);
  for (ii = 1; ii < (sizeof seq) / (sizeof *seq); ++ii) {
    int com = seq[ii-1];
    while ((   seq[ii-1] < seq[ii] && com <= seq[ii])
	   || (seq[ii-1] > seq[ii] && com >= seq[ii])) {
      wheel_set_speed (0, com);
      wheel_tick (1.0e-2);
      printf ("%d  %d\n", com, wheel_get_enc (0));
      if (seq[ii-1] < seq[ii]) {
	++com;
      }
      else {
	--com;
      }
    }
  }
  return 0;
}

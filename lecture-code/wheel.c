#include <stdlib.h>
#include <math.h>

struct channel_s {
  int n_enc;
  double r_gear;
  double w_max;
  int s_min, s_max;
  double w_cur;
  double p_cur;
};

static struct channel_s * channel;
static int nchan;

void wheel_init (int nchannels)
{
  int ii;
  channel = malloc (nchannels * (sizeof * channel));
  for (ii = 0 ; ii < nchannels; ++ii) {
    channel[ii].n_enc = 16;
    channel[ii].r_gear = 4.0;
    channel[ii].w_max = 3.0;
    channel[ii].s_min = 0;
    channel[ii].s_max = 254;
    channel[ii].w_cur = 0.0;
    channel[ii].p_cur = 0.0;
  }
  nchan = nchannels;
}


void wheel_fini (void)
{
  free (channel);
  channel = NULL;
  nchan = 0;
}


void wheel_set_speed (int channel_id, int com)
{
  struct channel_s * ch = channel + channel_id;
  ch->w_cur = (2.0 * ch->w_max / (ch->s_max - ch->s_min)) * (com - ch->s_min);
  ch->w_cur = rint (ch->w_cur * (ch->s_max - ch->s_min)) / (ch->s_max - ch->s_min);
  ch->w_cur -= ch->w_max;
  if (com <= ch->s_min) {
    ch->w_cur = - ch->w_max;
  }
  else if (com >= ch->s_max) {
    ch->w_cur = ch->w_max;
  }
}


/* XXX to do: should be unsigned and 16 (?) bits to be more realistic*/
int wheel_get_enc (int channel_id)
{
  struct channel_s * ch = channel + channel_id;
  return floor (ch->p_cur / 2.0 / M_PI * ch->n_enc * ch->r_gear);
}


void wheel_tick (double dt)
{
  int ii;
  for (ii = 0 ; ii < nchan; ++ii) {
    channel[ii].p_cur += dt * channel[ii].w_cur;
  }
}

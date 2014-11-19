#ifndef DEIS_WHEEL_H
#define DEIS_WHEEL_H

void wheel_init (int nchannels);
void wheel_fini (void);
void wheel_set_speed (int channel_id, int com);
int wheel_get_enc (int channel_id);
void wheel_tick (double dt);

#endif

/* Nepumuk2 Setup for Halmstad University DT8007 Design of Embedded Intelligent Systems
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#include "piesim.h"


int pie_init (void)
{
  return 0;
}


int pie_tick (void)
{
  struct piesim_sensors_s sensors;
  if (0 != piesim_read_sensors (&sensors)) {
    return -1;
  }
  
  printf ("%f  %f\n", sensors.left_distance, sensors.right_distance);
  
  struct piesim_command_s command;
  command.left_speed = 0.1;
  command.right_speed = 0.2;
  if (0 != piesim_write_command (&command)) {
    return -2;
  }
  
  return 0;
}

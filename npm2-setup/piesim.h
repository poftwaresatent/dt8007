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

#ifndef PIESIM_PIE_H
#define PIESIM_PIE_H

#ifdef __cplusplus
extern "C" {
#endif
  
  
  struct piesim_sensors_s {
    double left_distance, right_distance;
  };
  
  int piesim_read_sensors (struct piesim_sensors_s * sensors);
  
  
  struct piesim_command_s {
    double left_speed, right_speed;
  };
  
  int piesim_write_command (struct piesim_command_s const * command);
  
  
#ifdef __cplusplus
}
#endif

#endif

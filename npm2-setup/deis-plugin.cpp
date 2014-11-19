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

#include "student-code.h"
#include "piesim.h"

#include <npm2/Plugin.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Process.hpp>
#include <npm2/RayDistanceSensor.hpp>
#include <npm2/DifferentialDrive.hpp>

using namespace npm2;


class DEISProcess
  : public Process
{
public:
  explicit DEISProcess (string const & name);
  
  virtual state_t init (ostream & erros);
  virtual state_t run (double timestep, ostream & erros);
  
  RayDistanceSensor * left_sensor_;
  RayDistanceSensor * right_sensor_;
  DifferentialDrive * drive_;
};

static DEISProcess process ("deis");


int npm2_plugin_init ()
{
  Factory::instance().declareSingleton <DEISProcess> ("DEISProcess", &process);
  return 0;
}


DEISProcess::
DEISProcess (string const & name)
  : Process (name),
    left_sensor_ (0),
    right_sensor_ (0),
    drive_ (0)
{
  reflectSlot ("left_sensor", &left_sensor_);
  reflectSlot ("right_sensor", &right_sensor_);
  reflectSlot ("drive", &drive_);
}
  
  
DEISProcess::state_t DEISProcess::
init (ostream & erros)
{
  if (( ! drive_) || ( ! left_sensor_) || ( ! right_sensor_)) {
    erros << "DEISProcess needs a drive and two sensors (left and right)\n";
    return FAILED;
  }
  
  int const res (pie_init());
  if (0 != res) {
    erros << "DEISProcess: pie_init returned " << res << "\n";
    return FAILED;
  }
  
  return RUNNING;
}
  
  
DEISProcess::state_t DEISProcess::
run (double timestep, ostream & erros)
{
  int const res (pie_tick());
  if (0 != res) {
    erros << "DEISProcess: pie_tick returned " << res << "\n";
    return FAILED;
  }
  return RUNNING;
}


int piesim_read_sensors (struct piesim_sensors_s * sensors)
{
  sensors->left_distance = process.left_sensor_->distance_;
  sensors->right_distance = process.right_sensor_->distance_;
  return 0;
}


int piesim_write_command (struct piesim_command_s const * command)
{
  process.drive_->setSpeed (command->left_speed, command->right_speed);
  return 0;
}

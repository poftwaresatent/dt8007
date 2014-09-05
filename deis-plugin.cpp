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

#include <npm2/Plugin.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Process.hpp>

using namespace npm2;


class DEISProcess
  : public Process
{
public:
  explicit DEISProcess (string const & name);
  
protected:
  virtual state_t init (ostream & erros);
  virtual state_t run (double timestep, ostream & erros);
};


int npm2_plugin_init ()
{
  Factory::instance().declare <DEISProcess> ("DEISProcess");
  return 0;
}


DEISProcess::
DEISProcess (string const & name)
  : Process (name)
{
}
  
  
DEISProcess::state_t DEISProcess::
init (ostream & erros)
{
  erros << "init\n";
  return RUNNING;
}
  
  
DEISProcess::state_t DEISProcess::
run (double timestep, ostream & erros)
{
  erros << "run\n";
  return RUNNING;
}

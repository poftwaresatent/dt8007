/* Copyright (C) 2014 Roland Philippsen. All rights reserved.
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
#include <npm2/Simulator.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Process.hpp>
#include <npm2/KinematicControl.hpp>
#include <cmath>

using namespace npm2;
using namespace sfl;


class SplineFollowProcess
  : public Process
{
public:
  explicit SplineFollowProcess (string const & name);
  
  virtual state_t init (ostream & erros);
  virtual state_t run (double timestep, ostream & erros);
  
  KinematicControl * control_;
  Goal start_;
  Goal end_;
  double duration_;
  double tstart_;
  double d_orient_;
  
  struct Point {
    Point (): x (0.0), y (0.0) {}
    Point (double _x, double _y): x (_x), y (_y) {}
    double x, y;
  };
  
  Point points_[6];
};


int npm2_plugin_init ()
{
  Factory::instance().declare <SplineFollowProcess> ("SplineFollowProcess");
  return 0;
}


SplineFollowProcess::
SplineFollowProcess (string const & name)
  : Process (name),
    control_ (0),
    start_ (0.0, 0.0, 0.0, 0.1, 0.1),
    end_ (1.0, 1.0, 0.0, 0.1, 0.1),
    duration_ (2.0),
    d_orient_ (0.5)
{
  reflectSlot ("control", &control_);
  reflectParameter ("start", &start_);
  reflectParameter ("end", &end_);
  reflectParameter ("duration", &duration_);
  reflectParameter ("d_orient", &d_orient_);
}


SplineFollowProcess::state_t SplineFollowProcess::
init (ostream & erros)
{
  if ( ! control_) {
    erros << "SplineFollowProcess needs a control\n";
    return FAILED;
  }
  
  tstart_ = Simulator::clock();
  
  control_->enable (true);
  control_->setGoal (start_);
  
  points_[0].x = start_.X() - d_orient_ * cos (start_.Theta());
  points_[0].y = start_.Y() - d_orient_ * sin (start_.Theta());
  points_[1].x = start_.X();
  points_[1].y = start_.Y();
  points_[2].x = start_.X() + d_orient_ * cos (start_.Theta());
  points_[2].y = start_.Y() + d_orient_ * sin (start_.Theta());
  points_[3].x = end_.X() - d_orient_ * cos (end_.Theta());
  points_[3].y = end_.Y() - d_orient_ * sin (end_.Theta());
  points_[4].x = end_.X();
  points_[4].y = end_.Y();
  points_[5].x = end_.X() + d_orient_ * cos (end_.Theta());
  points_[5].y = end_.Y() + d_orient_ * sin (end_.Theta());
  
  return RUNNING;
}


static void foobar (double pax, double pay,
		    double pbx, double pby,
		    double pcx, double pcy,
		    double pdx, double pdy,
		    double tau,
		    double & gx, double & gy, double & gth)
{
  double const bm1  ((  -tau*tau*tau + 3*tau*tau -  3*tau + 1) / 6);
  double const bm1d ((                -3*tau*tau +  6*tau - 3) / 6);
  
  double const b0   (( 3*tau*tau*tau - 6*tau*tau          + 4) / 6);
  double const b0d  ((                 9*tau*tau - 12*tau    ) / 6);
  
  double const b1   ((-3*tau*tau*tau + 3*tau*tau +  3*tau + 1) / 6);
  double const b1d  ((                -9*tau*tau +  6*tau + 3) / 6);
  
  double const b2   (    tau*tau*tau/6);
  double const b2d  (        tau*tau/2);
  
  gx = pax*bm1 + pbx*b0 + pcx*b1 * pdx*b2;
  gy = pay*bm1 + pby*b0 + pcy*b1 * pdy*b2;
  gth = atan2 (pay*bm1d + pby*b0d + pcy*b1d * pdy*b2d,
	       pax*bm1d + pbx*b0d + pcx*b1d * pdx*b2d);
}

	       
SplineFollowProcess::state_t SplineFollowProcess::
run (double timestep, ostream & erros)
{
  double const lambda (3.0 * (Simulator::clock() - tstart_) / duration_);
  cout << "lambda " << lambda << "\n";
  
  if (lambda <= 0.0) {
    control_->setGoal (start_);
  }
  else if (lambda <= 1.0) {
    double gx, gy, gth;
    foobar (points_[0].x, points_[0].y,
	    points_[1].x, points_[1].y,
	    points_[2].x, points_[2].y,
	    points_[3].x, points_[3].y,
	    lambda,
	    gx, gy, gth);
    control_->setGoal (Goal (gx, gy, gth, end_.Dr(), end_.Dtheta()));
  }
  else if (lambda <= 2.0) {
    double gx, gy, gth;
    foobar (points_[1].x, points_[1].y,
	    points_[2].x, points_[2].y,
	    points_[3].x, points_[3].y,
	    points_[4].x, points_[4].y,
	    lambda - 1.0,
	    gx, gy, gth);
    control_->setGoal (Goal (gx, gy, gth, end_.Dr(), end_.Dtheta()));
  }
  else if (lambda <= 3.0) {
    double gx, gy, gth;
    foobar (points_[2].x, points_[2].y,
	    points_[3].x, points_[3].y,
	    points_[4].x, points_[4].y,
	    points_[5].x, points_[5].y,
	    lambda - 2.0,
	    gx, gy, gth);
    control_->setGoal (Goal (gx, gy, gth, end_.Dr(), end_.Dtheta()));
  }
  else {
    control_->setGoal (end_);
  }
  
  return RUNNING;
}

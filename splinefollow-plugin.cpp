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
#include <npm2/Drawing.hpp>
#include <npm2/gl.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Process.hpp>
#include <npm2/KinematicControl.hpp>
#include <sfl/api/Goal.hpp>

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
  Frame start_;
  Frame end_;
  Goal carrot_;
  double duration_;
  double tstart_;
  double dr_;
  double dth_;
  double d_orient_;
  
  struct Point {
    Point (): x (0.0), y (0.0) {}
    Point (double _x, double _y): x (_x), y (_y) {}
    double x, y;
  };
  
  Point points_[6];
};


class SplineFollowDrawing
  : public Drawing
{
public:
  explicit SplineFollowDrawing (const string & name);
  
  virtual void draw ();
  
  SplineFollowProcess * process_;
  double wheelbase_;
};


int npm2_plugin_init ()
{
  Factory::instance().declare <SplineFollowProcess> ("SplineFollowProcess");
  Factory::instance().declare <SplineFollowDrawing> ("SplineFollowDrawing");
  return 0;
}


SplineFollowProcess::
SplineFollowProcess (string const & name)
  : Process (name),
    control_ (0),
    start_ (0.0, 0.0, 0.0),
    end_ (1.0, 1.0, 0.0),
    carrot_ (0.0, 0.0, 0.0, 0.1, 0.1),
    duration_ (2.0),
    dr_ (0.1),
    dth_ (0.1),
    d_orient_ (0.5)
{
  reflectSlot ("control", &control_);
  reflectParameter ("start", &start_);
  reflectParameter ("end", &end_);
  reflectParameter ("duration", &duration_);
  reflectParameter ("dr", &dr_);
  reflectParameter ("dth", &dth_);
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
  carrot_.Set (start_.X(), start_.Y(), start_.Theta(), dr_, dth_);
  control_->setGoal (carrot_);
  
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


static void calc_p (double pax, double pay,
		    double pbx, double pby,
		    double pcx, double pcy,
		    double pdx, double pdy,
		    double tau,
		    double & qx, double & qy)
{
  double const bm1  ((  -tau*tau*tau + 3*tau*tau -  3*tau + 1) / 6);
  double const b0   (( 3*tau*tau*tau - 6*tau*tau          + 4) / 6);
  double const b1   ((-3*tau*tau*tau + 3*tau*tau +  3*tau + 1) / 6);
  double const b2   (    tau*tau*tau/6);
  
  qx = pax*bm1 + pbx*b0 + pcx*b1 * pdx*b2;
  qy = pay*bm1 + pby*b0 + pcy*b1 * pdy*b2;
}


static void calc_dp (double pax, double pay,
		     double pbx, double pby,
		     double pcx, double pcy,
		     double pdx, double pdy,
		     double tau,
		     double & dqx, double & dqy)
{
  double const bm1d ((-3*tau*tau +  6*tau - 3) / 6);
  double const b0d  (( 9*tau*tau - 12*tau    ) / 6);
  double const b1d  ((-9*tau*tau +  6*tau + 3) / 6);
  double const b2d  (    tau*tau/2);
  
  dqx = pax*bm1d + pbx*b0d + pcx*b1d * pdx*b2d;
  dqy = pay*bm1d + pby*b0d + pcy*b1d * pdy*b2d;
}


static void calc_carrot (double pax, double pay,
			 double pbx, double pby,
			 double pcx, double pcy,
			 double pdx, double pdy,
			 double tau,
			 Goal & carrot)
{
  double qx, qy, dqx, dqy;
  calc_p (pax, pay, pbx, pby, pcx, pcy, pdx, pdy, tau, qx, qy);
  calc_dp (pax, pay, pbx, pby, pcx, pcy, pdx, pdy, tau, dqx, dqy);
  
  carrot.Set (qx, qy, atan2 (dqy, dqx), carrot.Dr(), carrot.Dtheta());
}


SplineFollowProcess::state_t SplineFollowProcess::
run (double timestep, ostream & erros)
{
  double const lambda (3.0 * (Simulator::clock() - tstart_) / duration_);
  double tau;
  int offset;
  
  if (lambda <= 0.0) {
    tau = 0;
    offset = 0;
  }
  else if (lambda <= 1.0) {
    tau = lambda;
    offset = 0;
  }
  else if (lambda <= 2.0) {
    tau = lambda - 1.0;
    offset = 1;
  }
  else if (lambda <= 3.0) {
    tau = lambda - 2.0;
    offset = 2;
  }
  else {
    tau = 1.0;
    offset = 2;
  }
  
  calc_carrot (points_[offset + 0].x, points_[offset + 0].y,
	       points_[offset + 1].x, points_[offset + 1].y,
	       points_[offset + 2].x, points_[offset + 2].y,
	       points_[offset + 3].x, points_[offset + 3].y,
	       tau,
	       carrot_);
  control_->setGoal (carrot_);
  
  return RUNNING;
}


SplineFollowDrawing::
SplineFollowDrawing (const string & name)
  : Drawing (name, "curve and control point of a SplineFollowProcess"),
    process_ (0),
    wheelbase_ (0.2)
{
  reflectSlot ("process", &process_);
  reflectParameter ("wheelbase", &wheelbase_);
}


static void drawPose (double px, double py, double pth,
		      double width)
{
  glBegin (GL_LINES);
  
  double xx (width / 2);
  double yy (0);
  Frame const pose (px, py, pth);
  pose.To (xx, yy);
  glVertex2d (pose.X(), pose.Y());
  glVertex2d (xx, yy);
  
  xx = 0;
  yy = width / 2;
  pose.To (xx, yy);
  glVertex2d (pose.X(), pose.Y());
  glVertex2d (xx, yy);
  
  xx = 0;
  yy = - width / 2;
  pose.To (xx, yy);
  glVertex2d (pose.X(), pose.Y());
  glVertex2d (xx, yy);
  
  glEnd();
}


void SplineFollowDrawing::
draw ()
{
  if ( ! process_) {
    return;
  }
  
  glColor3d (0.5, 0.25, 0.25);
  glLineWidth (1);
  glBegin (GL_LINE_STRIP);
  for (int offset (0); offset < 3; ++offset) {
    for (double tau (0.0); tau < 1.0; tau += 0.1) {
      double qx, qy;
      calc_p (process_->points_[offset + 0].x, process_->points_[offset + 0].y,
	      process_->points_[offset + 1].x, process_->points_[offset + 1].y,
	      process_->points_[offset + 2].x, process_->points_[offset + 2].y,
	      process_->points_[offset + 3].x, process_->points_[offset + 3].y,
	      tau,
	      qx, qy);
      glVertex2d (qx, qy);
    }
  }
  glEnd();

  glColor3d (1, 0.5, 0.5);
  glPointSize (4);
  glBegin (GL_POINTS);
  for (int ii (0); ii < 6; ++ii) {
    glVertex2d (process_->points_[ii].x, process_->points_[ii].y);
  }
  glEnd();
  
  glColor3d (1, 0.5, 0.5);
  glLineWidth (1);
  glPointSize (1);
  drawPose (process_->carrot_.X(), process_->carrot_.Y(), process_->carrot_.Theta(), wheelbase_);
}

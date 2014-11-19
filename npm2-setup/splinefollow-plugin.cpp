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
  
  double px_[6], py_[6];
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
  
  px_[0] = start_.X() - d_orient_ * cos (start_.Theta());
  py_[0] = start_.Y() - d_orient_ * sin (start_.Theta());
  px_[1] = start_.X();
  py_[1] = start_.Y();
  px_[2] = start_.X() + d_orient_ * cos (start_.Theta());
  py_[2] = start_.Y() + d_orient_ * sin (start_.Theta());
  px_[3] = end_.X() - d_orient_ * cos (end_.Theta());
  py_[3] = end_.Y() - d_orient_ * sin (end_.Theta());
  px_[4] = end_.X();
  py_[4] = end_.Y();
  px_[5] = end_.X() + d_orient_ * cos (end_.Theta());
  py_[5] = end_.Y() + d_orient_ * sin (end_.Theta());
  
  return RUNNING;
}


static void base (double tt, double * bb)
{
  bb[0] = (  -pow(tt,3) + 3*pow(tt,2) - 3*tt + 1) / 6.0;
  bb[1] = ( 3*pow(tt,3) - 6*pow(tt,2)        + 4) / 6.0;
  bb[2] = (-3*pow(tt,3) + 3*pow(tt,2) + 3*tt + 1) / 6.0;
  bb[3] = (   pow(tt,3)                         ) / 6.0;
}


static void calc_p (double * px, double * py,
		    double tau,
		    double & qx, double & qy)
{
  double bb[4];
  base (tau, bb);
  qx = 0.0;
  qy = 0.0;
  for (size_t ii (0); ii < 4; ++ii) {
    qx += px[ii] * bb[ii];
    qy += py[ii] * bb[ii];
  }
}


static void dbase (double tt, double * db)
{
  db[0] = (-3*pow(tt,2) +  6*tt - 3) / 6.0;
  db[1] = ( 9*pow(tt,2) - 12*tt    ) / 6.0;
  db[2] = (-9*pow(tt,2) +  6*tt + 3) / 6.0;
  db[3] = ( 3*pow(tt,2)            ) / 6.0;
}


static void calc_dp (double * px, double * py,
		     double tau,
		     double & dqx, double & dqy)
{
  double db[4];
  dbase (tau, db);
  dqx = 0.0;
  dqy = 0.0;
  for (size_t ii (0); ii < 4; ++ii) {
    dqx += px[ii] * db[ii];
    dqy += py[ii] * db[ii];
  }
}


static void calc_carrot (double * px, double * py,
			 double tau,
			 Goal & carrot)
{
  double qx, qy, dqx, dqy;
  calc_p  (px, py, tau, qx, qy);
  calc_dp (px, py, tau, dqx, dqy);
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
  
  calc_carrot (px_ + offset, py_ + offset, tau, carrot_);
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
  
  glColor3d (1.0, 0.8, 0.8);
  glLineWidth (1);
  glBegin (GL_LINE_STRIP);
  int offset (0);
  for (int ii (0); ii <= 10; ++ii) {
    double qx, qy;
    calc_p (process_->px_ + offset, process_->py_ + offset, 0.1 * ii, qx, qy);
    glVertex2d (qx, qy);
  }
  glEnd();
  
  glColor3d (0.8, 1.0, 0.8);
  glLineWidth (1);
  glBegin (GL_LINE_STRIP);
  offset = 1;
  for (int ii (0); ii <= 10; ++ii) {
    double qx, qy;
    calc_p (process_->px_ + offset, process_->py_ + offset, 0.1 * ii, qx, qy);
    glVertex2d (qx, qy);
  }
  glEnd();
  
  glColor3d (0.8, 0.8, 1.0);
  glLineWidth (1);
  glBegin (GL_LINE_STRIP);
  offset = 2;
  for (int ii (0); ii <= 10; ++ii) {
    double qx, qy;
    calc_p (process_->px_ + offset, process_->py_ + offset, 0.1 * ii, qx, qy);
    glVertex2d (qx, qy);
  }
  glEnd();
  
  glColor3d (1.0, 0.5, 0.5);
  glPointSize (4);
  glBegin (GL_POINTS);
  for (int ii (0); ii < 3; ++ii) {
    glVertex2d (process_->px_[ii], process_->py_[ii]);
  }
  glEnd();
  
  glColor3d (0.5, 1.0, 0.5);
  glPointSize (4);
  glBegin (GL_POINTS);
  for (int ii (3); ii < 6; ++ii) {
    glVertex2d (process_->px_[ii], process_->py_[ii]);
  }
  glEnd();
  
  glColor3d (1, 0.5, .5);
  glLineWidth (1);
  glPointSize (1);
  drawPose (process_->carrot_.X(), process_->carrot_.Y(), process_->carrot_.Theta(), wheelbase_);
}

/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openflipper.org                            *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenFlipper.                                         *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
 *                                                                           *
\*===========================================================================*/



//=============================================================================
//
//  CLASS GLTrackball - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================


#include "GLTrackball.hh"


//== NAMESPACES ===============================================================


namespace ACG {


//== IMPLEMENTATION ========================================================== 


GLTrackball::
GLTrackball(GLState& _state) 
  : glstate_(_state), 
    center_(0,0,0) ,
    last_point_ok_(false),
    action_(ROTATION)
{
  for (int i=0; i<10; ++i)
    button_down_[i] = false;
}


//-----------------------------------------------------------------------------


void 
GLTrackball::mouse_press(int button, int x, int y)
{
  last_point_2D_ = ACG::Vec2i(x, y);
  last_point_ok_ = map_to_sphere(last_point_2D_, last_point_3D_);

  button_down_[button] = true;
}


//-----------------------------------------------------------------------------


void 
GLTrackball::mouse_release(int button, int /* x */ , int /* y */ )
{
  last_point_ok_ = false;
  button_down_[button] = false;

  // GLUT: button 3 or 4 -> mouse wheel clicked
  if (button == 3)       
    zoom(0, (int)(last_point_2D_[1] - 0.05*glstate_.viewport_width()));
  else if (button == 4)
    zoom(0, (int)(last_point_2D_[1] + 0.05*glstate_.viewport_width()));
}


//-----------------------------------------------------------------------------


void 
GLTrackball::mouse_move(int x, int y)
{
  if (button_down_[0] && button_down_[1])
    action_ = ZOOM;
  else if (button_down_[0])
    action_ = ROTATION;
  else if (button_down_[1])
    action_ = TRANSLATION;

  switch (action_)
  {
    case ROTATION:    rotation(x, y);     break;
    case TRANSLATION: translation(x, y);  break;
    case ZOOM:        zoom(x, y);         break;
  }

  last_point_2D_ = ACG::Vec2i(x, y);
  last_point_ok_ = map_to_sphere(last_point_2D_, last_point_3D_);
}


//-----------------------------------------------------------------------------


void 
GLTrackball::rotation(int x, int y)
{
  if (last_point_ok_) 
  {
    Vec2i  new_point_2D_;
    Vec3f  new_point_3D_;
    bool   new_point_ok_;

    new_point_2D_ = ACG::Vec2i(x, y);
    new_point_ok_ = map_to_sphere(new_point_2D_, new_point_3D_);
    
    if (new_point_ok_)
    {
      Vec3f axis      = (last_point_3D_ % new_point_3D_);
      float cos_angle = (last_point_3D_ | new_point_3D_);

      if (fabs(cos_angle) < 1.0) 
      {
	    double angle = 2.0*acos(cos_angle) * 180.0 / M_PI;

	    Vec3d t = glstate_.modelview().transform_point(center_);
	    glstate_.translate(-t[0], -t[1], -t[2], MULT_FROM_LEFT);
    	glstate_.rotate(angle, axis[0], axis[1], axis[2], MULT_FROM_LEFT);
	    glstate_.translate( t[0],  t[1],  t[2], MULT_FROM_LEFT);
      }
    }
  }
}


//-----------------------------------------------------------------------------


void 
GLTrackball::translation(int x, int y)
{
  double dx     = double(x - last_point_2D_[0]);
  double dy     = double(y - last_point_2D_[1]);

  double z      = glstate_.modelview().transform_point(center_)[2];
  double w      = double(glstate_.viewport_width());
  double h      = double(glstate_.viewport_height());
  double fovy   = double(glstate_.fovy());
  double nearpl = double(glstate_.near_plane());

  double aspect = w / h;
  double top    = double(tan(fovy/2.0*M_PI/180.0) * nearpl);
  double right  = aspect*top;

  glstate_.translate(-2.0*dx/w*right/nearpl*z, 
		              2.0*dy/h*top/nearpl*z, 
		              0.0,
		              MULT_FROM_LEFT);
}


//-----------------------------------------------------------------------------


void 
GLTrackball::zoom(int /* x */ , int y)
{
  double dy = double(y - last_point_2D_[1]);
  double z  = glstate_.modelview().transform_point(center_)[2];
  double h  = double(glstate_.viewport_height());

  glstate_.translate(0.0,
		     0.0,
		     -2.0*dy/h*z, 
		     MULT_FROM_LEFT);
}


//-----------------------------------------------------------------------------


bool
GLTrackball::map_to_sphere(const Vec2i& _point, Vec3f& _result)
{
  float width  = float(glstate_.viewport_width());
  float height = float(glstate_.viewport_height());
  
  if ( (_point[0] >= 0) && (_point[0] <= width) &&
       (_point[1] >= 0) && (_point[1] <= height) ) 
  {
    double x            = (_point[0] - 0.5*width)  / width;
    double y            = (0.5*height - _point[1]) / height;
    double sinx         = sin(M_PI * x * 0.5);
    double siny         = sin(M_PI * y * 0.5);
    double sinx2siny2   = sinx * sinx + siny * siny;
    
    _result[0] = float(sinx);
    _result[1] = float(siny);
    _result[2] = sinx2siny2 < 1.0 ? float( sqrt(1.0 - sinx2siny2) ) : 0.0f;
    
    return true;
  }
  else return false;
}


//=============================================================================
} // namespace ACG
//=============================================================================

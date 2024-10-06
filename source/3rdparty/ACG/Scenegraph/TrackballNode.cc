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
//  CLASS TrackballNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "TrackballNode.hh"
#include <ACG/GL/GLPrimitives.hh>


//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ========================================================== 


void
TrackballNode::draw(GLState& /* _state */ , const DrawModes::DrawMode& /* _drawMode */ )
{
  // draw the track ball
  if (drawTrackball_) 
  {
    ACG::GLState::disable(GL_LIGHTING);
    ACG::GLState::shadeModel( GL_FLAT );
    glPushMatrix();
    glTranslatef(center()[0], center()[1], center()[2]);
    glScalef(radius_, radius_, radius_);

    GLint previous[2];
    glGetIntegerv(GL_POLYGON_MODE,previous);
    glPolygonMode(GL_FRONT_AND_BACK,  GL_LINE);
    ACG::GLSphere sphere(20, 20);
    sphere.draw_primitive();
    glPolygonMode(GL_FRONT,previous[0]);
    glPolygonMode(GL_BACK,previous[1]);
    glPopMatrix();
  }


  // draw the coord axes
  if (drawAxes_)
  {
    // store original settings
    GLfloat backupColor[4], backupLineWidth;
    glGetFloatv(GL_CURRENT_COLOR, backupColor);
    glGetFloatv(GL_LINE_WIDTH, &backupLineWidth);
    
    ACG::GLState::disable(GL_LIGHTING);
    ACG::GLState::shadeModel( GL_FLAT );
    glLineWidth(3.0);

    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex(center());
    glVertex(center() + radius_*xAxis_);
    glEnd();

    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
    glVertex(center());
    glVertex(center() + radius_*yAxis_);
    glEnd();

    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
    glVertex(center());
    glVertex(center() + radius_*zAxis_);
    glEnd();

    // restore original settings
    glColor4fv(backupColor);
    glLineWidth(backupLineWidth);
  }
}


//----------------------------------------------------------------------------


void
TrackballNode::mouseEvent(GLState& _state, QMouseEvent* _event)
{
  Vec3d         oldPoint3D;
  Vec2i         newPoint2D(_event->pos().x(), _event->pos().y());
  Vec3d         newPoint3D;


  switch (_event->type())
  {
    case QEvent::MouseButtonPress:
    {
      break;
    }


    case QEvent::MouseButtonDblClick:
    {
      if (mapToSphere(_state, oldPoint2D_, oldPoint3D))
      {
	// toggle drawTrackball_
        if (_event->button() == Qt::LeftButton)
  	  drawTrackball_ = !drawTrackball_;

        // toggle drawAxes_
        if (_event->button() == Qt::MiddleButton)
	  drawAxes_ = !drawAxes_;
      }
      break;
    }
    
    
    case QEvent::MouseMove:
    {
      bool hit0 = mapToSphere(_state, newPoint2D, newPoint3D);
      bool hit1 = mapToSphere(_state, oldPoint2D_, oldPoint3D);

      if (hit0 && hit1)
      {

	// scaling
	if ((_event->button() & Qt::LeftButton) &&
	    (_event->button() & Qt::MiddleButton))
	{
	  double s = 1.0 + ((double) (newPoint2D[1] - oldPoint2D_[1]) /
			   (double) _state.viewport_height());
	  scale(s);
	}


	// translation
	else if (_event->button() & Qt::MiddleButton)
	{
	  double value_x = (radius_ * ((newPoint2D[0] - oldPoint2D_[0])) 
			   * 2.0 / (double) _state.viewport_width());
	  
	  double value_y = (radius_ * ((newPoint2D[1] - oldPoint2D_[1]))
			   * 2.0 / (double) _state.viewport_height());
	  
	  
	  /* need inverse transposed matrix in order to
	     transform direction vectors */
	  GLMatrixd m = inverse_matrix();
	  m.transpose();


	  Vec3d dx, dy;

	  // axis aligned
	  if (drawAxes_)
	  {
 	    dx = m.transform_point(xAxis_);
 	    dy = m.transform_point(yAxis_);
	  }

	  // screen space translation
	  else 
	  {
 	    dx = m.transform_vector(_state.right());
 	    dy = m.transform_vector(_state.up());
	  }
	  
	  dx.normalize();
	  dy.normalize();
	  translate(value_x*dx - value_y*dy);
	}
      


	// rotation
	else if (_event->button() & Qt::LeftButton) 
	{
	  Vec3d axis = oldPoint3D % newPoint3D;	
	  double cos_angle = ( oldPoint3D | newPoint3D );

	  
	  if (fabs(cos_angle) < 1.0)
	  {
	    // rotate coord axes
	    if (_event->modifiers() & Qt::AltModifier)
	    {
	      GLMatrixd mat;
	      
	      mat.identity();
	      mat.rotate(acos(cos_angle)*180.0/M_PI, axis);
	      
	      /* transform_point works since only 
		 rotations and no translations are
		 involved */
	      xAxis_ = mat.transform_point(xAxis_);
	      yAxis_ = mat.transform_point(yAxis_);
	      zAxis_ = mat.transform_point(zAxis_);
	    }
	    
	    // normal rotation
	    else  rotate(acos(cos_angle)*180.0/M_PI, axis);
	  }
	}
      }


      break;
    }

    default: // avoid warning
      break;
  }


  // store 2D point
  oldPoint2D_ = newPoint2D;
}


//----------------------------------------------------------------------------


bool
TrackballNode::mapToSphere( const GLState& _state,
			    const Vec2i& _v2, 
			    Vec3d& _v3 )
{
  // Qt -> GL coordinate systems
  unsigned int x = _v2[0];
  unsigned int y = _state.context_height() - _v2[1];


  // get ray from eye through pixel (trackball coords)
  Vec3d origin, direction;
  _state.viewing_ray(x, y, origin, direction);


  // translate and scale trackball to unit sphere
  origin -= center();
  origin /= radius_;


  // calc sphere-ray intersection
  // (sphere is centered at origin, has radius 1)
  double a = direction.sqrnorm(),
        b = 2.0 * (origin | direction),
        c = origin.sqrnorm() - 1.0,
        d = b*b - 4.0*a*c,
        t;

  if      (d <  0.0)  return false;
  else if (d == 0.0)  t = -b / (2.0*a);
  else 
  { 
    // t1 = (-b - sqrt(d)) / (2.0*a),  t2 = (-b + sqrt(d)) / (2.0*a)
    a = 1.0 / (2.0*a);
    d = sqrt(d);
    double t1 = (-b - d) * a;
    double t2 = (-b + d) * a;
    t = (t1 < t2) ? t1 : t2;
  }

  _v3 = origin + direction*t;
  
  return true;
}


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

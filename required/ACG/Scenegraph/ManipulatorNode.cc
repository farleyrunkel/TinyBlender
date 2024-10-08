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
//  CLASS ManipulatorNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "ManipulatorNode.hh"

//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================


const Vec4f cylinder_color (0.8f, 0.4f, 0.4f, 1.0f);
const Vec4f sphere_color   (0.8f, 0.4f, 0.4f, 1.0f);
const Vec4f select_color   (1.0f, 0.1f, 0.1f, 1.0f);
const float SCALE_CONST  =  5.0f;


//----------------------------------------------------------------------------


ManipulatorNode::
ManipulatorNode( BaseNode* _parent, const std::string& _name )
  : TransformNode(_parent, _name),
    draw_cylinder_(false),
    direction_(1.0,0.0,0.0),
    cylinder_(0),
    cylinder_radius_(1),
    cylinder_height_(10),
    cylinder_slices_(30),
    cylinder_stacks_(10),
    cylinder_clicked_(false),
    sphere_clicked_(false),
    touched_(false)
{
}


//----------------------------------------------------------------------------


ManipulatorNode::
~ManipulatorNode()
{
  if (cylinder_)
    delete cylinder_;
}


//----------------------------------------------------------------------------


void
ManipulatorNode::
setIdentity()
{
  direction_ = rotation().transform_vector( direction_ );
  TransformNode::setIdentity();
}


//----------------------------------------------------------------------------


void
ManipulatorNode::
setup_cylinder_system(GLState& _state)
{
  _state.translate(center()[0], center()[1], center()[2]);
  _state.mult_matrix(inverse_scale (), scale ()); // Adapt scaling

  // rotation axis & angle
  Vec3d z(0.0, 0.0, 1.0);
  double scal_prod = (direction_ | z);
  Vec3d  axis      = z % direction_;
  double norm      = axis.norm();
  double angle;

  if (norm > NumLimitsT<float>::min())
  {
    axis /= norm;
    if (scal_prod >  1.0) scal_prod =  1.0;
    if (scal_prod < -1.0) scal_prod = -1.0;
    angle = 180.0 / M_PI * acos(scal_prod);
  }
  else
  {
    axis  = Vec3d(1.0, 0.0, 0.0);
    angle = (scal_prod > 0.0) ? 0.0 : 180.0;
  }

  _state.rotate(angle, axis[0], axis[1], axis[2]);
}


void
ManipulatorNode::
setup_sphere_system(GLState& _state)
{
  setup_cylinder_system(_state);

  _state.translate(0, 0, cylinder_height_+ 2*cylinder_radius_);
}


//----------------------------------------------------------------------------


void
ManipulatorNode::draw(GLState& _state, const DrawModes::DrawMode& /* _drawMode */ )
{
  if (draw_cylinder_)
  {
      if(_state.compatibilityProfile())
      {
        ACG::GLState::enable(GL_LIGHTING);
        ACG::GLState::shadeModel(GL_FLAT);
      }


    // backup colors
    Vec4f backup_diffuse  = _state.diffuse_color();
    Vec4f backup_specular = _state.specular_color();


    // draw cylinder

    if (!cylinder_)
    {
      cylinder_ = new GLCylinder(cylinder_slices_, cylinder_stacks_, cylinder_radius_, false, false);
    }

    _state.push_modelview_matrix();
    setup_cylinder_system(_state);

    if( cylinder_clicked_)
    {
      _state.set_diffuse_color(select_color * 0.6f);
      _state.set_specular_color(select_color * 0.8f);
    }
    else
    {
      _state.set_diffuse_color(cylinder_color * 0.6f);
      _state.set_specular_color(cylinder_color * 0.8f);
    }

	// Zylinder in die X-Achse
	/*glPushMatrix();
	glLoadIdentity();
	glRotatef(-90, 0.0, 1.0, 0.0);*/

    if(_state.compatibilityProfile())
        ACG::GLState::shadeModel(GL_SMOOTH);
    cylinder_->setBottomRadius(cylinder_radius_);
    cylinder_->setTopRadius(cylinder_radius_);
    cylinder_->draw(_state, cylinder_height_);

	//glPopMatrix();


    // BIG wireframe sphere
    if( sphere_clicked_ )
    {
      _state.set_diffuse_color(select_color * 0.6f);
      _state.set_specular_color(select_color * 0.0f);
      if(_state.compatibilityProfile())
      {
        ACG::GLState::shadeModel(GL_SMOOTH);
        GLint previous[2];
        glGetIntegerv(GL_POLYGON_MODE,previous);
        glPolygonMode(GL_FRONT_AND_BACK,  GL_LINE);
        _state.scale(cylinder_height_+4*cylinder_radius_,cylinder_height_+4*cylinder_radius_,cylinder_height_+4*cylinder_radius_);
        ACG::GLSphere sphere(20, 20);
        sphere.draw_primitive();
        glPolygonMode(GL_FRONT,previous[0]);
        glPolygonMode(GL_BACK,previous[1]);
      }
    }

    _state.pop_modelview_matrix();




    // draw sphere

    _state.push_modelview_matrix();
    setup_sphere_system(_state);

    if( sphere_clicked_)
    {
      _state.set_diffuse_color(select_color * 0.6f);
      _state.set_specular_color(select_color * 0.8f);
    }
    else
    {
      _state.set_diffuse_color(sphere_color * 0.6f);
      _state.set_specular_color(sphere_color * 0.8f);
    }

    if(_state.compatibilityProfile())
    {
        ACG::GLState::shadeModel(GL_SMOOTH);
        _state.scale(2*cylinder_radius_,2*cylinder_radius_,2*cylinder_radius_);
        ACG::GLSphere sphere(20, 20);
        sphere.draw_primitive();
    }

    _state.pop_modelview_matrix();




    // restore
    _state.set_diffuse_color(backup_diffuse);
    _state.set_specular_color(backup_specular);
  }
}


//----------------------------------------------------------------------------


void
ManipulatorNode::mouseEvent(GLState& _state, QMouseEvent* _event)
{
  Vec3d         oldPoint3D;
  Vec2i         newPoint2D(_event->pos().x(), _event->pos().y());
  Vec3d         newPoint3D;
  double        new_axis_hit;


  switch (_event->type())
  {
    case QEvent::MouseButtonPress:
    {
      // hit sphere ?
      sphere_clicked_ = hitSphere(_state, newPoint2D);

      // hit cylinder ?
      cylinder_clicked_ = mapToCylinder(_state, newPoint2D, new_axis_hit);

      // If the user clicked on the manipulator, remember it
      if ( sphere_clicked_ || cylinder_clicked_)
        touched_ = true;

      // select only sphere or cylinder
      if(sphere_clicked_ && cylinder_clicked_)
        cylinder_clicked_ = false;

      oldPoint2D_   = newPoint2D;
      break;
    }


    case QEvent::MouseButtonRelease:
    {
      sphere_clicked_   = false;
      cylinder_clicked_ = false;
      break;
    }


    case QEvent::MouseButtonDblClick:
    {
      draw_cylinder_ = !draw_cylinder_;
      break;
    }


    case QEvent::MouseMove:
    {

      // IF sphere clicked rotate or change direction
      if(sphere_clicked_)
      {    
        bool hit0 = mapToSphere(_state, newPoint2D,  newPoint3D);
        bool hit1 = mapToSphere(_state, oldPoint2D_, oldPoint3D);

        if (  (newPoint2D != oldPoint2D_) && hit0 && hit1)
        {
          // change direction
          if(_event->modifiers() & Qt::ShiftModifier)
          {

            // calculate new cylinder direction
            direction_.normalize();
            newPoint3D.normalize();
            oldPoint3D.normalize();

            direction_ += newPoint3D - oldPoint3D;

          }
          // rotate
          else
          {

            Vec3d axis = oldPoint3D % newPoint3D;

            double cos_angle = ( oldPoint3D | newPoint3D );           

            if (fabs(cos_angle) < 1.0)
              rotate(acos(cos_angle)*180.0/M_PI, axis);
          }
        }
        else sphere_clicked_ = false;
      }


      // cylinder clicked  change scaling or change translation
      if(cylinder_clicked_)
      {
        double old_axis_hit;

        mapToCylinder(_state, oldPoint2D_, old_axis_hit);
        mapToCylinder(_state, newPoint2D,  new_axis_hit);

        // scale
        if(_event->modifiers() & Qt::ShiftModifier)
        {
          scale(1.0 + (new_axis_hit - old_axis_hit) /
              (cylinder_height_ * SCALE_CONST));
        }

        // twist
        else if(_event->modifiers() & (Qt::ControlModifier |
            Qt::AltModifier))
        {
          rotate( 45.0 * (new_axis_hit-old_axis_hit) / cylinder_height_,
              direction_);

        }

        // translate
        else
        {
          translate(new_axis_hit - old_axis_hit);
        }
      }

      break;
    }

    default: // avoid warning
      break;
  }


  // save old Point
  oldPoint2D_   = newPoint2D;
}


//----------------------------------------------------------------------------


bool ManipulatorNode::hitSphere( GLState& _state,
                                 const Vec2i& _v2)
{
  // Qt -> GL coordinate systems
  unsigned int x = _v2[0];
  unsigned int y = _state.context_height() - _v2[1];


  // get ray from eye through pixel, in sphere coords
  Vec3d origin, direction;

  _state.set_updateGL(false);
  _state.push_modelview_matrix();

  setup_sphere_system(_state);
  _state.scale(2*cylinder_radius_);

  _state.viewing_ray(x, y, origin, direction);

  _state.pop_modelview_matrix();
  _state.set_updateGL(true);



  // calc sphere-ray intersection
  // (sphere is centered at origin, has radius 1)
  double a = direction.sqrnorm(),
        b = 2.0 * (origin | direction),
        c = origin.sqrnorm() - 1.0,
        d = b*b - 4.0*a*c;

  return (d >= 0.0);
}


//----------------------------------------------------------------------------


bool
ManipulatorNode::mapToSphere( GLState& _state,
			      const Vec2i& _v2,
			      Vec3d& _v3 )
{
  // Qt -> GL coordinate systems
  unsigned int x = _v2[0];
  unsigned int y = _state.context_height() - _v2[1];



  // get ray from eye through pixel (trackball coords)
  // *no* rotation, points have to be in world coords
  Vec3d origin, direction;

  _state.set_updateGL(false);
  _state.push_modelview_matrix();

  _state.translate(center()[0], center()[1], center()[2]);
  _state.mult_matrix(inverse_scale (), scale ());
  _state.scale(cylinder_height_ + 4*cylinder_radius_);

  _state.viewing_ray(x, y, origin, direction);

  _state.pop_modelview_matrix();
  _state.set_updateGL(true);



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
    a = 1.0 / (2.0*a);
    d = sqrt(d);
    double t1 = (-b - d) * a;
    double t2 = (-b + d) * a;
    t = (t1 < t2) ? t1 : t2;
  }



  // map back to world coords
  _v3 = origin + direction*t;

  return true;
}


//----------------------------------------------------------------------------


bool
ManipulatorNode::mapToCylinder(GLState&       _state,
			       const Vec2i&   _v1,
			       double&         _axis_hit)
{
  // Qt -> GL coordinate systems
  unsigned int x = _v1[0];
  unsigned int y = _state.context_height() - _v1[1];


  // get ray from eye through pixel (cylinder coords)
  Vec3d origin, direction;

  _state.set_updateGL(false);
  _state.push_modelview_matrix();

  setup_cylinder_system(_state);

  _state.viewing_ray(x, y, origin, direction);

  _state.pop_modelview_matrix();
  _state.set_updateGL(true);



  // get cylinder axis ray: it's in its own coord system!
  const Vec3d origin2(0,0,0), direction2(0,0,1);


  // compute pseude-intersection
  Vec3d normal = (direction % direction2).normalize();
  Vec3d vd = ((origin2 - origin) % direction);
  _axis_hit = (normal | vd);

  double orthodistance = std::abs( ( origin2 - origin ) | normal);


  // hit cylinder ?
  return((orthodistance < cylinder_radius_) &&
         (_axis_hit >= 0)                   &&
         (_axis_hit <= cylinder_height_));
}


//----------------------------------------------------------------------------


void ManipulatorNode::pick(GLState& _state, PickTarget _target) {
  if (_target == PICK_FACE || _target == PICK_ANYTHING) {
    if (draw_cylinder_) {

      _state.pick_set_maximum(2);

      // cylinder
      _state.push_modelview_matrix();
      setup_cylinder_system(_state);
      _state.pick_set_name(0);
      cylinder_->setBottomRadius(cylinder_radius_);
      cylinder_->setTopRadius(cylinder_radius_);
      cylinder_->draw(_state, cylinder_height_);
      _state.pop_modelview_matrix();

      // sphere
      _state.push_modelview_matrix();
      setup_sphere_system(_state);
      _state.pick_set_name(1);
      if(_state.compatibilityProfile()) {
        _state.scale(2*cylinder_radius_,2*cylinder_radius_,2*cylinder_radius_);
        ACG::GLSphere sphere(20, 20);
        sphere.draw_primitive();
      }
      _state.pop_modelview_matrix();
    }
  }
}


//----------------------------------------------------------------------------


void
ManipulatorNode::set_direction(Vec3d& _v)
{

    direction_ = inverse_rotation().transform_vector(_v.normalize());
}

//----------------------------------------------------------------------------

Vec3d
ManipulatorNode::direction() const
{

    return rotation().transform_vector(direction_);
}



//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

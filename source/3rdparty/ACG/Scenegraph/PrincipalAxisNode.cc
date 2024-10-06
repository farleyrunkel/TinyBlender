/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2016, RWTH-Aachen University                 *
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
//  CLASS PrincipalAxisNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include <ACG/GL/acg_glew.hh>
#include "PrincipalAxisNode.hh"
#include <ACG/GL/IRenderer.hh>
#include <ACG/GL/gl.hh>

#include <ACG/QtWidgets/QtPrincipalAxisDialog.hh>

#include <limits>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ========================================================== 


// void 
// MyGlutPrimitiveNode::
// initialize()
// {
//   std::cerr << "initialize MyGlutPrimitiveNode static members!\n";
//   update_display_lists();
//   initialized_ = true;
// }



/// Default constructor
PrincipalAxisNode::PrincipalAxisNode( BaseNode*         _parent,
                                const std::string&       _name)
  : BaseNode(_parent, _name),
    auto_range_(false),
    max_abs_value_(1.0),
    min_abs_value_(0.0),
    max_draw_radius_(1.0),
    min_draw_radius_(0.0),
    default_radius_(true),
    slices_(10),
    cylinder_radius_scale_(1.0),
    min_spacing_(0.0),
    draw_style_(DS_3D),
    color_mode_(CM_Axis),
    cone_height_factor_(2.5f),
    cylinder_(8, 2, 1.0f, true, false),
    cone_(8, 2, 2.0f / cone_height_factor_, 0.0f, true, false),
    invalidateInstanceData_(true),
    supportsInstancing_(-1),
    vbo_(0),
    updateVBO_(true) {

  static const Vec4f default_cols[3] = {
               Vec4f(0.91f, 0.11f, 0.09f, 1.0f),
               Vec4f(0.0f,   .43f, 1.0f , 1.0f),
               Vec4f(0.0f,  0.70f, 0.0f , 1.0f)
  };
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
          axes_colors[i][j] = default_cols[i][j];
      }
  }

  for(unsigned int i=0; i<3; ++i)
    show_tensor_component_[i] = 2;
}


//----------------------------------------------------------------------------


PrincipalAxisNode::~PrincipalAxisNode()
{
}

//----------------------------------------------------------------------------

void PrincipalAxisNode::set_color_mode(ColorMode _cm)
{
  if (color_mode_ != _cm)
  {
    color_mode_ = _cm;
    invalidateInstanceData_ = true;
  }
}

//----------------------------------------------------------------------------

void 
PrincipalAxisNode::
show_options_dialog()
{
  QtPrincipalAxisDialog* pd = new QtPrincipalAxisDialog(*this);
  pd->show();
}


//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
show_tensor_component(unsigned int _i, unsigned char _show)
{
  if (_i > 2) return;

  // instance data has to be invalidated if components change visibility.
  // toggling between single and double direction is possible without update
  if ((show_tensor_component_[_i] && !_show) || (!show_tensor_component_[_i] && _show))
    invalidateInstanceData_ = true;

  show_tensor_component_[_i] = _show;

  auto_update_range();
  updateVBO();
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
resize(size_t _n)
{
  size_t old_n = pc_.size();

  pc_.resize(_n);
  draw_pc_.resize(_n);

  // initialize new draw_pc_ values
  for(size_t i=old_n; i<_n; ++i)
    draw_pc_[i] = false;

  auto_update_range();
}


//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
enable ( size_t _i)
{
  if(_i < draw_pc_.size())
  {
    if (!draw_pc_[_i])
    {
      draw_pc_[_i] = true;
      invalidateInstanceData_ = true;
    }
  }
  else std::cerr << "principal component index out of range\n";
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
disable ( size_t _i)
{
  if(_i < draw_pc_.size())
  {
    if (!draw_pc_[_i])
    {
      draw_pc_[_i] = false;
      invalidateInstanceData_ = true;
    }
  }
  else std::cerr << "principal component index out of range\n";
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
disable_all()
{
  for(size_t i=0; i<pc_.size(); ++i)
    disable(i);
}


//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
set(size_t _i, const PrincipalComponent& _pc)
{
  if( _i < pc_.size())
  {
    // set values
    pc_[_i] = _pc;
    
    // update range
    if( auto_range_)
      auto_update_range();

    invalidateInstanceData_ = true;

//     // update bounding_box
//     update_bounding_box();
  }
  else std::cerr << "PrincipalComponent index error!\n";

  updateVBO();
}


//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
get(size_t _i, PrincipalComponent& _pc)
{
  if( _i < pc_.size())
  {
    // set values
    _pc = pc_[_i];
  }
  else std::cerr << "PrincipalComponent index error!\n";
}

//----------------------------------------------------------------------------


void
PrincipalAxisNode::
add(const PrincipalComponent& _pc, bool _enable)
{
  pc_.push_back(_pc);
  draw_pc_.push_back(_enable);

  // update range
  if( auto_range_)
    auto_update_range();

  invalidateInstanceData_ = true;

  updateVBO();
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
set_auto_range( bool _b)
{
  auto_range_ = _b;
}


//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
set_min_abs_value( double _v)
{
  if( auto_range_)
    std::cerr << "Warning: Auto update min/max abs_values is enabled! Setting has no effect.\n";

  min_abs_value_ = _v;
  invalidateInstanceData_ = true;
  updateVBO();
}


//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
set_max_abs_value( double _v)
{
  if( auto_range_)
    std::cerr << "Warning: Auto update min/max abs_values is enabled! Setting has no effect.\n";

  max_abs_value_ = _v;
  invalidateInstanceData_ = true;
  updateVBO();
}

//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
set_min_draw_radius( double _v)
{
  min_draw_radius_ = _v;
  default_radius_ = false;
  invalidateInstanceData_ = true;
  updateVBO();
}


//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
set_max_draw_radius( double _v)
{
  max_draw_radius_ = _v;
  default_radius_ = false;
  invalidateInstanceData_ = true;
  updateVBO();
}


//----------------------------------------------------------------------------


void 
PrincipalAxisNode::
auto_update_range()
{
  min_abs_value_ = std::numeric_limits<double>::max();
  max_abs_value_ = 0.0;

  for(size_t i=0; i<pc_.size(); ++i)
    if(draw_pc_[i])
      for(size_t j=0; j<3; ++j)
	if(show_tensor_component_[j])
	{
	  max_abs_value_ = std::max( max_abs_value_, pc_[i].a[j].norm() );
	  min_abs_value_ = std::min( min_abs_value_, pc_[i].a[j].norm() );
	}

  update_bounding_box();
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
update_bounding_box()
{
  bool uninitialized = true;
  
  bbMin_ = Vec3d( FLT_MAX, FLT_MAX, FLT_MAX);
  bbMax_ = Vec3d(-FLT_MAX,-FLT_MAX,-FLT_MAX);

  for(size_t i=0; i<pc_.size(); ++i)
  {
    Vec3d lmin = (pc_[i].p) - Vec3d(1,1,1)*max_draw_radius_;
    Vec3d lmax = (pc_[i].p) + Vec3d(1,1,1)*max_draw_radius_;

    if(draw_pc_[i])
    {
      if(uninitialized)
      {
        bbMin_ = lmin;
        bbMax_ = lmax;
        uninitialized = false;
      }
      else
      {
        bbMin_.minimize(lmin);
        bbMax_.maximize(lmax);
      }
    }

  }
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
boundingBox(Vec3d& _bbMin, Vec3d& _bbMax)
{
  if( bbMin_ != Vec3d(FLT_MAX, FLT_MAX, FLT_MAX) )
  {
    _bbMin.minimize( bbMin_);
    _bbMax.maximize( bbMax_);
  }
}


//----------------------------------------------------------------------------

  
DrawModes::DrawMode 
PrincipalAxisNode::
availableDrawModes() const
{
  return ( DrawModes::POINTS              |
	   DrawModes::WIREFRAME           |
	   DrawModes::HIDDENLINE          |
	   DrawModes::SOLID_FLAT_SHADED   |
	   DrawModes::SOLID_SMOOTH_SHADED );
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
draw(GLState& /*_state*/, const DrawModes::DrawMode& _drawMode)
{
  if( draw_style_ == DS_3D)
    ACG::GLState::enable(GL_LIGHTING);
  else
    ACG::GLState::disable(GL_LIGHTING);

  if (_drawMode & DrawModes::SOLID_SMOOTH_SHADED)
    ACG::GLState::shadeModel(GL_SMOOTH);
  else
    ACG::GLState::shadeModel(GL_FLAT);

  glMatrixMode(GL_MODELVIEW);

  for(unsigned int i=0; i<pc_.size(); ++i)
    if( draw_pc_[i] )
       draw_principal_component( pc_[i]);
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
draw_principal_component(const PrincipalComponent& _pc)
{
  glPushMatrix();
  glTranslated( _pc.p[0], _pc.p[1], _pc.p[2]);

  //  for(unsigned int i=0; i<1; ++i)
  for(size_t i=0; i<3; ++i)
  {
    if( ! show_tensor_component_[i]) continue;

    Vec3d a;
    a  = _pc.a[i];

    // compute arrow length in world coords
    double length;
    length = std::max( min_abs_value_, a.norm());
    length = std::min( max_abs_value_, length  );

    if( a.norm() > 1e-8 )
      a.normalize();

    // Bug fixed: Visualizing all unit vectors yieled scaled_length = nan
    double scaled_length(min_draw_radius_);
    if( fabs(max_abs_value_-min_abs_value_) > 1e-6) 
      scaled_length += (length-min_abs_value_)/(max_abs_value_-min_abs_value_)*(max_draw_radius_-min_draw_radius_);

    a *= scaled_length;

//     std::cerr << "**************************\n";
//     std::cerr << "min_abs_value: " << min_abs_value_ << std::endl;
//     std::cerr << "max_abs_value: " << max_abs_value_ << std::endl;
//     std::cerr << "min_draw_radius: " << min_draw_radius_ << std::endl;
//     std::cerr << "max_draw_radius: " << max_draw_radius_ << std::endl;
//     std::cerr << "scaled_length: " << scaled_length << std::endl;
//     std::cerr << "length       : " << length << std::endl;
//     std::cerr << "sign         : " << _pc.sign[i] << std::endl;
//     std::cerr << "**************************\n";


    if( color_mode_ == CM_Sign)
    {
      // choose color due to eigenvalue sign
      if(_pc.sign[i] == true)
      {
        glColor3f(1.f, 0.f, 0.f);

        GLfloat mat_amb_diff[] = {1.0, 0.0, 0.0, 1.0};
        glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_amb_diff);
        glColor4fv( mat_amb_diff);
      }
      else
      {
        glColor3f(0.f, 0.f, 1.f);

        GLfloat mat_amb_diff[] = {0.0, 0.0, 1.0, 1.0};
        glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_amb_diff);
        glColor4fv( mat_amb_diff);
      }
    }
    else
      // also default
    {
      // choose color due to eigenvalue
      glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, axes_colors[i]);
      glColor4fv( axes_colors[i]);
    }

    // draw both axis
    if( draw_style_ == DS_3D)
    {
      double radius = max_draw_radius_ * 0.015 * cylinder_radius_scale_;

      draw_arrow( a     , radius);

      if( show_tensor_component_[i] == 2)
      draw_arrow( a*(-1), radius);
    }
    else
    {
      double width = cylinder_radius_scale_;

      draw_line( a     , width);
      if( show_tensor_component_[i] == 2)
        draw_line( a*(-1), width);
    }
  }

  glPopMatrix();
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
draw_arrow( const Vec3d& _axis, double _r)
{
  double size        = _axis.norm();

  if( size > 1e-10)
  {
    glPushMatrix();


    Vec3d direction = _axis;
    Vec3d z_axis(0,0,1);
    Vec3d rot_normal;
    double rot_angle;

    direction.normalize();
    rot_angle  = acos((z_axis | direction))*180/M_PI;
    rot_normal = ((z_axis % direction).normalize());
  

    if(fabs(rot_angle) > 0.0001 && fabs(180-rot_angle) > 0.0001)
      glRotatef(rot_angle,rot_normal[0], rot_normal[1], rot_normal[2]);
    else
      glRotatef(rot_angle,1,0,0);

    glPushMatrix();
    glScalef(_r, _r, 0.85f*size);
    cylinder_.draw_primitive();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0, 0, 0.85f*size);
    glScalef(cone_height_factor_ * _r, cone_height_factor_ * _r, cone_height_factor_ * _r);
    cone_.draw_primitive();
    glPopMatrix();




    glPopMatrix();
  }
}


//----------------------------------------------------------------------------


void
PrincipalAxisNode::
draw_line( const Vec3d& _axis, double _w)
{
  glLineWidth(_w);

  glBegin(GL_LINES);
  glVertex3f(0.0f,0.0f,0.0f);
  glVertex3f(_axis[0], _axis[1], _axis[2]);
  glEnd();
}

//----------------------------------------------------------------------------


void
PrincipalAxisNode::
pick(GLState& /*_state*/, PickTarget _target)
{
  switch (_target)
  {
    case PICK_ANYTHING:
    case PICK_FACE: 
    { 
//       ACG::GLState::disable(GL_LIGHTING);
//       ACG::GLState::shadeModel(GL_FLAT);
//       glPushMatrix();
//       glTranslatef(position_[0], position_[1], position_[2]);
//       draw_obj();
//       glPopMatrix();
      break; 
    }

    default:
      break;
  }      
}


//----------------------------------------------------------------------------


void PrincipalAxisNode::set_axes_colors(const Vec4f colors[3]) {
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            axes_colors[i][j] = colors[i][j];
        }
    }

    invalidateInstanceData_ = true;
    updateVBO();
}

void PrincipalAxisNode::get_axes_colors(Vec4f out_colors[3]) const {
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            out_colors[i][j] = axes_colors[i][j];
        }
    }
}


//----------------------------------------------------------------------------


// Following code is taken from http://stackoverflow.com/questions/4372224/fast-method-for-computing-3x3-symmetric-matrix-spectral-decomposition:
// Slightly modified version of  Stan Melax's code for 3x3 matrix diagonalization (Thanks Stan!)
// source: http://www.melax.com/diag.html?attredirects=0
void PrincipalAxisNode::diagonalize(const double (&A)[3][3], double (&Q)[3][3], double (&D)[3][3])
{
    // A must be a symmetric matrix.
    // returns Q and D such that
    // Diagonal matrix D = QT * A * Q;  and  A = Q*D*QT
    const int maxsteps=24;  // certainly wont need that many.
    double o[3], m[3];
    double q [4] = {0.0,0.0,0.0,1.0};
    double jr[4];
    double AQ[3][3];
    for(int i=0;i < maxsteps;++i)
    {
        int k0, k1, k2;
        double sqw, sqx, sqy, sqz;
        double tmp1, tmp2, mq;
        double thet, sgn, t, c;

        // quat to matrix
        sqx      = q[0]*q[0];
        sqy      = q[1]*q[1];
        sqz      = q[2]*q[2];
        sqw      = q[3]*q[3];
        Q[0][0]  = ( sqx - sqy - sqz + sqw);
        Q[1][1]  = (-sqx + sqy - sqz + sqw);
        Q[2][2]  = (-sqx - sqy + sqz + sqw);
        tmp1     = q[0]*q[1];
        tmp2     = q[2]*q[3];
        Q[1][0]  = 2.0 * (tmp1 + tmp2);
        Q[0][1]  = 2.0 * (tmp1 - tmp2);
        tmp1     = q[0]*q[2];
        tmp2     = q[1]*q[3];
        Q[2][0]  = 2.0 * (tmp1 - tmp2);
        Q[0][2]  = 2.0 * (tmp1 + tmp2);
        tmp1     = q[1]*q[2];
        tmp2     = q[0]*q[3];
        Q[2][1]  = 2.0 * (tmp1 + tmp2);
        Q[1][2]  = 2.0 * (tmp1 - tmp2);

        // AQ = A * Q
        AQ[0][0] = Q[0][0]*A[0][0]+Q[1][0]*A[0][1]+Q[2][0]*A[0][2];
        AQ[0][1] = Q[0][1]*A[0][0]+Q[1][1]*A[0][1]+Q[2][1]*A[0][2];
        AQ[0][2] = Q[0][2]*A[0][0]+Q[1][2]*A[0][1]+Q[2][2]*A[0][2];
        AQ[1][0] = Q[0][0]*A[0][1]+Q[1][0]*A[1][1]+Q[2][0]*A[1][2];
        AQ[1][1] = Q[0][1]*A[0][1]+Q[1][1]*A[1][1]+Q[2][1]*A[1][2];
        AQ[1][2] = Q[0][2]*A[0][1]+Q[1][2]*A[1][1]+Q[2][2]*A[1][2];
        AQ[2][0] = Q[0][0]*A[0][2]+Q[1][0]*A[1][2]+Q[2][0]*A[2][2];
        AQ[2][1] = Q[0][1]*A[0][2]+Q[1][1]*A[1][2]+Q[2][1]*A[2][2];
        AQ[2][2] = Q[0][2]*A[0][2]+Q[1][2]*A[1][2]+Q[2][2]*A[2][2];
        // D = Qt * AQ
        D[0][0] = AQ[0][0]*Q[0][0]+AQ[1][0]*Q[1][0]+AQ[2][0]*Q[2][0];
        D[0][1] = AQ[0][0]*Q[0][1]+AQ[1][0]*Q[1][1]+AQ[2][0]*Q[2][1];
        D[0][2] = AQ[0][0]*Q[0][2]+AQ[1][0]*Q[1][2]+AQ[2][0]*Q[2][2];
        D[1][0] = AQ[0][1]*Q[0][0]+AQ[1][1]*Q[1][0]+AQ[2][1]*Q[2][0];
        D[1][1] = AQ[0][1]*Q[0][1]+AQ[1][1]*Q[1][1]+AQ[2][1]*Q[2][1];
        D[1][2] = AQ[0][1]*Q[0][2]+AQ[1][1]*Q[1][2]+AQ[2][1]*Q[2][2];
        D[2][0] = AQ[0][2]*Q[0][0]+AQ[1][2]*Q[1][0]+AQ[2][2]*Q[2][0];
        D[2][1] = AQ[0][2]*Q[0][1]+AQ[1][2]*Q[1][1]+AQ[2][2]*Q[2][1];
        D[2][2] = AQ[0][2]*Q[0][2]+AQ[1][2]*Q[1][2]+AQ[2][2]*Q[2][2];
        o[0]    = D[1][2];
        o[1]    = D[0][2];
        o[2]    = D[0][1];
        m[0]    = fabs(o[0]);
        m[1]    = fabs(o[1]);
        m[2]    = fabs(o[2]);

        k0      = (m[0] > m[1] && m[0] > m[2])?0: (m[1] > m[2])? 1 : 2; // index of largest element of offdiag
        k1      = (k0+1)%3;
        k2      = (k0+2)%3;
        if (o[k0]==0.0)
        {
            break;  // diagonal already
        }
        thet    = (D[k2][k2]-D[k1][k1])/(2.0*o[k0]);
        sgn     = (thet > 0.0)?1.0:-1.0;
        thet   *= sgn; // make it positive
        t       = sgn /(thet +((thet < 1.E6)?sqrt(thet*thet+1.0):thet)) ; // sign(T)/(|T|+sqrt(T^2+1))
        c       = 1.0/sqrt(t*t+1.0); //  c= 1/(t^2+1) , t=s/c
        if(c==1.0)
        {
            break;  // no room for improvement - reached machine precision.
        }
        jr[0 ]  = jr[1] = jr[2] = jr[3] = 0.0;
        jr[k0]  = sgn*sqrt((1.0-c)/2.0);  // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)
        jr[k0] *= -1.0; // since our quat-to-matrix convention was for v*M instead of M*v
        jr[3 ]  = sqrt(1.0f - jr[k0] * jr[k0]);
        if(jr[3]==1.0)
        {
            break; // reached limits of floating point precision
        }
        q[0]    = (q[3]*jr[0] + q[0]*jr[3] + q[1]*jr[2] - q[2]*jr[1]);
        q[1]    = (q[3]*jr[1] - q[0]*jr[2] + q[1]*jr[3] + q[2]*jr[0]);
        q[2]    = (q[3]*jr[2] + q[0]*jr[1] - q[1]*jr[0] + q[2]*jr[3]);
        q[3]    = (q[3]*jr[3] - q[0]*jr[0] - q[1]*jr[1] - q[2]*jr[2]);
        mq      = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0]   /= mq;
        q[1]   /= mq;
        q[2]   /= mq;
        q[3]   /= mq;
    }
}

void PrincipalAxisNode::getRenderObjects(IRenderer* _renderer, GLState& _state,
        const DrawModes::DrawMode& _drawMode,
        const ACG::SceneGraph::Material* _mat) {

    if (pc_.empty())
        return;


    if (draw_style_ == DS_2D && !lineDecl_.getNumElements())
    {
      // line vertex layout
      // float3 pos
      // float x_offset (for quad extrusion with line width in vertex shader)
      float lineVBOData[] =
      {
        0.0f, 0.0f, 1.0f, -1.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f, 1.0f,
        0.0f, 0.0f, 0.0f, -1.0f,
      };

      lineBuffer_.upload(sizeof(lineVBOData), lineVBOData, GL_STATIC_DRAW);

      lineDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
      lineDecl_.addElement(GL_FLOAT, 1, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_xOffset");
    }

    // check support for instancing if not done yet
    if (supportsInstancing_ < 0)
      supportsInstancing_ = checkExtensionSupported("GL_ARB_instanced_arrays") ? 1 : 0;
    if (supportsInstancing_)
    {
      RenderObject obj;

      obj.initFromState(&_state);
      obj.depthTest = true;

      if (!_drawMode.getLayer(0)->lighting())
        obj.shaderDesc.shadeMode = SG_SHADE_UNLIT;
      else
      {
        if (_drawMode.getLayer(0)->flatShaded())
          obj.shaderDesc.shadeMode = SG_SHADE_FLAT;
        else
          obj.shaderDesc.shadeMode = SG_SHADE_GOURAUD;
      }


      // count number of instances
      int numInstances = 0;

      int visibleTensors = 0;
      for (size_t k = 0; k < 3; ++k)
      {
        if (show_tensor_component_[k]) 
          ++visibleTensors;
      }

      for (size_t i = 0; i < pc_.size(); ++i)
      {
        if (draw_pc_[i])
          numInstances += visibleTensors;
      }

      if (!numInstances)
        return;

      if (invalidateInstanceData_)
      {
        /*
        data stored per instance:
        float4x3 world matrix
        byte4_unorm color
        */

        const int numDwordsPerInstance = 4 * 3 + 2;
        std::vector<float> instanceData(numInstances * numDwordsPerInstance);
        int instanceOffset = 0;

        for (unsigned int i = 0; i < pc_.size(); ++i)
        {
          if (draw_pc_[i])
          {
            const PrincipalComponent& pc = pc_[i];
            for (unsigned int k = 0; k < 3; ++k)
            {
              if (!show_tensor_component_[k]) continue;

              // compute axis transform
              double size = 1.0;
              GLMatrixd axisWorld = axisTransform(pc, k, &size);

              // store 4x3 part of axis transform

              for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 4; ++c)
                  instanceData[instanceOffset * numDwordsPerInstance + r * 4 + c] = axisWorld(r, c);

              // store size
              instanceData[instanceOffset * numDwordsPerInstance + 4 * 3] = size;

              // store color
              Vec4uc instanceColor(0, 0, 0, 255);

              if (color_mode_ == CM_Sign)
              {
                // choose color based on eigenvalue sign
                if (pc.sign[k] == true)
                  instanceColor = Vec4uc(255, 0, 0, 255);
                else
                  instanceColor = Vec4uc(0, 0, 255, 255);
              }
              else // choose color based on eigenvalue
              {
                for (int m = 0; m < 4; ++m)
                  instanceColor[m] = static_cast<unsigned char>(std::max(std::min(int(axes_colors[k][m] * 255.0f), int(255)), int(0)));
              }

              memcpy(&instanceData[instanceOffset * numDwordsPerInstance + 4 * 3 + 1], instanceColor.data(), 4);

              ++instanceOffset;
            }
          }
        }

        lineInstanceBuffer_.upload(instanceData.size() * 4, &instanceData[0], GL_STATIC_DRAW);

        lineDeclInstanced_.clear();
        // vbo data layout
        lineDeclInstanced_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
        lineDeclInstanced_.addElement(GL_FLOAT, 1, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_xOffset");
        // instance data layout
        lineDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_worldTransform0", 1, lineInstanceBuffer_.id());
        lineDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_worldTransform1", 1, lineInstanceBuffer_.id());
        lineDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_worldTransform2", 1, lineInstanceBuffer_.id());
        lineDeclInstanced_.addElement(GL_FLOAT, 1, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_size", 1, lineInstanceBuffer_.id());
        lineDeclInstanced_.addElement(GL_UNSIGNED_BYTE, 4, VERTEX_USAGE_COLOR, size_t(0), 0, 1, lineInstanceBuffer_.id());

        cylinderDeclInstanced_.clear();
        cylinderDeclInstanced_ = *cylinder_.getVertexDecl();
        cylinderDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_worldTransform0", 1, lineInstanceBuffer_.id());
        cylinderDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_worldTransform1", 1, lineInstanceBuffer_.id());
        cylinderDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_worldTransform2", 1, lineInstanceBuffer_.id());
        cylinderDeclInstanced_.addElement(GL_FLOAT, 1, VERTEX_USAGE_SHADER_INPUT, size_t(0), "pa_size", 1, lineInstanceBuffer_.id());
        cylinderDeclInstanced_.addElement(GL_UNSIGNED_BYTE, 4, VERTEX_USAGE_COLOR, size_t(0), 0, 1, lineInstanceBuffer_.id());

        invalidateInstanceData_ = false;
      }

      // scale and offset based on visualization mode (single or both directions)
      Vec3f pa_scale(0.0f, 0.0f, 0.0f), pa_offset(0.0f, 0.0f, 0.0f);

      int curTensor = 0;
      for (int k = 0; k < 3; ++k)
      {
        if (show_tensor_component_[k] == 2)
        {
          pa_scale[curTensor] = 2.0f;
          pa_offset[curTensor] = -1.0f;
        }
        else
        {
          pa_scale[curTensor] = 1.0f;
          pa_offset[curTensor] = 0.0f;
        }
        if (show_tensor_component_[k])
          ++curTensor;
      }

      if (draw_style_ == DS_2D)
      {
        // line object
        obj.debugName = "PrincipalAxisNode.line";
        obj.name = name() + std::string(".line");

        obj.shaderDesc.shadeMode = SG_SHADE_UNLIT;
        obj.shaderDesc.vertexColors = true;
        obj.shaderDesc.vertexTemplateFile = "PrincipalAxisNode/lines_extruded_instanced.glsl";

        obj.setUniform("pa_lineWidth", float(cylinder_radius_scale_));
        obj.setUniform("pa_screenSize", Vec2f(_state.viewport_width(), _state.viewport_height()));
        obj.setUniform("pa_scale", pa_scale);
        obj.setUniform("pa_offset", pa_offset);
        obj.setUniform("pa_visible_tensors", visibleTensors);

        obj.vertexBuffer = lineBuffer_.id();
        obj.vertexDecl = &lineDeclInstanced_;

        obj.glDrawInstancedArrays(GL_TRIANGLE_STRIP, 0, 4, numInstances);

        _renderer->addRenderObject(&obj);
      }
      else
      {
        float radius = max_draw_radius_ * 0.015 * cylinder_radius_scale_;

        obj.debugName = "PrincipalAxisNode.cylinder";
        obj.name = name() + std::string(".cylinder");

        obj.shaderDesc.vertexColors = true;
        obj.shaderDesc.vertexTemplateFile = "PrincipalAxisNode/3d_instanced.glsl";
        obj.shaderDesc.colorMaterialMode = GL_AMBIENT_AND_DIFFUSE;

        obj.ambient = obj.diffuse = Vec3f(1.0f, 1.0f, 1.0f);

        obj.setUniform("pa_cone_radius", radius);
        obj.setUniform("pa_cone_offset", Vec3f(0.0f, 0.0f, 0.0f));
        obj.setUniform("pa_scale", pa_scale * 0.85f);
        obj.setUniform("pa_offset", pa_offset * 0.85f);
        obj.setUniform("pa_visible_tensors", visibleTensors);

        obj.vertexBuffer = cylinder_.getVBO();
        obj.vertexDecl = &cylinderDeclInstanced_;

        obj.glDrawInstancedArrays(GL_TRIANGLES, 0, cylinder_.getNumTriangles() * 3, numInstances);

        _renderer->addRenderObject(&obj);

        // arrowheads
        obj.debugName = "PrincipalAxisNode.cone";

        obj.shaderDesc.vertexTemplateFile = "PrincipalAxisNode/3d_cone_instanced.glsl";

        obj.setUniform("pa_cone_radius", cone_height_factor_ * radius);
        obj.setUniform("pa_cone_offset", Vec3f(0.0f, 0.0f, 0.0f));
        obj.setUniform("pa_cone_mirror", 1.0f);

        obj.vertexBuffer = cone_.getVBO();

        obj.glDrawInstancedArrays(GL_TRIANGLES, 0, cone_.getNumTriangles() * 3, numInstances);

        _renderer->addRenderObject(&obj);

        if (show_tensor_component_[0] == 2 ||
          show_tensor_component_[1] == 2 ||
          show_tensor_component_[2] == 2)
        {
          obj.debugName = "PrincipalAxisNode.cone_mirror";
          obj.setUniform("pa_cone_mirror", -1.0f);

          _renderer->addRenderObject(&obj);
        }
      }
    }
    else
    {
      // non-instanced

//      emitIndividualRenderobjects(_renderer, _state, _drawMode, _mat);

      RenderObject ro;
      ro.initFromState(&_state);
      ro.setMaterial(_mat);

      nodeName_ = std::string("PrincipalAxisNode: ") + name();
      ro.debugName = nodeName_;

      ro.depthTest = true;
      ro.depthWrite = true;

      ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;

      // simulate line width via quad extrusion in geometry shader
      QString geomTemplate = ShaderProgGenerator::getShaderDir();
      geomTemplate += "Wireframe/geom_line2quad.tpl";

      ro.shaderDesc.geometryTemplateFile = geomTemplate;

      ro.setUniform("screenSize",
        Vec2f((float)_state.viewport_width(),
          (float)_state.viewport_height()));
      ro.setUniform("lineWidth", static_cast<float>(cylinder_radius_scale_));

      createVBO();
      ro.vertexBuffer = vbo_;
      // vertexDecl is defined in createVBO
      ro.vertexDecl = &vertexDecl_;

      int curOffset = 0;
      int vertexCount = static_cast<int>(pc_.size() * 2);
      for (int i = 0; i < 3; ++i) 
      {
        if (show_tensor_component_[i]) 
        {
          // set axis color
          for (int k = 0; k < 3; ++k)
            ro.emissive[k] = axes_colors[i][k];

          ro.glDrawArrays(GL_LINES, curOffset, vertexCount);
          _renderer->addRenderObject(&ro);

          curOffset += vertexCount;
        }
      }

    }
}

//----------------------------------------------------------------------------

void PrincipalAxisNode::emitIndividualRenderobjects(IRenderer* _renderer, GLState& _state, const DrawModes::DrawMode& _drawMode, const ACG::SceneGraph::Material* _mat)
{
  RenderObject obj;

  obj.initFromState(&_state);
  obj.depthTest = true;

  if (!_drawMode.getLayer(0)->lighting())
    obj.shaderDesc.shadeMode = SG_SHADE_UNLIT;
  else
  {
    if (_drawMode.getLayer(0)->flatShaded())
      obj.shaderDesc.shadeMode = SG_SHADE_FLAT;
    else
      obj.shaderDesc.shadeMode = SG_SHADE_GOURAUD;
  }


  for (size_t i = 0; i < pc_.size(); ++i)
  {
    if (draw_pc_[i])
    {
      const PrincipalComponent& pc = pc_[i];

      for (unsigned int k = 0; k < 3; ++k)
      {
        if (!show_tensor_component_[k]) continue;

        if (color_mode_ == CM_Sign)
        {
          // choose color due to eigenvalue sign
          if (pc.sign[k] == true)
            obj.ambient = obj.diffuse = Vec3f(1.0f, 0.0f, 0.0f);
          else
            obj.ambient = obj.diffuse = Vec3f(0.0f, 0.0f, 1.0f);
        }
        else // choose color due to eigenvalue
          obj.ambient = obj.diffuse = Vec3f(axes_colors[k][0], axes_colors[k][1], axes_colors[k][2]);

        double size = 1.0;
        GLMatrixd axisModelView = _state.modelview() * axisTransform(pc, k, &size);

        if (size > 1e-10)
        {
          // draw both axis
          if (draw_style_ == DS_3D)
          {
            double radius = max_draw_radius_ * 0.015 * cylinder_radius_scale_;

            // cylinder
            obj.name = name() + std::string(".cylinder");

            obj.modelview = axisModelView;

            if (show_tensor_component_[k] == 2)
            {
              obj.modelview.scale(radius, radius, 2.0 * 0.85 * size);
              obj.modelview.translate(0.0, 0.0, -0.5);
            }
            else
              obj.modelview.scale(radius, radius, 0.85 * size);

            obj.vertexDecl = cylinder_.getVertexDecl();
            obj.vertexBuffer = cylinder_.getVBO();

            obj.glDrawArrays(GL_TRIANGLES, 0, cylinder_.getNumTriangles() * 3);

            _renderer->addRenderObject(&obj);

            // arrowhead
            obj.name = name() + std::string(".cone");

            obj.modelview = axisModelView;
            obj.modelview.translate(0, 0, 0.85 * size);
            obj.modelview.scale(cone_height_factor_ * radius, cone_height_factor_ * radius, cone_height_factor_ * radius);

            obj.vertexDecl = cone_.getVertexDecl();
            obj.vertexBuffer = cone_.getVBO();

            obj.glDrawArrays(GL_TRIANGLES, 0, cone_.getNumTriangles() * 3);

            _renderer->addRenderObject(&obj);

            if (show_tensor_component_[k] == 2)
            {
              obj.modelview = axisModelView;
              obj.modelview.rotate(180, 1, 0, 0);
              obj.modelview.translate(0, 0, 0.85 * size);

              obj.modelview.scale(cone_height_factor_ * radius, cone_height_factor_ * radius, cone_height_factor_ * radius);

              _renderer->addRenderObject(&obj);
            }
          }
          else
          {
            // line object
            obj.name = name() + std::string(".line");

            obj.modelview = axisModelView;
            obj.shaderDesc.shadeMode = SG_SHADE_UNLIT;
            obj.emissive = obj.diffuse;

            if (show_tensor_component_[k] == 2)
            {
              obj.modelview.scale(1.0, 1.0, 2.0 * size);
              obj.modelview.translate(0.0, 0.0, -0.5);
            }
            else
              obj.modelview.scale(1.0, 1.0, size);

            obj.vertexBuffer = lineBuffer_.id();
            obj.vertexDecl = &lineDecl_;

            obj.glDrawArrays(GL_LINES, 0, 2);

            _renderer->addRenderObject(&obj);
          }
        }
      }
    }
  }
}

//----------------------------------------------------------------------------

Vec3d
PrincipalAxisNode::
axisScaled(const PrincipalComponent& _pc, int _axis) const
{
  Vec3d a;
  a = _pc.a[_axis];

  // compute arrow length in world coords
  double length;
  length = std::max(min_abs_value_, a.norm());
  length = std::min(max_abs_value_, length);

  if (a.norm() > 1e-8)
    a.normalize();

  // Bug fixed: Visualizing all unit vectors yieled scaled_length = nan
  double scaled_length(min_draw_radius_);

  if (fabs(max_abs_value_ - min_abs_value_) > 1e-6)
    scaled_length += (length - min_abs_value_) / (max_abs_value_ - min_abs_value_)*(max_draw_radius_ - min_draw_radius_);

  a *= scaled_length;
  return a;
}

//----------------------------------------------------------------------------

GLMatrixd
PrincipalAxisNode::
axisTransform(const PrincipalComponent& _pc, int _axis, double* _outSize) const
{
  assert(0 <= _axis && _axis < 3);

  GLMatrixd axisTransform;
  axisTransform.identity();
  axisTransform.translate(_pc.p);

  Vec3d a = axisScaled(_pc, _axis);
  double size = a.norm();

  if (_outSize)
    *_outSize = size;

  // orientation
  if (size > 1e-10)
  {
    Vec3d direction = a;
    Vec3d z_axis(0, 0, 1);
    Vec3d rot_normal;
    double rot_angle;
    direction.normalize();
    rot_angle = acos((z_axis | direction)) * 180 / M_PI;
    rot_normal = ((z_axis % direction).normalize());

    if (fabs(rot_angle) > 0.0001 && fabs(180 - rot_angle) > 0.0001)
      axisTransform.rotate(rot_angle, rot_normal[0], rot_normal[1], rot_normal[2]);
    else
      axisTransform.rotate(rot_angle, 1, 0, 0);
  }

  return axisTransform;
}

//----------------------------------------------------------------------------



void PrincipalAxisNode::createVBO() {
    if (!updateVBO_)
        return;

    // create vbo if it does not exist
    if (!vbo_)
        glGenBuffers(1, &vbo_);

    int tensorComponentCount = 0;
    for (int i = 0; i < 3; ++i) {
        tensorComponentCount += show_tensor_component_[i] ? 1 : 0;
    }
    vertexDecl_.clear();
    vertexDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);

    //3 coordinates per vertex, 2 vertices per principal component
    std::vector<float> vboData(3 * 2 * tensorComponentCount * pc_.size(), 0.f);

    float* vboPtr = &vboData[0];

    for (int tensor_component = 0; tensor_component < 3; ++tensor_component) {
        if (!show_tensor_component_[tensor_component]) continue;

        for (std::vector<PrincipalComponent>::const_iterator it = pc_.begin();
                it != pc_.end(); ++it) {

            Vec3d pc_dir = it->a[tensor_component];

            /*
             * Weird scale computation copied from draw().
             */
            const double pc_dir_norm = pc_dir.norm();
            double length;
            length = std::max( min_abs_value_, pc_dir_norm);
            length = std::min( max_abs_value_, length  );

            if( pc_dir_norm > 1e-8 ) pc_dir.normalize();

            // Bug fixed: Visualizing all unit vectors yieled scaled_length = nan
            double scaled_length(min_draw_radius_);
            if (fabs(max_abs_value_-min_abs_value_) > 1e-6)
              scaled_length += (length-min_abs_value_)/(max_abs_value_-min_abs_value_)*(max_draw_radius_-min_draw_radius_);

            pc_dir *= scaled_length;

            const Vec3d pc_from = it->p -
                    (show_tensor_component_[tensor_component] == 2
                        ? pc_dir
                        : Vec3d(0, 0, 0));
            const Vec3d pc_to = it->p + pc_dir;

            for (int i = 0; i < 3; ++i)
                *(vboPtr++) = pc_from[i];
            for (int i = 0; i < 3; ++i)
                *(vboPtr++) = pc_to[i];
        }
    }
    assert(vboPtr == &vboData[0] + vboData.size());

    glBindBuffer(GL_ARRAY_BUFFER_ARB, vbo_);
    glBufferData(GL_ARRAY_BUFFER_ARB, vboData.size() * sizeof(float),
            &vboData[0], GL_STATIC_DRAW_ARB);

    // Update done.
    updateVBO_ = false;
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

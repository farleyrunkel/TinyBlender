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
//  CLASS GLState - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include <ACG/GL/acg_glew.hh>
#include "GLState.hh"
#include <ACG/GL/removedEnums.hh>

#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <cstring>


//== NAMESPACES ===============================================================


namespace ACG {


//== IMPLEMENTATION ==========================================================

//#define GLSTATE_AVOID_REDUNDANT_GLCALLS

const Vec4f    GLState::default_clear_color(0.f, 0.f, 0.f, 1.f);
const Vec4f    GLState::default_base_color(0.f, 0.f, 0.f, 1.f);
const Vec4f    GLState::default_ambient_color(0.2f, 0.2f, 0.2f, 1.f);
const Vec4f    GLState::default_diffuse_color(0.5f, 0.53f, 0.6f, 1.f);
const Vec4f    GLState::default_specular_color(0.75f, 0.8f, 0.85f, 1.f);
const Vec4f    GLState::default_overlay_color(0.f, 0.f, 0.f, 1.f);
const float    GLState::default_shininess(100.f);


//-----------------------------------------------------------------------------

bool GLState::depthFuncLock_ = false;
bool GLState::depthRangeLock_ = false;
bool GLState::blendFuncSeparateLock_[] = { false };
bool GLState::blendEquationLock_ = false;
bool GLState::blendColorLock_ = false;
bool GLState::alphaFuncLock_ = false;
bool GLState::shadeModelLock_ = false;
bool GLState::cullFaceLock_ = false;
bool GLState::vertexPointerLock_ = false;
bool GLState::normalPointerLock_ = false;
bool GLState::texcoordPointerLock_ = false;
bool GLState::colorPointerLock_ = false;
bool GLState::drawBufferLock_ = false;
bool GLState::programLock_ = false;

std::deque <GLStateContext> GLState::stateStack_;
std::bitset<0xFFFF+1> GLState::glStateLock_;
int GLState::glBufferTargetLock_[] = {0};
int GLState::glTextureStageLock_[] = {0};
bool GLState::framebufferLock_[] = {false};
int GLState::maxTextureCoords_ = 0;
int GLState::maxCombinedTextureImageUnits_ = 0;
int GLState::maxDrawBuffers_ = 0;

int GLState::num_texture_units_ = 0;

GLStateContext::GLStateContext() :
    activeTexture_(GL_TEXTURE0),
    drawBufferSingle_(GL_BACK),
    activeDrawBuffer_(0),
    program_(0)
{
  framebuffers_[0] = framebuffers_[1] = 0;
  memset(drawBufferState_, GL_BACK, sizeof(drawBufferState_));
  blendFuncState_[0] = GL_SRC_ALPHA;
  blendFuncState_[1] = GL_ONE_MINUS_SRC_ALPHA;
  blendFuncState_[2] = GL_SRC_ALPHA;
  blendFuncState_[3] = GL_ONE_MINUS_SRC_ALPHA;
  texGenMode_ = GL_EYE_LINEAR;
}

GLState::GLState(bool _updateGL, bool _compatibilityProfile)
  : compatibilityProfile_(_compatibilityProfile),
    render_pass_(1),
    max_render_passes_(1),
    bb_min_(ACG::Vec3d(0.0,0.0,0.0)),
    bb_max_(ACG::Vec3d(0.0,0.0,0.0)),
    left_(-1),
    bottom_(1),
    width_(2),
    height_(2),
    glwidth_(2),
    glheight_(2),
    near_plane_(1.0),
    far_plane_(100.0),
    multisampling_(false),
    allow_multisampling_(true),
    mipmapping_(true),
    updateGL_(_updateGL),
    blending_(false),
    msSinceLastRedraw_ (1),
    colorPicking_(true)
{

  if ( stateStack_.empty() )
  {
    stateStack_.push_back(GLStateContext());

    memset(glBufferTargetLock_, 0, sizeof(glBufferTargetLock_));

    framebufferLock_[0] = framebufferLock_[1] = false;

    glStateLock_.reset();
  }

  initialize();
  ACG::compatibilityProfile(compatibilityProfile_);
}

//-----------------------------------------------------------------------------


void GLState::initialize()
{
  // clear matrix stacks
  while (!stack_projection_.empty())
    stack_projection_.pop();
  while (!stack_modelview_.empty())
    stack_modelview_.pop();
  while (!stack_inverse_projection_.empty())
    stack_inverse_projection_.pop();
  while (!stack_inverse_modelview_.empty())
    stack_inverse_modelview_.pop();


  // load identity matrix
  reset_projection();
  reset_modelview();


  // colors
  set_clear_color(default_clear_color);
  set_base_color(default_base_color);
  set_ambient_color(default_ambient_color);
  set_diffuse_color(default_diffuse_color);
  set_specular_color(default_specular_color);
  set_overlay_color(default_overlay_color);
  set_shininess(default_shininess);


  // thickness
  set_point_size(1.0f);
  set_line_width(1.0f);

  // multisampling
  set_multisampling(true);

  // Get max number of texture units
  GLint value;
  glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS_ARB, &value);

  num_texture_units_ = value;

  // lighting
  set_twosided_lighting(true);
}

//-----------------------------------------------------------------------------

void GLState::setState ()
{
  makeCurrent();

  if (compatibilityProfile_ ) {

    // projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(projection_.get_raw_data());
    glMatrixMode(GL_MODELVIEW);

    // modelview matrix
    glLoadMatrixd(modelview_.get_raw_data());

  }

  // clear color
  glClearColor(clear_color_[0], clear_color_[1], clear_color_[2], clear_color_[3]);

  if (compatibilityProfile_ ) {
    // base color
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, base_color_.data());

    // ambient color
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT , ambient_color_.data());

    // diffuse color
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE , diffuse_color_.data());

    // specular color
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular_color_.data());

    // shininess
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess_);
  }

  // point size
  glPointSize(point_size_);

  // line width
  glLineWidth(line_width_);

  if ( compatibilityProfile_ ) {
    // two sided lighting
    if (twosided_lighting_  )
      glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );
    else
      glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE );
  }

  // viewport
  glViewport(left_, bottom_, width_, height_);
}


//-----------------------------------------------------------------------------

void GLState::clearBuffers ()
{

  if ( compatibilityProfile_ ) {
    glPushAttrib (GL_ALL_ATTRIB_BITS);
  }

  GLState::disable(GL_DEPTH_TEST);
  GLState::disable(GL_DITHER);

  if ( compatibilityProfile_ ) {
    glShadeModel(GL_FLAT);

    GLState::disable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity ();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity ();
  }


  // GetoriginalScissor settings
  GLboolean scissor =  glIsEnabled(GL_SCISSOR_TEST);

  GLint origBox[4];
  glGetIntegerv(GL_SCISSOR_BOX,&origBox[0]);

  //Enable scissor
  if (!scissor)
    GLState::enable(GL_SCISSOR_TEST);

  // Restrict to our current viewport
  glScissor(  left_,bottom_,width_,height_ );

  // Clear restricted region
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  // Reset to originalsettings
  glScissor(  origBox[0], origBox[1], origBox[2], origBox[3] );

  if (!scissor)
    GLState::disable(GL_SCISSOR_TEST);

  if ( compatibilityProfile_ ) {
    glPopMatrix ();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopAttrib ();
  }
}

//-----------------------------------------------------------------------------

void GLState::setCompatibilityProfile( bool _compatibility ) {
  compatibilityProfile_ = _compatibility;
  ACG::compatibilityProfile(_compatibility);
}

bool GLState::compatibilityProfile() const  {
  return compatibilityProfile_;
}

//-----------------------------------------------------------------------------

void GLState::reset_projection()
{
  projection_.identity();
  inverse_projection_.identity();

  if (updateGL_ && compatibilityProfile_ )
  {
    makeCurrent();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
  }
}


//-----------------------------------------------------------------------------


void GLState::set_projection(const GLMatrixd& _m, const GLMatrixd& _inv_m)
{
  projection_ = _m;
  inverse_projection_ = _inv_m;

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(projection_.get_raw_data());
    glMatrixMode(GL_MODELVIEW);
  }
}


//-----------------------------------------------------------------------------


void GLState::reset_modelview()
{
  modelview_.identity();
  inverse_modelview_.identity();

  if (updateGL_ && compatibilityProfile_ )
  {
    makeCurrent();
    glLoadIdentity();
  }
}


//-----------------------------------------------------------------------------


void GLState::set_modelview(const GLMatrixd& _m, const GLMatrixd& _inv_m)
{
  modelview_ = _m;
  inverse_modelview_ = _inv_m;

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glLoadMatrixd(modelview_.get_raw_data());
  }
}


//-----------------------------------------------------------------------------


void GLState::ortho( double _left, double _right,
             double _bottom, double _top,
             double _n, double _f )
{
  near_plane_ = _n;
  far_plane_  = _f;

  projection_.ortho(_left, _right, _bottom, _top, _n, _f);
  inverse_projection_.inverse_ortho(_left,_right,_bottom,_top,_n,_f);

  if (updateGL_ && compatibilityProfile_ )
  {
    makeCurrent();
    glMatrixMode(GL_PROJECTION);
    glOrtho(_left, _right, _bottom, _top, _n, _f);
    glMatrixMode(GL_MODELVIEW);
  }
}


//-----------------------------------------------------------------------------


void GLState::frustum( double _left, double _right,
               double _bottom, double _top,
               double _n, double _f )
{
  near_plane_ = _n;
  far_plane_  = _f;

  projection_.frustum(_left, _right, _bottom, _top, _n, _f);
  inverse_projection_.inverse_frustum(_left,_right,_bottom,_top,_n,_f);

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMatrixMode(GL_PROJECTION);
    glFrustum(_left, _right, _bottom, _top, _n, _f);
    glMatrixMode(GL_MODELVIEW);
  }
}


//-----------------------------------------------------------------------------


void GLState::perspective( double _fovY, double _aspect,
               double _n, double _f )
{
  near_plane_ = _n;
  far_plane_  = _f;

  projection_.perspective(_fovY, _aspect, _n, _f);
  inverse_projection_.inverse_perspective(_fovY, _aspect, _n, _f);

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(projection_.data());
    glMatrixMode(GL_MODELVIEW);
  }
}


//-----------------------------------------------------------------------------


void GLState::viewport( int _left, int _bottom,
            int _width, int _height,
            int _glwidth, int _glheight)
{
  left_   = _left;
  bottom_ = _bottom;
  width_  = _width;
  height_ = _height;

  if (_glwidth < _width || _glheight < _height)
  {
    glwidth_ = _width;
    glheight_ = _height;
  } else {
    glwidth_ = _glwidth;
    glheight_ = _glheight;
  }

  window2viewport_.identity();
  window2viewport_(0,0) = 0.5f * width_;
  window2viewport_(0,3) = 0.5f * width_ + left_;
  window2viewport_(1,1) = 0.5f * height_;
  window2viewport_(1,3) = 0.5f * height_ + bottom_;
  window2viewport_(2,2) = 0.5f;
  window2viewport_(2,3) = 0.5f;

  inverse_window2viewport_.identity();
  inverse_window2viewport_(0,0) =  2.0f / width_;
  inverse_window2viewport_(0,3) = -(2.0*left_ + width_) / width_;
  inverse_window2viewport_(1,1) =  2.0f / height_;
  inverse_window2viewport_(1,3) = -(2.0*bottom_ + height_) / height_;
  inverse_window2viewport_(2,2) =  2.0f;
  inverse_window2viewport_(2,3) = -1.0f;

  if (updateGL_)
  {
    makeCurrent();
    glViewport(_left, _bottom, _width, _height);
  }
}


//-----------------------------------------------------------------------------


void GLState::lookAt( const Vec3d& _eye,
              const Vec3d& _center,
              const Vec3d& _up )
{
  modelview_.lookAt(_eye, _center, _up);
  inverse_modelview_.inverse_lookAt(_eye, _center, _up);

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glLoadMatrixd(modelview_.data());
  }
}


//-----------------------------------------------------------------------------


void GLState::translate( double _x, double _y, double _z,
             MultiplyFrom _mult_from )
{
  if (_mult_from == MULT_FROM_RIGHT)
  {
    modelview_.translate(_x, _y, _z);
    inverse_modelview_.translate(-_x, -_y, -_z, MULT_FROM_LEFT);
  }
  else
  {
    modelview_.translate(_x, _y, _z, MULT_FROM_LEFT);
    inverse_modelview_.translate(-_x, -_y, -_z);
  }

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glLoadMatrixd(modelview_.get_raw_data());
  }
}

//-----------------------------------------------------------------------------

void GLState::translate( Vec3d _vector,
                MultiplyFrom _mult_from ) {
  translate( _vector[0] , _vector[1] , _vector[2] ,_mult_from);
}

//-----------------------------------------------------------------------------


void GLState::rotate( double _angle, double _x, double _y, double _z,
              MultiplyFrom _mult_from )
{
  if (_mult_from == MULT_FROM_RIGHT)
  {
    modelview_.rotate(_angle, _x, _y, _z);
    inverse_modelview_.rotate(-_angle, _x, _y, _z, MULT_FROM_LEFT);
  }
  else
  {
    modelview_.rotate(_angle, _x, _y, _z, MULT_FROM_LEFT);
    inverse_modelview_.rotate(-_angle, _x, _y, _z);
  }

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glLoadMatrixd(modelview_.get_raw_data());
  }
}


//-----------------------------------------------------------------------------


void GLState::scale( double _sx, double _sy, double _sz,
             MultiplyFrom _mult_from )
{
  if (_mult_from == MULT_FROM_RIGHT)
  {
    modelview_.scale(_sx, _sy, _sz, MULT_FROM_RIGHT);
    inverse_modelview_.scale(1.0f/_sx, 1.0f/_sy, 1.0f/_sz, MULT_FROM_LEFT);
  }
  else
  {
    modelview_.scale(_sx, _sy, _sz, MULT_FROM_LEFT);
    inverse_modelview_.scale(1.0f/_sx, 1.0f/_sy, 1.0f/_sz, MULT_FROM_RIGHT);
  }

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glLoadMatrixd(modelview_.get_raw_data());
  }
}


//-----------------------------------------------------------------------------


void GLState::mult_matrix( const GLMatrixd& _m, const GLMatrixd& _inv_m,
               MultiplyFrom _mult_from )
{
  if (_mult_from == MULT_FROM_RIGHT)
  {
    modelview_ *= _m;
    inverse_modelview_.leftMult(_inv_m);
  }
  else
  {
    modelview_.leftMult(_m);
    inverse_modelview_ *= _inv_m;
  }

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glLoadMatrixd(modelview_.get_raw_data());
  }
}



//-----------------------------------------------------------------------------


Vec3d GLState::project(const Vec3d& _point) const
{
  Vec3d t = modelview_.transform_point(_point);
  t = projection_.transform_point(t);
  return window2viewport_.transform_point(t);
}


//-----------------------------------------------------------------------------


Vec3d GLState::unproject(const Vec3d& _winPoint) const
{
  Vec3d t = inverse_window2viewport_.transform_point(_winPoint);
  t = inverse_projection_.transform_point(t);
  return inverse_modelview_.transform_point(t);
}


//-----------------------------------------------------------------------------


void GLState::set_clear_color(const Vec4f& _col)
{
  clear_color_ = _col;

  if (updateGL_)
  {
    makeCurrent();
    glClearColor(_col[0], _col[1], _col[2], _col[3]);
  }
}


//-----------------------------------------------------------------------------


void GLState::set_base_color(const Vec4f& _col)
{
  base_color_ = _col;

  if (updateGL_ && compatibilityProfile_ )
  {
    makeCurrent();
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, _col.data());
  }
}

//-----------------------------------------------------------------------------


void GLState::set_color(const Vec4f& _col)
{
  color_ = _col;

  if (updateGL_)
  {
    makeCurrent();
    glColor(color_);
  }
}



//-----------------------------------------------------------------------------


void GLState::set_ambient_color(const Vec4f& _col)
{
  ambient_color_ = _col;

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, _col.data());
  }
}


//-----------------------------------------------------------------------------


void GLState::set_diffuse_color(const Vec4f& _col)
{
  diffuse_color_ = _col;

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, _col.data());
  }
}


//-----------------------------------------------------------------------------


void GLState::set_specular_color(const Vec4f& _col)
{
  specular_color_ = _col;

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, _col.data());
  }

}

//-----------------------------------------------------------------------------


void GLState::set_overlay_color(const Vec4f& _col)
{
  overlay_color_ = _col;
}


//-----------------------------------------------------------------------------


void GLState::set_shininess(float _shininess)
{    
   shininess_ = _shininess;

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, std::min(128.f, _shininess));
  }
}


//-----------------------------------------------------------------------------


void GLState::set_point_size(float _f)
{
  point_size_ = _f;

  if (updateGL_)
  {
    makeCurrent();
    glPointSize(point_size_);
  }
}


//-----------------------------------------------------------------------------


void GLState::set_line_width(float _f)
{
  line_width_ = _f;
if(compatibilityProfile())
{
  if (updateGL_)
  {
    makeCurrent();
    glLineWidth(line_width_);
  }
}
}

//-----------------------------------------------------------------------------

void GLState::set_bounding_box(ACG::Vec3d _min, ACG::Vec3d _max ) {
  bb_min_ = _min;
  bb_max_ = _max;
}

//-----------------------------------------------------------------------------

void GLState::get_bounding_box(ACG::Vec3d& _min, ACG::Vec3d& _max ) {
  _min = bb_min_;
  _max = bb_max_;
}


//-----------------------------------------------------------------------------


void GLState::set_twosided_lighting(bool _b)
{
  twosided_lighting_ = _b;

  if (updateGL_ && compatibilityProfile_ )
  {
    makeCurrent();
    if (twosided_lighting_)
      glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );
    else
      glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE );
  }
}


//-----------------------------------------------------------------------------

void GLState::set_multisampling(bool _b)
{

  multisampling_ = _b;

  if (updateGL_)
  {
    makeCurrent();
    if ( allow_multisampling_ ) {

      if ( _b )
        GLState::enable( GL_MULTISAMPLE );
      else
        GLState::disable( GL_MULTISAMPLE );

    } else {

      multisampling_ = false;

      if ( glIsEnabled( GL_MULTISAMPLE ) )
        GLState::disable( GL_MULTISAMPLE );

    }
  }

}

//-----------------------------------------------------------------------------

double GLState::fovy() const
{
    assert(projection_(1,1) != 0.0);

    return atan(1.0/projection_(1,1))*2.0;
}

//-----------------------------------------------------------------------------

double GLState::aspect() const
{
    assert(projection_(0,0) != 0.0);

    return projection_(1,1) / projection_(0,0);
}

//-----------------------------------------------------------------------------

Vec3d GLState::eye() const
{
  return inverse_modelview_.transform_point(Vec3d(0.0, 0.0, 0.0));
}

//-----------------------------------------------------------------------------


Vec3d GLState::viewing_direction(int _x, int _y) const
{
  Vec3d dir = ( unproject(Vec3d(_x, _y, 1.0)) -
        unproject(Vec3d(_x, _y, 0.0)) );
  dir.normalize();
  return dir;
}


//-----------------------------------------------------------------------------


Vec3d GLState::up() const
{
  Vec3d dir( unproject(Vec3d(0.5*width_, height_-1,   0.0)) -
         unproject(Vec3d(0.5*width_, 0.5*height_, 0.0)) );
  dir.normalize();
  return dir;
}


//-----------------------------------------------------------------------------


Vec3d GLState::right() const
{
  Vec3d dir( unproject(Vec3d(width_-1,   0.5*height_, 0.0)) -
         unproject(Vec3d(0.5*width_, 0.5*height_, 0.0)) );
  dir.normalize();
  return dir;
}


//-----------------------------------------------------------------------------


void GLState::viewing_ray( int _x, int _y,
               Vec3d& _origin, Vec3d& _direction) const
{
  _origin = unproject(Vec3d(_x, _y, 0.0));
  _direction = unproject(Vec3d(_x, _y, 1.0)) - _origin;
  _direction.normalize();
}


//-----------------------------------------------------------------------------

const GLenum& GLState::depthFunc() const
{
  return stateStack_.back().depthFunc_;
}

//-----------------------------------------------------------------------------

void GLState:: set_depthFunc(const GLenum& _depth_func)
{
  depthFunc(_depth_func);
}

//-----------------------------------------------------------------------------

void GLState::depthFunc(GLenum _depthFunc)
{
  if (!depthFuncLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().depthFunc_ != _depthFunc)
#endif
    {
      glDepthFunc(_depthFunc);
      stateStack_.back().depthFunc_ = _depthFunc;
    }
  }
}

//-----------------------------------------------------------------------------

void GLState::push_projection_matrix()
{
  stack_projection_.push(projection_);
  stack_inverse_projection_.push(inverse_projection_);

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
  }
}


//-----------------------------------------------------------------------------


void GLState::pop_projection_matrix()
{
  projection_ = stack_projection_.top();
  inverse_projection_ = stack_inverse_projection_.top();

  stack_projection_.pop();
  stack_inverse_projection_.pop();

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
  }
}


//-----------------------------------------------------------------------------


void GLState::push_modelview_matrix()
{
  stack_modelview_.push(modelview_);
  stack_inverse_modelview_.push(inverse_modelview_);

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glPushMatrix();
  }
}


//-----------------------------------------------------------------------------


void GLState::pop_modelview_matrix()
{
  modelview_ = stack_modelview_.top();
  inverse_modelview_ = stack_inverse_modelview_.top();

  stack_modelview_.pop();
  stack_inverse_modelview_.pop();

  if (updateGL_ && compatibilityProfile_)
  {
    makeCurrent();
    glPopMatrix();
  }
}

//-----------------------------------------------------------------------------

void GLState::pick_init (bool _color)
{
  colorPicking_ = _color;
  colorStack_.initialize ( this);
}

//-----------------------------------------------------------------------------

bool GLState::pick_set_maximum (size_t _idx)
{
  bool rv = colorStack_.setMaximumIndex (_idx);
  if (colorPicking_)
    return rv;
  return true;
}

//-----------------------------------------------------------------------------

void GLState::pick_set_name (size_t _idx)
{
  colorStack_.setIndex (_idx);
}

//-----------------------------------------------------------------------------

Vec4uc GLState::pick_get_name_color (size_t _idx)
{
  if (colorPicking_)
    return colorStack_.getIndexColor (_idx);
  return Vec4uc (0, 0, 0, 0);
}

//-----------------------------------------------------------------------------

Vec4f GLState::pick_get_name_color_norm (unsigned int _idx)
{
  Vec4f rv(0.0f, 0.0f, 0.0f, 0.0f);
  if (colorPicking_)
  {
    Vec4uc color_abs = colorStack_.getIndexColor(_idx);
    rv = OpenMesh::vector_cast<Vec4f, Vec4uc>(color_abs) / 255.0f;
  }
  return rv;
}

//-----------------------------------------------------------------------------

void GLState::pick_push_name (size_t _idx)
{
  colorStack_.pushIndex (_idx);
}

//-----------------------------------------------------------------------------

void GLState::pick_pop_name ()
{
  colorStack_.popIndex ();
}

//-----------------------------------------------------------------------------

std::vector<size_t> GLState::pick_color_to_stack (Vec4uc _rgba) const
{
  if (colorPicking_ && colorStack_.initialized ())
    return colorStack_.colorToStack (_rgba);
  return std::vector<size_t> ();
}

//-----------------------------------------------------------------------------

size_t GLState::pick_free_indicies () const
{
  if (colorPicking_ && colorStack_.initialized ())
    return colorStack_.freeIndicies ();
  return -1;
}

//-----------------------------------------------------------------------------

bool GLState::pick_error () const
{
  if (colorPicking_)
    return colorStack_.error ();
  return false;
}

//-----------------------------------------------------------------------------

size_t GLState::pick_current_index () const
{
  if (colorPicking_)
    return colorStack_.currentIndex ();
  else
    return 0;
}

//-----------------------------------------------------------------------------

bool GLState::color_picking () const
{
  return colorPicking_;
}

//-----------------------------------------------------------------------------

GLenum GLState::glStateCaps[] = {GL_ALPHA_TEST,
GL_AUTO_NORMAL,
GL_MAP2_VERTEX_3,
GL_MAP2_VERTEX_4,
GL_BLEND,
GL_CLIP_PLANE0,
GL_CLIP_PLANE1,
GL_CLIP_PLANE2,
GL_CLIP_PLANE3,
GL_CLIP_PLANE4,
GL_CLIP_PLANE5,
GL_COLOR_LOGIC_OP,
GL_COLOR_MATERIAL,
GL_COLOR_SUM,
GL_COLOR_TABLE,
GL_CONVOLUTION_1D,
GL_CONVOLUTION_2D,
GL_CULL_FACE,
GL_DEPTH_TEST,
GL_DITHER,
GL_FOG,
GL_HISTOGRAM,
GL_INDEX_LOGIC_OP,
GL_LIGHT0,
GL_LIGHT1,
GL_LIGHT2,
GL_LIGHT3,
GL_LIGHT4,
GL_LIGHT5,
GL_LIGHT6,
GL_LIGHT7,
GL_LIGHTING,
GL_LINE_SMOOTH,
GL_LINE_STIPPLE,
GL_MAP1_COLOR_4,
GL_MAP1_INDEX,
GL_MAP1_NORMAL,
GL_MAP1_TEXTURE_COORD_1,
GL_MAP1_TEXTURE_COORD_2,
GL_MAP1_TEXTURE_COORD_3,
GL_MAP1_TEXTURE_COORD_4,
GL_MAP1_VERTEX_3,
GL_MAP1_VERTEX_4,
GL_MAP2_COLOR_4,
GL_MAP2_INDEX,
GL_MAP2_NORMAL,
GL_MAP2_TEXTURE_COORD_1,
GL_MAP2_TEXTURE_COORD_2,
GL_MAP2_TEXTURE_COORD_3,
GL_MAP2_TEXTURE_COORD_4,
GL_MAP2_VERTEX_3,
GL_MAP2_VERTEX_4,
GL_MINMAX,
GL_MULTISAMPLE,
GL_NORMALIZE,
GL_RESCALE_NORMAL,
GL_POINT_SMOOTH,
GL_POINT_SPRITE,
GL_POLYGON_OFFSET_FILL,
GL_FILL,
GL_POLYGON_OFFSET_LINE,
GL_LINE,
GL_POLYGON_OFFSET_POINT,
GL_POINT,
GL_POLYGON_SMOOTH,
GL_POLYGON_STIPPLE,
GL_POST_COLOR_MATRIX_COLOR_TABLE,
GL_POST_CONVOLUTION_COLOR_TABLE,
GL_RESCALE_NORMAL,
GL_NORMALIZE,
GL_SAMPLE_ALPHA_TO_COVERAGE,
GL_SAMPLE_ALPHA_TO_ONE,
GL_SAMPLE_COVERAGE,
GL_SAMPLE_COVERAGE_INVERT,
GL_SEPARABLE_2D,
GL_SCISSOR_TEST,
GL_STENCIL_TEST,
GL_TEXTURE_1D,
GL_TEXTURE_2D,
GL_TEXTURE_3D,
GL_TEXTURE_CUBE_MAP,
GL_TEXTURE_GEN_Q,
GL_TEXTURE_GEN_R,
GL_TEXTURE_GEN_S,
GL_TEXTURE_GEN_T,
GL_VERTEX_PROGRAM_POINT_SIZE,
GL_VERTEX_PROGRAM_TWO_SIDE,
GL_COLOR_ARRAY,
GL_EDGE_FLAG_ARRAY,
GL_FOG_COORD_ARRAY,
GL_INDEX_ARRAY,
GL_NORMAL_ARRAY,
GL_SECONDARY_COLOR_ARRAY,
GL_TEXTURE_COORD_ARRAY,
GL_VERTEX_ARRAY};

void GLState::syncFromGL()
{
  // get enabled states
  GLenum caps[] = {GL_ALPHA_TEST,
    GL_AUTO_NORMAL,
    GL_MAP2_VERTEX_3,
    GL_MAP2_VERTEX_4,
    GL_BLEND,
    GL_CLIP_PLANE0,
    GL_CLIP_PLANE1,
    GL_CLIP_PLANE2,
    GL_CLIP_PLANE3,
    GL_CLIP_PLANE4,
    GL_CLIP_PLANE5,
    GL_COLOR_LOGIC_OP,
    GL_COLOR_MATERIAL,
    GL_COLOR_SUM,
    GL_COLOR_TABLE,
    GL_CONVOLUTION_1D,
    GL_CONVOLUTION_2D,
    GL_CULL_FACE,
    GL_DEPTH_TEST,
    GL_DITHER,
    GL_FOG,
    GL_HISTOGRAM,
    GL_INDEX_LOGIC_OP,
    GL_LIGHT0,
    GL_LIGHT1,
    GL_LIGHT2,
    GL_LIGHT3,
    GL_LIGHT4,
    GL_LIGHT5,
    GL_LIGHT6,
    GL_LIGHT7,
    GL_LIGHTING,
    GL_LINE_SMOOTH,
    GL_LINE_STIPPLE,
    GL_MAP1_COLOR_4,
    GL_MAP1_INDEX,
    GL_MAP1_NORMAL,
    GL_MAP1_TEXTURE_COORD_1,
    GL_MAP1_TEXTURE_COORD_2,
    GL_MAP1_TEXTURE_COORD_3,
    GL_MAP1_TEXTURE_COORD_4,
    GL_MAP1_VERTEX_3,
    GL_MAP1_VERTEX_4,
    GL_MAP2_COLOR_4,
    GL_MAP2_INDEX,
    GL_MAP2_NORMAL,
    GL_MAP2_TEXTURE_COORD_1,
    GL_MAP2_TEXTURE_COORD_2,
    GL_MAP2_TEXTURE_COORD_3,
    GL_MAP2_TEXTURE_COORD_4,
    GL_MAP2_VERTEX_3,
    GL_MAP2_VERTEX_4,
    GL_MINMAX,
    GL_MULTISAMPLE,
    GL_NORMALIZE,
    GL_RESCALE_NORMAL,
    GL_POINT_SMOOTH,
    GL_POINT_SPRITE,
    GL_POLYGON_OFFSET_FILL,
    GL_POLYGON_OFFSET_LINE,
    GL_POLYGON_OFFSET_POINT,
    GL_POLYGON_SMOOTH,
    GL_POLYGON_STIPPLE,
    GL_POST_COLOR_MATRIX_COLOR_TABLE,
    GL_POST_CONVOLUTION_COLOR_TABLE,
    GL_RESCALE_NORMAL,
    GL_NORMALIZE,
    GL_SAMPLE_ALPHA_TO_COVERAGE,
    GL_SAMPLE_ALPHA_TO_ONE,
    GL_SAMPLE_COVERAGE,
    GL_SEPARABLE_2D,
    GL_SCISSOR_TEST,
    GL_STENCIL_TEST,
    GL_TEXTURE_1D,
    GL_TEXTURE_2D,
    GL_TEXTURE_3D,
    GL_TEXTURE_CUBE_MAP,
    GL_TEXTURE_GEN_Q,
    GL_TEXTURE_GEN_R,
    GL_TEXTURE_GEN_S,
    GL_TEXTURE_GEN_T,
    GL_VERTEX_PROGRAM_POINT_SIZE,
    GL_VERTEX_PROGRAM_TWO_SIDE,
    GL_COLOR_ARRAY,
    GL_EDGE_FLAG_ARRAY,
    GL_FOG_COORD_ARRAY,
    GL_INDEX_ARRAY,
    GL_NORMAL_ARRAY,
    GL_SECONDARY_COLOR_ARRAY,
    GL_TEXTURE_COORD_ARRAY,
    GL_VERTEX_ARRAY};

  for (unsigned int i = 0; i < sizeof(caps) / sizeof(GLenum); ++i)
  {
    if (glIsEnabled(caps[i])) stateStack_.back().glStateEnabled_.set(caps[i]);
    else stateStack_.back().glStateEnabled_.reset(caps[i]);
  }

  GLint getparam;

#ifdef GL_VERSION_1_4
  glGetIntegerv(GL_BLEND_SRC_RGB, &getparam);
  stateStack_.back().blendFuncState_[0] = getparam;

  glGetIntegerv(GL_BLEND_DST_ALPHA, &getparam);
  stateStack_.back().blendFuncState_[1] = getparam;

  glGetIntegerv(GL_BLEND_SRC_ALPHA, &getparam);
  stateStack_.back().blendFuncState_[2] = getparam;

  glGetIntegerv(GL_BLEND_DST_ALPHA, &getparam);
  stateStack_.back().blendFuncState_[3] = getparam;
#else
  glGetIntegerv(GL_BLEND_SRC, &getparam);
  stateStack_.back().blendFuncState_[0] = getparam;

  glGetIntegerv(GL_BLEND_DST, &getparam);
  stateStack_.back().blendFuncState_[1] = getparam;
#endif


  glGetIntegerv(GL_BLEND_EQUATION_RGB, &getparam);
  stateStack_.back().blendEquationState_ = getparam;

  glGetFloatv(GL_BLEND_COLOR, stateStack_.back().blendColorState_);

  glGetIntegerv(GL_ALPHA_TEST_FUNC, &getparam);
  stateStack_.back().alphaFuncState_ = getparam;

  glGetFloatv(GL_ALPHA_TEST_REF, &stateStack_.back().alphaRefState_);

  glGetIntegerv(GL_DEPTH_FUNC, &getparam);
  stateStack_.back().depthFunc_ = getparam;

  glGetDoublev(GL_DEPTH_RANGE, stateStack_.back().depthRange_);

  // bound buffers

  GLenum bufGets[8] = {
    GL_ARRAY_BUFFER_BINDING, GL_ARRAY_BUFFER,
    GL_ELEMENT_ARRAY_BUFFER_BINDING, GL_ELEMENT_ARRAY_BUFFER,
    GL_PIXEL_PACK_BUFFER_BINDING, GL_PIXEL_PACK_BUFFER,
    GL_PIXEL_UNPACK_BUFFER_BINDING, GL_PIXEL_UNPACK_BUFFER};

  for (int i = 0; i < 4; ++i)
    glGetIntegerv(bufGets[i*2], (GLint*)stateStack_.back().glBufferTargetState_ + getBufferTargetIndex(bufGets[i*2+1]));


  // bound textures
  glGetIntegerv(GL_ACTIVE_TEXTURE, &getparam);
  stateStack_.back().activeTexture_ = getparam;

  GLenum texBufGets[] = {
    GL_TEXTURE_BINDING_1D, GL_TEXTURE_1D,
    GL_TEXTURE_BINDING_2D, GL_TEXTURE_2D,
    GL_TEXTURE_BINDING_3D, GL_TEXTURE_3D,
    GL_TEXTURE_BINDING_CUBE_MAP, GL_TEXTURE_CUBE_MAP
    , GL_TEXTURE_BINDING_RECTANGLE_ARB, GL_TEXTURE_RECTANGLE_ARB
  };

  glGetIntegerv(GL_MAX_TEXTURE_COORDS, &maxTextureCoords_);
  glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &maxCombinedTextureImageUnits_);

  // safe clamp
  if (maxTextureCoords_ > 16) maxTextureCoords_ = 16;
  if (maxCombinedTextureImageUnits_ > 16) maxCombinedTextureImageUnits_ = 16;

  int numTexUnits = maxTextureCoords_;
  if (numTexUnits < maxCombinedTextureImageUnits_) numTexUnits = maxCombinedTextureImageUnits_;

  for (int i = 0; i < numTexUnits; ++i)
  {
    glActiveTexture(GL_TEXTURE0 + i);

    getparam = 0;
    // for each texture stage query 5 texture types: 1D, 2D, 3D, Cube, Rect
    for (int k = 0; k < 5 && !getparam; ++k)
    {
      glGetIntegerv(texBufGets[k*2], &getparam);
      if (getparam)
      {
        stateStack_.back().glTextureStage_[i].buf_ = getparam;
        stateStack_.back().glTextureStage_[i].target_ = texBufGets[k*2+1];
      }
    }
  }

  // restore active texture unit
  if (numTexUnits > 0)
    glActiveTexture(stateStack_.back().activeTexture_);


  // shade model
  glGetIntegerv(GL_SHADE_MODEL, &getparam);
  stateStack_.back().shadeModel_ = getparam;

  // cull face
  glGetIntegerv(GL_CULL_FACE_MODE, &getparam);
  stateStack_.back().cullFace_ = getparam;


  // vertex pointers
  {
    GLenum ptrEnums[] = {
      GL_VERTEX_ARRAY_SIZE, GL_VERTEX_ARRAY_TYPE,
      GL_VERTEX_ARRAY_STRIDE, GL_VERTEX_ARRAY_POINTER,
      GL_COLOR_ARRAY_SIZE,  GL_COLOR_ARRAY_TYPE,
      GL_COLOR_ARRAY_STRIDE, GL_COLOR_ARRAY_POINTER,
      GL_TEXTURE_COORD_ARRAY_SIZE, GL_TEXTURE_COORD_ARRAY_TYPE,
      GL_TEXTURE_COORD_ARRAY_STRIDE, GL_TEXTURE_COORD_ARRAY_POINTER};

    GLStateContext::GLVertexPointer* ptrs[] = {&stateStack_.back().vertexPointer_,
      &stateStack_.back().colorPointer_, &stateStack_.back().texcoordPointer_};

    for (int i = 0; i < 3 ; ++i)
    {
      glGetIntegerv(ptrEnums[i*4], &getparam);
      ptrs[i]->size = getparam;
      glGetIntegerv(ptrEnums[i*4+1], &getparam);
      ptrs[i]->type = getparam;
      glGetIntegerv(ptrEnums[i*4+2], &getparam);
      ptrs[i]->stride = getparam;
      glGetPointerv(ptrEnums[i*4+3], (GLvoid**)&ptrs[i]->pointer);
    }

    glGetIntegerv(GL_NORMAL_ARRAY_STRIDE, &getparam);
    stateStack_.back().normalPointer_.size = getparam;
    glGetIntegerv(GL_NORMAL_ARRAY_TYPE, &getparam);
    stateStack_.back().normalPointer_.type = getparam;
    glGetPointerv(GL_NORMAL_ARRAY_POINTER, (GLvoid**)&stateStack_.back().normalPointer_.pointer);
  }


  // draw buffer state
  glGetIntegerv(GL_MAX_DRAW_BUFFERS, &maxDrawBuffers_);
  if (maxDrawBuffers_ > 16) maxDrawBuffers_ = 16;

  for (int i = 0; i < maxDrawBuffers_; ++i)
  {
    glGetIntegerv(GL_DRAW_BUFFER0 + i, &getparam);
    stateStack_.back().drawBufferState_[i] = getparam;
  }

  glGetIntegerv(GL_DRAW_BUFFER, &getparam);
  stateStack_.back().drawBufferSingle_ = getparam;

  // framebuffer
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &getparam);
  stateStack_.back().framebuffers_[0] = getparam;
  glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &getparam);
  stateStack_.back().framebuffers_[1] = getparam;

  // shader program
  glGetIntegerv(GL_CURRENT_PROGRAM, &getparam);
  stateStack_.back().program_ = getparam;
}

//-----------------------------------------------------------------------------


void GLState::enable(GLenum _cap, bool _warn)
{
  if (!glStateLock_.test(_cap))
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (!stateStack_.back().glStateEnabled_.test(_cap))
#endif
    {
        if (ACG::compatibilityProfile() || removedEnums.find(_cap) == removedEnums.end())
            glEnable(_cap);
        else
        {
            if(_warn)
            std::cerr << "OpenGL Warning:You are trying to use a GLenum that has been removed for OpenGL core profiles. ( Enum Nuber is: " << _cap << ")" << std::endl;
        }
      stateStack_.back().glStateEnabled_.set(_cap);
    }
  }
}

void GLState::disable(GLenum _cap, bool _warn)
{
  if (!glStateLock_.test(_cap))
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().glStateEnabled_.test(_cap))
#endif
    {
        if (ACG::compatibilityProfile() || removedEnums.find(_cap) == removedEnums.end())
            glDisable(_cap);
        else
        {
            if (_warn)
                std::cerr << "OpenGL Warning:You are trying to use a GLenum that has been removed for OpenGL core profiles." << std::endl;
        }
      stateStack_.back().glStateEnabled_.reset(_cap);
    }
  }
}

void GLState::lockState(GLenum _cap)
{
  glStateLock_.set(_cap);
}

void GLState::unlockState(GLenum _cap)
{
  glStateLock_.reset(_cap);
}

bool GLState::isStateLocked(GLenum _cap)
{
  return glStateLock_.test(_cap);
}

bool GLState::isStateEnabled(GLenum _cap)
{
  return stateStack_.back().glStateEnabled_.test(_cap);
}

//-----------------------------------------------------------------------------
// client state functions

void GLState::enableClientState(GLenum _cap)
{
  if (!glStateLock_.test(_cap))
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (!stateStack_.back().glStateEnabled_.test(_cap))
#endif
    {
      glEnableClientState(_cap);
      stateStack_.back().glStateEnabled_.set(_cap);
    }
  }
}

void GLState::disableClientState(GLenum _cap)
{
  if (!glStateLock_.test(_cap))
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().glStateEnabled_.test(_cap))
#endif
    {
      glDisableClientState(_cap);
      stateStack_.back().glStateEnabled_.reset(_cap);
    }
  }
}

void GLState::lockClientState(GLenum _cap)
{
  glStateLock_.set(_cap);
}

void GLState::unlockClientState(GLenum _cap)
{
  glStateLock_.reset(_cap);
}

bool GLState::isClientStateLocked(GLenum _cap)
{
  return glStateLock_.test(_cap);
}

bool GLState::isClientStateEnabled(GLenum _cap)
{
  return stateStack_.back().glStateEnabled_.test(_cap);
}

//-----------------------------------------------------------------------------
// blending functions

void GLState::blendFuncSeparate(GLenum _srcRGB, GLenum _dstRGB, GLenum _srcAlpha, GLenum _dstAlpha)
{
  // fix parameters according to lock status
  if (blendFuncSeparateLock_[0])
  {
    _srcRGB = stateStack_.back().blendFuncState_[0];
    _dstRGB = stateStack_.back().blendFuncState_[1];
  }

  if (blendFuncSeparateLock_[1])
  {
    _srcAlpha = stateStack_.back().blendFuncState_[2];
    _dstAlpha = stateStack_.back().blendFuncState_[3];
  }

  if (!blendFuncSeparateLock_[0] || !blendFuncSeparateLock_[1])
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().blendFuncState_[0] != _srcRGB || stateStack_.back().blendFuncState_[1] != _dstRGB ||
      stateStack_.back().blendFuncState_[2] != _srcAlpha || stateStack_.back().blendFuncState_[3] != _dstAlpha)
#endif
    {
#ifdef GL_VERSION_1_4
      // check if glew has loaded glBlendFuncSeparate already
      if (glBlendFuncSeparate)
        glBlendFuncSeparate(_srcRGB, _dstRGB, _srcAlpha, _dstAlpha);
      else
        glBlendFunc(_srcRGB, _dstRGB);
      stateStack_.back().blendFuncState_[0] = _srcRGB;
      stateStack_.back().blendFuncState_[1] = _dstRGB;
      stateStack_.back().blendFuncState_[2] = _srcAlpha;
      stateStack_.back().blendFuncState_[3] = _dstAlpha;
#else
      glBlendFunc(_srcRGB, _dstRGB);
      stateStack_.back().blendFuncState_[0] = _srcRGB;
      stateStack_.back().blendFuncState_[1] = _dstRGB;
      stateStack_.back().blendFuncState_[2] = _srcRGB;
      stateStack_.back().blendFuncState_[3] = _dstRGB;
#endif
    }
  }
}

void GLState::getBlendFuncSeparate(GLenum* _srcRGB, GLenum* _dstRGB, GLenum* _srcAlpha, GLenum* _dstAlpha)
{
  if (_srcRGB) *_srcRGB = stateStack_.back().blendFuncState_[0];
  if (_dstRGB) *_dstRGB = stateStack_.back().blendFuncState_[1];
  if (_srcAlpha) *_srcAlpha = stateStack_.back().blendFuncState_[2];
  if (_dstAlpha) *_dstAlpha = stateStack_.back().blendFuncState_[3];
}

void GLState::blendEquation(GLenum _mode)
{
  if (!blendEquationLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().blendEquationState_ != _mode)
#endif
    {
      glBlendEquation(_mode);
      stateStack_.back().blendEquationState_ = _mode;
    }
  }
}

void GLState::blendColor(GLclampf _red, GLclampf _green, GLclampf _blue, GLclampf _alpha)
{
  if (!blendColorLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().blendColorState_[0] != _red || stateStack_.back().blendColorState_[1] != _green ||
        stateStack_.back().blendColorState_[2] != _blue || stateStack_.back().blendColorState_[3] != _alpha)
#endif
    {
      glBlendColor(_red, _green, _blue, _alpha);
      stateStack_.back().blendColorState_[0] = _red;  stateStack_.back().blendColorState_[1] = _green;
      stateStack_.back().blendColorState_[2] = _blue;  stateStack_.back().blendColorState_[3] = _alpha;
    }
  }
}

void GLState::getBlendColor(GLclampf* _col)
{
  for (int i = 0; i < 4; ++i) _col[i] = stateStack_.back().blendColorState_[i];
}


void GLState::alphaFunc(GLenum _func, GLclampf _ref)
{
  if (!alphaFuncLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().alphaFuncState_ != _func || stateStack_.back().alphaRefState_ != _ref)
#endif
    {
      glAlphaFunc(_func, _ref);
      stateStack_.back().alphaFuncState_ = _func;
      stateStack_.back().alphaRefState_ = _ref;
    }
  }
}

void GLState::getAlphaFunc(GLenum* _func, GLclampf* _ref)
{
  if (_func) *_func = stateStack_.back().alphaFuncState_;
  if (_ref) *_ref = stateStack_.back().alphaRefState_;
}

void GLState::shadeModel(GLenum _mode)
{
  if (!shadeModelLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().shadeModel_ != _mode)
#endif
    {
      glShadeModel(_mode);
      stateStack_.back().shadeModel_ = _mode;
    }
  }
}

void GLState::cullFace(GLenum _mode)
{
  if (!cullFaceLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().cullFace_ != _mode)
#endif
    {
      glCullFace(_mode);
      stateStack_.back().cullFace_ = _mode;
    }
  }
}

void GLState::depthRange(GLclampd _zNear, GLclampd _zFar)
{
  if (!depthRangeLock_)
  {
 #ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (abs(_zNear - stateStack_.back().depthRange_[0]) > 1e-6 ||
      abs(_zFar - stateStack_.back().depthRange_[1]) > 1e-6)
 #endif
    {
      glDepthRange(_zNear, _zFar);
      stateStack_.back().depthRange_[0] = _zNear;
      stateStack_.back().depthRange_[1] = _zFar;
    }
  }
}

void GLState::getDepthRange(GLclampd* _zNearOut, GLclampd* _zFarOut)
{
  if (_zNearOut) *_zNearOut = stateStack_.back().depthRange_[0];
  if (_zFarOut) *_zFarOut = stateStack_.back().depthRange_[1];
}

//-----------------------------------------------------------------------------

int GLState::getBufferTargetIndex(GLenum _target)
{
  switch (_target)
  {
  case GL_ARRAY_BUFFER: return 0;
  case GL_ELEMENT_ARRAY_BUFFER: return 1;
  case GL_PIXEL_PACK_BUFFER: return 2;
  case GL_PIXEL_UNPACK_BUFFER: return 3;
#ifdef GL_ARB_uniform_buffer_object
  case GL_UNIFORM_BUFFER: return 4;
#endif
#ifdef GL_ARB_shader_storage_buffer_object
  case GL_SHADER_STORAGE_BUFFER: return 5;
#endif
#ifdef GL_ARB_shader_atomic_counters
  case GL_ATOMIC_COUNTER_BUFFER: return 6;
#endif
#ifdef GL_ARB_copy_buffer
  case GL_COPY_READ_BUFFER: return 7;
  case GL_COPY_WRITE_BUFFER: return 8;
#endif
#ifdef GL_ARB_compute_shader
  case GL_DISPATCH_INDIRECT_BUFFER: return 9;
#endif
#ifdef GL_ARB_draw_indirect
  case GL_DRAW_INDIRECT_BUFFER: return 10;
#endif
#ifdef GL_ARB_query_buffer_object
  case GL_QUERY_BUFFER: return 11;
#endif
  case GL_TEXTURE_BUFFER: return 12;
#ifdef GL_VERSION_3_0
  case GL_TRANSFORM_FEEDBACK_BUFFER: return 13;
#endif
  }
  std::cerr << "error : GLState::bindBuffer - unknown buffer target type" << _target << std::endl;
  return -1;
}

void GLState::bindBuffer(GLenum _target, GLuint _buffer)
{
  int idx = getBufferTargetIndex(_target);
  if (idx >= 0 && !glBufferTargetLock_[idx])
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().glBufferTargetState_[idx] != _buffer)
#endif
    {
      glBindBuffer(_target, _buffer);
      stateStack_.back().glBufferTargetState_[idx] = _buffer;
    }
  }
}

void GLState::lockBufferTarget(GLenum _target)
{
  glBufferTargetLock_[getBufferTargetIndex(_target)] = 1;
}

void GLState::unlockBufferTarget(GLenum _target)
{
  glBufferTargetLock_[getBufferTargetIndex(_target)] = 0;
}

bool GLState::isBufferTargetLocked(GLenum _target)
{
  return glBufferTargetLock_[getBufferTargetIndex(_target)] != 0;
}

GLuint GLState::getBoundBuf(GLenum _target)
{
  return stateStack_.back().glBufferTargetState_[getBufferTargetIndex(_target)];
}


GLenum GLState::getShaderType(GLuint _shader)
{
    auto isShader = glIsShader(_shader);
    if(isShader == GL_FALSE) { return GL_NONE; }
    GLint type = GL_NONE;
    glGetShaderiv(_shader, GL_SHADER_TYPE, &type);
    return static_cast<GLenum>(type);
}

GLuint GLState::getAttachedShader(GLuint _program, GLenum _type)
{
    if(glIsProgram(_program) == GL_FALSE) { return GL_NONE; }
    // vertex, fragment, tess control, tess eval, geometry
    static const GLsizei maxNumShaders = 5;    
    GLuint shaders[maxNumShaders] = {};
    GLsizei numShaders = 0;
    glGetAttachedShaders(_program, maxNumShaders, &numShaders, shaders);
    for(auto i = 0; i < numShaders; ++i) {
        const auto& shader = shaders[i];
        if(getShaderType(shader) == _type) {
            return shader;
        }
    }
    return GL_NONE;
}

size_t GLState::getShaderSourceLength(GLuint _shader)
{
    if(glIsShader(_shader) == GL_FALSE) { return 0; }
    GLint length = 0;
    glGetShaderiv(_shader, GL_SHADER_SOURCE_LENGTH, &length);
    return static_cast<size_t>(length);
}

size_t GLState::getShaderSource(GLuint _shader, char *_buffer, size_t _bufferSize)
{
    if(glIsShader(_shader) == GL_FALSE) { return 0; }
    GLsizei length = 0;
    glGetShaderSource(_shader, static_cast<GLsizei>(_bufferSize), &length, _buffer);
    return static_cast<size_t>(length);
}

//-----------------------------------------------------------------------------

void GLState::activeTexture(GLenum _texunit)
{
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
  if (stateStack_.back().activeTexture_ != _texunit)
#endif
  {
    glActiveTexture(_texunit);
    stateStack_.back().activeTexture_ = _texunit;
  }
}

void GLState::bindTexture(GLenum _target, GLuint _buffer)
{
  int activeTex = getActiveTextureIndex();

  assert(activeTex >= 0);

  GLStateContext::TextureStage* stage = stateStack_.back().glTextureStage_ + activeTex;

  if (!glTextureStageLock_[activeTex])
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (_buffer != stage->buf_ || _target != stage->target_)
#endif
    {
      glBindTexture(_target, _buffer);

      stage->target_ = _target;
      stage->buf_ = _buffer;
    }
  }
}

void GLState::lockTextureStage()
{
  glTextureStageLock_[getActiveTextureIndex()] = 1;
}

void GLState::unlockTextureStage()
{
  glTextureStageLock_[getActiveTextureIndex()] = 0;
}

bool GLState::isTextureTargetLocked()
{
  return glTextureStageLock_[getActiveTextureIndex()] != 0;
}

GLuint GLState::getBoundTextureBuffer()
{
  return stateStack_.back().glTextureStage_[getActiveTextureIndex()].buf_;
}

GLenum GLState::getBoundTextureTarget()
{
  return stateStack_.back().glTextureStage_[getActiveTextureIndex()].target_;
}

//----------------------------------------------------------
// vertex pointers

void GLState::vertexPointer(GLint _size, GLenum _type, GLsizei _stride, const GLvoid* _pointer)
{
  if (!vertexPointerLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (!stateStack_.back().vertexPointer_.equals(_size, _type, _stride, _pointer))
#endif
    {
      glVertexPointer(_size, _type, _stride, _pointer);
      stateStack_.back().vertexPointer_.set(_size, _type, _stride, _pointer);
    }
  }
}

void GLState::getVertexPointer(GLint* _size, GLenum* _type, GLsizei* _stride, const GLvoid** _pointer)
{
  if (_size) *_size = stateStack_.back().vertexPointer_.size;
  if (_stride) *_stride = stateStack_.back().vertexPointer_.stride;
  if (_type) *_type = stateStack_.back().vertexPointer_.type;
  if (_pointer) *_pointer = stateStack_.back().vertexPointer_.pointer;
}

void GLState::normalPointer(GLenum _type, GLsizei _stride, const GLvoid* _pointer)
{
  if (!normalPointerLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (!stateStack_.back().normalPointer_.equals(stateStack_.back().normalPointer_.size, _type, _stride, _pointer))
#endif
    {
      glNormalPointer(_type, _stride, _pointer);
      stateStack_.back().normalPointer_.set(3, _type, _stride, _pointer);
    }
  }
}

void GLState::getNormalPointer(GLenum* _type, GLsizei* _stride, const GLvoid** _pointer)
{
  if (_type) *_type = stateStack_.back().normalPointer_.type;
  if (_stride) *_stride = stateStack_.back().normalPointer_.stride;
  if (_pointer) *_pointer = stateStack_.back().normalPointer_.pointer;
}


void GLState::colorPointer(GLint _size, GLenum _type, GLsizei _stride, const GLvoid* _pointer)
{
  if (!colorPointerLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (!stateStack_.back().colorPointer_.equals(_size, _type, _stride, _pointer))
#endif
    {
      glColorPointer(_size, _type, _stride, _pointer);
      stateStack_.back().colorPointer_.set(_size, _type, _stride, _pointer);
    }
  }
}

void GLState::getColorPointer(GLint* _size, GLenum* _type, GLsizei* _stride, const GLvoid** _pointer)
{
  if (_size) *_size = stateStack_.back().colorPointer_.size;
  if (_stride) *_stride = stateStack_.back().colorPointer_.stride;
  if (_type) *_type = stateStack_.back().colorPointer_.type;
  if (_pointer) *_pointer = stateStack_.back().colorPointer_.pointer;
}

void GLState::texcoordPointer(GLint _size, GLenum _type, GLsizei _stride, const GLvoid* _pointer)
{
  if (!texcoordPointerLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (!stateStack_.back().texcoordPointer_.equals(_size, _type, _stride, _pointer))
#endif
    {
      glTexCoordPointer(_size, _type, _stride, _pointer);
      stateStack_.back().texcoordPointer_.set(_size, _type, _stride, _pointer);
    }
  }
}

void GLState::getTexcoordPointer(GLint* _size, GLenum* _type, GLsizei* _stride, const GLvoid** _pointer)
{
  if (_size) *_size = stateStack_.back().texcoordPointer_.size;
  if (_stride) *_stride = stateStack_.back().texcoordPointer_.stride;
  if (_type) *_type = stateStack_.back().texcoordPointer_.type;
  if (_pointer) *_pointer = stateStack_.back().texcoordPointer_.pointer;
}

void GLState::setTexGenMode(GLenum _coord, GLenum _name, GLint _param)
{
    if (ACG::compatibilityProfile())
    {
        glTexGeni(_coord, _name, _param);
    }
    else
    {
        stateStack_.back().texGenMode_ = _param;
    }
}

void GLState::getTexGenMode(GLenum _coord, GLenum _name, GLint* _param)
{
    if (ACG::compatibilityProfile())
    {
        glGetTexGeniv(_coord, _name, _param);
    }
    else
    {
        *_param = stateStack_.back().texGenMode_;
    }
}

//---------------------------------------------------------------------
// draw buffer functions

void GLState::drawBuffer(GLenum _mode)
{
  if (!drawBufferLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().drawBufferSingle_ != _mode || stateStack_.back().activeDrawBuffer_)
#endif
    {
      glDrawBuffer(_mode);
      stateStack_.back().drawBufferSingle_ = _mode;
      stateStack_.back().activeDrawBuffer_ = 0;
    }
  }
}

void GLState::drawBuffers(GLsizei _n, const GLenum* _bufs)
{
  if (!drawBufferLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    int bChange = !stateStack_.back().activeDrawBuffer_;
    for (int i = 0; i < _n && (!bChange); ++i)
    {
      if (stateStack_.back().drawBufferState_[i] != _bufs[i])
        bChange = 1;
    }

    if (bChange)
#endif
    {
      glDrawBuffers(_n, _bufs);

      for (int i = 0; i < _n; ++i)
        stateStack_.back().drawBufferState_[i] = _bufs[i];

      stateStack_.back().activeDrawBuffer_ = _n;
    }
  }
}

/// get current framebuffer of a target
GLuint GLState::getFramebufferDraw()
{
  GLint curfbo;
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, (GLint*)&curfbo);
  return curfbo;
}

/// get current framebuffer of a target
GLuint GLState::getFramebufferRead()
{
  GLint curfbo;
  glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, (GLint*)&curfbo);
  return curfbo;
}

void GLState::bindFramebuffer(GLenum _target, GLuint _framebuffer)
{
  int i = -1;
  switch (_target)
  {
  case GL_FRAMEBUFFER:
    {
      bindFramebuffer(GL_READ_FRAMEBUFFER, _framebuffer);
      bindFramebuffer(GL_DRAW_FRAMEBUFFER, _framebuffer);
      return;
    }
  case GL_DRAW_FRAMEBUFFER: i = 0; break;
  case GL_READ_FRAMEBUFFER: i = 1; break;
  }

  if (i >= 0 && !framebufferLock_[i])
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().framebuffers_[i] != _framebuffer)
#endif
    {
#ifndef __APPLE__
        if (glBindFramebuffer == nullptr)
         glewInit();
#endif
        glBindFramebuffer(_target, _framebuffer);
      stateStack_.back().framebuffers_[i] = _framebuffer;
    }
  }
}

void GLState::lockFramebuffer(GLenum _target)
{
  switch (_target)
  {
  case GL_FRAMEBUFFER:
    framebufferLock_[0] = framebufferLock_[1] = true; break;

  case GL_DRAW_FRAMEBUFFER: framebufferLock_[0] = true; break;
  case GL_READ_FRAMEBUFFER: framebufferLock_[1] = true; break;
  }
}

void GLState::unlockFramebuffer(GLenum _target)
{
  switch (_target)
  {
  case GL_FRAMEBUFFER:
    framebufferLock_[0] = framebufferLock_[1] = false; break;

  case GL_DRAW_FRAMEBUFFER: framebufferLock_[0] = false; break;
  case GL_READ_FRAMEBUFFER: framebufferLock_[1] = false; break;
  }
}

bool GLState::isFramebufferLocked(GLenum _target)
{
  switch (_target)
  {
  case GL_FRAMEBUFFER:
    return framebufferLock_[0] && framebufferLock_[1];

  case GL_DRAW_FRAMEBUFFER: return framebufferLock_[0];
  case GL_READ_FRAMEBUFFER: return framebufferLock_[1];
  }
  return false;
}

void GLState::useProgram(GLuint _program)
{
  if (!programLock_)
  {
#ifdef GLSTATE_AVOID_REDUNDANT_GLCALLS
    if (stateStack_.back().program_ != _program)
#endif
    {
      glUseProgram(_program);
      stateStack_.back().program_ = _program;
    }
  }
}

void GLState::genBuffersARB(GLsizei n, GLuint* buffers) {
    glGenBuffers(n, buffers);
}

void GLState::genBuffers(GLsizei n, GLuint* buffers) {
    glGenBuffers(n, buffers);
}

void GLState::bufferDataARB(
        GLenum target, GLsizeiptrARB size, const GLvoid *data, GLenum usage) {
    glBufferData(target, size, data, usage);
}

void GLState::bufferData(
        GLenum target, GLsizeiptr size, const GLvoid* data, GLenum usage) {
    glBufferData(target, size, data, usage);
}

void GLState::deleteBuffers(GLsizei n, const GLuint* buffers) {
    glDeleteBuffers(n, buffers);
}

GLvoid* GLState::mapBuffer (GLenum target, GLenum access) {
    return glMapBuffer(target, access);
}


GLboolean GLState::unmapBuffer (GLenum target) {
    return glUnmapBuffer(target);
}

size_t GLState::getBufferSize(GLenum target) {
    GLint size = 0;
    glGetBufferParameteriv(target, GL_BUFFER_SIZE, &size);
    return static_cast<size_t>(size);
}

//---------------------------------------------------------------------




//=============================================================================
} // namespace ACG
//=============================================================================

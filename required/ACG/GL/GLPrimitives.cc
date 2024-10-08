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


#include <ACG/GL/acg_glew.hh>

#include "GLPrimitives.hh"
#include <ACG/GL/IRenderer.hh>

namespace ACG {

//========================================================================
// GLPrimitive base class
//========================================================================

GLPrimitive::GLPrimitive() :
        vboDataInvalid_(true),
        normalOrientation_(OUTSIDE),
        numTris_(0),
        numLines_(0),
        vboData_(0),
        curTriPtr_(0),
        vbo_(0)
{

  vertexDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
  vertexDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_NORMAL);
  vertexDecl_.addElement(GL_FLOAT, 2, VERTEX_USAGE_TEXCOORD);
}

//------------------------------------------------------------------------

GLPrimitive::~GLPrimitive()
{
  if (vbo_)
    glDeleteBuffers(1, &vbo_);

  delete[] vboData_;
}

//------------------------------------------------------------------------

void GLPrimitive::addTriangleToVBO(const ACG::Vec3f* _p, const ACG::Vec3f* _n, const ACG::Vec2f* _tex)
{
  if (!numTris_ || vboDataInvalid_)
    numTris_ = getNumTriangles();

  if (!numTris_)
    return;

  assert(numLines_ == 0);


  if (!vboData_)
    vboData_ = new float[8 * 3 * numTris_];

  if (curTriPtr_ == numTris_)
    return;

  float* pTri = &vboData_[0] + (curTriPtr_++) * 3 * 8;

  // copy triangle
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < 3; ++k)
      *(pTri++) = _p[i][k];

    for (int k = 0; k < 3; ++k)
      *(pTri++) = _n[i][k];

    for (int k = 0; k < 2; ++k)
      *(pTri++) = _tex[i][k];
  }
}


void GLPrimitive::addLineToVBO( const ACG::Vec3f* _p, const ACG::Vec3f* _n, const ACG::Vec2f* _tex )
{
  if (!numLines_ || vboDataInvalid_)
    numLines_ = getNumLines();

  if (!numLines_)
    return;

  assert(numTris_ == 0);

  if (!vboData_)
    vboData_ = new float[8 * 2 * numLines_];

  if (curTriPtr_ == numLines_)
    return;

  float* pLine = &vboData_[0] + (curTriPtr_++) * 2 * 8;

  // copy line segment
  for (int i = 0; i < 2; ++i) {
    for (int k = 0; k < 3; ++k)
      *(pLine++) = _p[i][k];

    for (int k = 0; k < 3; ++k)
      *(pLine++) = _n[i][k];

    for (int k = 0; k < 2; ++k)
      *(pLine++) = _tex[i][k];
  }
}


//------------------------------------------------------------------------

void GLPrimitive::bindVBO()
{
  if (checkVBO())
  {
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    glVertexPointer(3, GL_FLOAT, 32, 0);
    if(ACG::compatibilityProfile())
    glEnableClientState(GL_VERTEX_ARRAY);

    glNormalPointer(GL_FLOAT, 32, (GLvoid*) 12);
    if(ACG::compatibilityProfile())
    glEnableClientState(GL_NORMAL_ARRAY);

    glClientActiveTexture(GL_TEXTURE0);
    glTexCoordPointer(2, GL_FLOAT, 32, (GLvoid*) 24);
    if(ACG::compatibilityProfile())
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  }
}

//------------------------------------------------------------------------


bool GLPrimitive::checkVBO()
{
  // create vbo if not done yet
  // update vbo data and upload to gpu if needed
  // return false iff vbo empty

  const int bufSize = numTris_ ? numTris_ * 3 * 8 * 4 : numLines_ * 2 * 8 * 4;

  if (!vbo_) {
    if (!vboData_ || (!numTris_ && !numLines_) || (numTris_ && numLines_))
      return false;

    // create vbo
    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, bufSize, vboData_, GL_STATIC_DRAW);

    delete[] vboData_;
    vboData_ = 0;
  }

  if (vboDataInvalid_) {
    updateVBOData();
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, bufSize, vboData_, GL_STATIC_DRAW);
    vboDataInvalid_ = false;
  }

  return true;
}

//------------------------------------------------------------------------

void GLPrimitive::unBindVBO()
{
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  if(ACG::compatibilityProfile())
  {
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  }
}

//------------------------------------------------------------------------

void GLPrimitive::draw_primitive()
{
  bindVBO();

  if (numTris_)
    glDrawArrays(GL_TRIANGLES, 0, numTris_ * 3);
  else
    glDrawArrays(GL_LINES, 0, numLines_ * 2);

  unBindVBO();
}

//------------------------------------------------------------------------

void GLPrimitive::draw_primitive(GLSL::Program* _program)
{
  if (checkVBO())
  {
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    vertexDecl_.activateShaderPipeline(_program);

    if (numTris_)
      glDrawArrays(GL_TRIANGLES, 0, numTris_ * 3);
    else
      glDrawArrays(GL_LINES, 0, numLines_ * 2);

    vertexDecl_.deactivateShaderPipeline(_program);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }
}

//------------------------------------------------------------------------

void GLPrimitive::addToRenderer_primitive( class IRenderer* _renderer, RenderObject* _ro )
{
  if (checkVBO())
  {
    _ro->vertexBuffer = vbo_;
    _ro->vertexDecl = &vertexDecl_;

    if (numTris_)
      _ro->glDrawArrays(GL_TRIANGLES, 0, numTris_ * 3);
    else
      _ro->glDrawArrays(GL_LINES, 0, numLines_ * 2);

    _renderer->addRenderObject(_ro);
  }
}

//------------------------------------------------------------------------

void GLPrimitive::updateVBOData() {
  curTriPtr_ = 0;

  if (vboData_) {
    delete[] vboData_;
    vboData_ = 0;
  }

  updateVBO();
}

//------------------------------------------------------------------------

unsigned int GLPrimitive::getVBO()
{
  return checkVBO() ? vbo_ : 0;
}

const VertexDeclaration* GLPrimitive::getVertexDecl() const
{
  return &vertexDecl_;
}

//========================================================================
// GLSphere
//========================================================================



GLSphere::GLSphere(int _slices, int _stacks) :
        slices_(_slices),
        stacks_(_stacks)
{
  updateVBO();
}

//------------------------------------------------------------------------

GLSphere::~GLSphere()
{

}

//------------------------------------------------------------------------

void GLSphere::draw(GLState& _state, float _radius, const ACG::Vec3f& _center)
{
  _state.push_modelview_matrix();

  _state.translate(ACG::Vec3d(_center));
  _state.scale(_radius, _radius, _radius);

  GLPrimitive::draw_primitive();

  _state.pop_modelview_matrix();
}

//------------------------------------------------------------------------

void GLSphere::addToRenderer( IRenderer* _renderer, const RenderObject* _base, float _radius, const ACG::Vec3f& _center /*= ACG::Vec3f(0.0f, 0.0f, 0.0f)*/ )
{
  RenderObject ro = *_base;

  ro.modelview.translate(ACG::Vec3d(_center));
  ro.modelview.scale((double)_radius, (double)_radius, (double)_radius);

  GLPrimitive::addToRenderer_primitive(_renderer, &ro);
}


//------------------------------------------------------------------------

int GLSphere::getNumTriangles()
{
  return 2 * slices_ + (stacks_ - 2) * slices_ * 2;
}

//------------------------------------------------------------------------

void GLSphere::updateVBO()
{
  for (int sl = 0; sl < slices_; ++sl) {
    // top triangle:
    {
      int st = 0;
      addTriangle(0, st, sl + 1, st + 1, sl, st + 1);
    }
    // middle quads:
    for (int st = 1; st < stacks_ - 1; ++st) {
      addTriangle(sl, st, sl + 1, st, sl, st + 1);
      addTriangle(sl + 1, st, sl + 1, st + 1, sl, st + 1);
    }
    // bottom triangle:
    {
      addTriangle(0, stacks_, sl, stacks_ - 1, sl + 1, stacks_ - 1);
    }
  }
}

//------------------------------------------------------------------------

void GLSphere::addTriangle(int sl0, int st0, int sl1, int st1, int sl2, int st2)
{
  ACG::Vec3f p[3];
  ACG::Vec3f n[3];
  ACG::Vec2f tex[3];

  n[0] = p[0] = positionOnSphere(sl0, st0);
  n[1] = p[1] = positionOnSphere(sl1, st1);
  n[2] = p[2] = positionOnSphere(sl2, st2);
  n[0].normalize();
  n[1].normalize();
  n[2].normalize();
  tex[0] = texCoordOnSphere(sl0, st0);
  tex[1] = texCoordOnSphere(sl1, st1);
  tex[2] = texCoordOnSphere(sl2, st2);

  addTriangleToVBO(p, n, tex);
}

//------------------------------------------------------------------------

ACG::Vec3f GLSphere::positionOnSphere(int _sliceNumber, int _stackNumber)
{
  ACG::Vec3f position;

  double alpha = (M_PI / double(stacks_)) * double(_stackNumber);
  double beta = ((2.0 * M_PI) / double(slices_)) * double(_sliceNumber);

  double ringRadius = sin(alpha);
  position[0] = sin(beta) * ringRadius;
  position[1] = cos(beta) * ringRadius;
  position[2] = cos(alpha);

  return position;
}

//------------------------------------------------------------------------

ACG::Vec2f GLSphere::texCoordOnSphere(int _sliceNumber, int _stackNumber)
{
  ACG::Vec2f texCoord;

  double alpha = (M_PI / double(stacks_)) * double(_stackNumber);
  texCoord[0] = double(_sliceNumber) / double(slices_);
  texCoord[1] = 0.5 * (cos(alpha) + 1.0);

  return texCoord;
}

//========================================================================
// GLCone
//========================================================================

GLCone::GLCone(int _slices, int _stacks, float _bottomRadius, float _topRadius, bool _bottomCap, bool _topCap) :
        slices_(_slices),
        stacks_(_stacks),
        bottomRadius_(_bottomRadius),
        topRadius_(_topRadius),
        bottomCap_(_bottomCap),
        topCap_(_topCap)
{
  updateVBO();
}

//------------------------------------------------------------------------

GLCone::~GLCone()
{

}

//------------------------------------------------------------------------

void GLCone::setBottomRadius(float _bottomRadius) {
  if (bottomRadius_ != _bottomRadius)
    vboDataInvalid_ = true;
  bottomRadius_ = _bottomRadius;
}

//------------------------------------------------------------------------

void GLCone::setTopRadius(float _topRadius) {
  if (topRadius_ != _topRadius)
    vboDataInvalid_ = true;
  topRadius_ = _topRadius;
}

//------------------------------------------------------------------------

void GLCone::setNormalOrientation(NormalOrientation orientation) {
  if (normalOrientation_ != orientation)
    vboDataInvalid_ = true;
  normalOrientation_ = orientation;
}

//------------------------------------------------------------------------

void GLCone::draw(GLState& _state, float _height, const ACG::Vec3f& _center, ACG::Vec3f _upDir)
{
//  ACG::mat4 mWorld = ACG::translate(ACG::mat4(1.0f), _center);

  _state.push_modelview_matrix();

  // translate
  _state.translate(ACG::Vec3d(_center));

  _upDir.normalize();

  // compute rotation matrix mAlign
  //  such that vBindDir rotates to _upDir
  ACG::GLMatrixd mAlign;
  mAlign.identity();

  ACG::Vec3f vBindDir(0.0f, 0.0f, 1.0f);

  ACG::Vec3f vRotAxis = OpenMesh::cross(_upDir, vBindDir);
  vRotAxis.normalize();

  ACG::Vec3f vUp = OpenMesh::cross(_upDir, vRotAxis);

  // rotate
  for (int i = 0; i < 3; ++i) {
    mAlign(i, 0) = vRotAxis[i];
    mAlign(i, 1) = vUp[i];
    mAlign(i, 2) = _upDir[i];
  }

  ACG::Vec3f vDelta = vBindDir - _upDir;
  if ( fabsf(OpenMesh::dot(vDelta, vDelta)) < 1e-3f)
    mAlign.identity();

  // scale
  mAlign.scale(1.0, 1.0, _height);

  ACG::GLMatrixd mAlignInv(mAlign);
  mAlignInv.invert();

  _state.mult_matrix(mAlign, mAlignInv);

  GLPrimitive::draw_primitive();

  _state.pop_modelview_matrix();
}

//------------------------------------------------------------------------


void GLCone::addToRenderer(IRenderer* _renderer,
                           const RenderObject* _base,
                           float _height,
                           const ACG::Vec3f& _center, 
                           ACG::Vec3f _upDir,
                           float _radiusScale)
{
  RenderObject ro = *_base;

  // translate
  ro.modelview.translate(ACG::Vec3d(_center));

  _upDir.normalize();

  // compute rotation matrix mAlign
  //  such that vBindDir rotates to _upDir
  ACG::GLMatrixf mAlign;
  mAlign.identity();

  ACG::Vec3f vBindDir(0.0f, 0.0f, 1.0f);

  ACG::Vec3f vRotAxis = OpenMesh::cross(_upDir, vBindDir);
  vRotAxis.normalize();

  ACG::Vec3f vUp = OpenMesh::cross(_upDir, vRotAxis);

  // rotate
  for (int i = 0; i < 3; ++i) {
    mAlign(i, 0) = vRotAxis[i];
    mAlign(i, 1) = vUp[i];
    mAlign(i, 2) = _upDir[i];
  }

  ACG::Vec3f vDelta = vBindDir - _upDir;
  if ( fabsf(OpenMesh::dot(vDelta, vDelta)) < 1e-3f)
    mAlign.identity();

  // scale
  mAlign.scale(_radiusScale, _radiusScale, _height);

  ro.modelview *= mAlign;

  GLPrimitive::addToRenderer_primitive(_renderer, &ro);
}

//------------------------------------------------------------------------

int GLCone::getNumTriangles()
{
  int numTris = stacks_ * slices_ * 2;

  if (bottomCap_)
    numTris += slices_;
  if (topCap_)
    numTris += slices_;

  return numTris;
}

//------------------------------------------------------------------------

ACG::Vec3f GLCone::positionOnCone(int _sliceNumber, int _stackNumber)
{
  ACG::Vec3f position;

  double beta = ((2.0 * M_PI) / slices_) * _sliceNumber;

  double relativeHeight = _stackNumber / (double) stacks_;
  double ringRadius = (1.0 - relativeHeight) * bottomRadius_ + relativeHeight * topRadius_;
  position[0] = sin(beta) * ringRadius;
  position[1] = cos(beta) * ringRadius;
  position[2] = relativeHeight;

  return position;
}

//------------------------------------------------------------------------

ACG::Vec2f GLCone::texCoordOnCone(int _sliceNumber, int _stackNumber)
{
  ACG::Vec2f texCoord;

  texCoord[0] = _sliceNumber / (double) slices_;
  texCoord[1] = _stackNumber / (double) stacks_;

  return texCoord;
}

//------------------------------------------------------------------------

ACG::Vec3f GLCone::normalOnCone(int _sliceNumber, int _stackNumber)
{
  ACG::Vec3f normal;

  double beta = ((2.0 * M_PI) / slices_) * _sliceNumber;
  // double relativeHeight = _stackNumber / (double) stacks_;

  normal[0] = sin(beta);
  normal[1] = cos(beta);
  normal[2] = (bottomRadius_ - topRadius_);

  normal.normalize();
  return normal;
}

//------------------------------------------------------------------------

void GLCone::addTriangle(int sl0, int st0, int sl1, int st1, int sl2, int st2)
{
  ACG::Vec3f p[3];
  ACG::Vec3f n[3];
  ACG::Vec2f tex[3];

  p[0] = positionOnCone(sl0, st0);
  p[1] = positionOnCone(sl1, st1);
  p[2] = positionOnCone(sl2, st2);
  if (normalOrientation_ == OUTSIDE) {
    n[0] = normalOnCone(sl0, st0);
    n[1] = normalOnCone(sl1, st1);
    n[2] = normalOnCone(sl2, st2);
  } else if (normalOrientation_ == INSIDE) {
    n[0] = -normalOnCone(sl0, st0);
    n[1] = -normalOnCone(sl1, st1);
    n[2] = -normalOnCone(sl2, st2);
  }
  tex[0] = texCoordOnCone(sl0, st0);
  tex[1] = texCoordOnCone(sl1, st1);
  tex[2] = texCoordOnCone(sl2, st2);

  addTriangleToVBO(p, n, tex);
}

//------------------------------------------------------------------------

void GLCone::updateVBO()
{
  for (int sl = 0; sl < slices_; ++sl) {
    // top triangle:
    if (topCap_) {
      ACG::Vec3f p[3];
      ACG::Vec3f n[3];
      ACG::Vec2f tex[3];

      p[0] = ACG::Vec3f(0.0, 0.0, 1.0);
      p[1] = positionOnCone(sl + 1, stacks_);
      p[2] = positionOnCone(sl, stacks_);
      if (normalOrientation_ == OUTSIDE) {
        n[0] = ACG::Vec3f(0.0, 0.0, 1.0);
        n[1] = ACG::Vec3f(0.0, 0.0, 1.0);
        n[2] = ACG::Vec3f(0.0, 0.0, 1.0);
      } else if (normalOrientation_ == INSIDE) {
        n[0] = ACG::Vec3f(0.0, 0.0, -1.0);
        n[1] = ACG::Vec3f(0.0, 0.0, -1.0);
        n[2] = ACG::Vec3f(0.0, 0.0, -1.0);
      }

      tex[0] = ACG::Vec2f(0.5, 0.5);
      double beta = ((2.0 * M_PI) / slices_) * (sl + 1);
      tex[1] = ACG::Vec2f(sin(beta), cos(beta));
      beta = ((2.0 * M_PI) / slices_) * (sl);
      tex[2] = ACG::Vec2f(sin(beta), cos(beta));

      addTriangleToVBO(p, n, tex);
    }
    // middle quads:
    for (int st = 0; st < stacks_; ++st) {
      addTriangle(sl, st, sl, st + 1, sl + 1, st);
      addTriangle(sl + 1, st, sl, st + 1, sl + 1, st + 1);
    }
    // bottom triangle:
    if (bottomCap_) {
      ACG::Vec3f p[3];
      ACG::Vec3f n[3];
      ACG::Vec2f tex[3];

      p[0] = ACG::Vec3f(0.0, 0.0, 0.0);
      p[1] = positionOnCone(sl, 0);
      p[2] = positionOnCone(sl + 1, 0);
      if (normalOrientation_ == OUTSIDE) {
        n[0] = ACG::Vec3f(0.0, 0.0, -1.0);
        n[1] = ACG::Vec3f(0.0, 0.0, -1.0);
        n[2] = ACG::Vec3f(0.0, 0.0, -1.0);
      } else if (normalOrientation_ == INSIDE) {
        n[0] = ACG::Vec3f(0.0, 0.0, 1.0);
        n[1] = ACG::Vec3f(0.0, 0.0, 1.0);
        n[2] = ACG::Vec3f(0.0, 0.0, 1.0);
      }

      tex[0] = ACG::Vec2f(0.5, 0.5);
      double beta = ((2.0 * M_PI) / slices_) * (sl);
      tex[1] = ACG::Vec2f(sin(beta), cos(beta));
      beta = ((2.0 * M_PI) / slices_) * (sl + 1);
      tex[2] = ACG::Vec2f(sin(beta), cos(beta));

      addTriangleToVBO(p, n, tex);
    }
  }
}

//========================================================================
// GLCylinder
//========================================================================

GLCylinder::GLCylinder(int _slices, int _stacks, float _radius, bool _bottomCap, bool _topCap) :
        GLCone(_slices, _stacks, _radius, _radius, _bottomCap, _topCap)
{
}

//========================================================================
// GLPartialDisk
//========================================================================

/**
 * @param[in] _slices the number of slices that subdivide the partial disk has to be at least 2
 * @param[in] _loops the number of loops that subdivide the slices has to be at least 1
 * @param[in] _innerRadius the inner radius should not be smaller than zero and greater than the outer radius
 * @param[in] _outerRadius the outer radius should not be zero or smaller than zero
 * @param[in] _startAngle the angle at which the partial disk starts
 * @param[in] _sweepAngle the angle at which the partial disk ends
 */
GLPartialDisk::GLPartialDisk(int _slices, int _loops, float _innerRadius, float _outerRadius, float _startAngle, float _sweepAngle) :
    slices_(_slices),
    loops_(_loops),
    innerRadius_(_innerRadius),
    outerRadius_(_outerRadius),
    startAngle_(_startAngle),
    sweepAngle_(_sweepAngle)
{
  updateVBO();
}

//------------------------------------------------------------------------

void GLPartialDisk::setInnerRadius(float _innerRadius) {
  if (innerRadius_ != _innerRadius)
    vboDataInvalid_ = true;
  innerRadius_ = _innerRadius;
}

//------------------------------------------------------------------------

void GLPartialDisk::setOuterRadius(float _outerRadius) {
  if (outerRadius_ != _outerRadius)
    vboDataInvalid_ = true;
  outerRadius_ = _outerRadius;
}

//------------------------------------------------------------------------

int GLPartialDisk::getNumTriangles() {
  return slices_ * (loops_+1) * 2;
}

//------------------------------------------------------------------------

void GLPartialDisk::updateVBO() {
  assert(slices_ >= 2);
  assert(loops_ >= 1);
  assert(outerRadius_ > 0.0f);
  assert(innerRadius_ >= 0.0f);
  assert(innerRadius_ < outerRadius_);

  if (sweepAngle_ < -360.0f)
    sweepAngle_ = 360.0f;
  if (sweepAngle_ > 360.0f)
    sweepAngle_ = 360.0f;
  if (sweepAngle_ < 0) {
    startAngle_ += sweepAngle_;
    sweepAngle_ = -sweepAngle_;
  }

  float* sinCache = new float[slices_+1];
  float* cosCache = new float[slices_+1];

  // precompute all sine and cosine that are needed
  float angleOffsetRadian = startAngle_ * M_PI / 180.0f;
  float sweepAngleRadian = sweepAngle_ * M_PI / 180.0f;
  for (int i = 0; i < slices_+1; ++i) {
    float angle = angleOffsetRadian + sweepAngleRadian * i/slices_;
    sinCache[i] = sin(angle);
    cosCache[i] = cos(angle);
  }

  // iterate over loops (starting from the inner most) to generate triangles
  float deltaRadius = outerRadius_ - innerRadius_;
  for (int i = loops_+1; i > 0; --i) {

    // for each slice generate two triangles
    for (int j = 0; j < slices_; ++j) {

      ACG::Vec3f p[3];
      ACG::Vec3f n[3];
      ACG::Vec2f tex[3];
      ACG::Vec3f p2[3];
      ACG::Vec3f n2[3];
      ACG::Vec2f tex2[3];

      // radius of the loop nearer to the center of the disk
      float innerRadius = outerRadius_ - deltaRadius * ((float) i / (loops_ + 1));
      // radius of the loop further from the center of the disk
      float outerRadius = outerRadius_ - deltaRadius * ((float) (i - 1) / (loops_ + 1));

      // first triangle:
      // 1        2
      //
      // 0
      // vertices
      p[0] = ACG::Vec3f(innerRadius * sinCache[j], innerRadius * cosCache[j], 0.0f);
      p[1] = ACG::Vec3f(outerRadius * sinCache[j], outerRadius * cosCache[j], 0.0f);
      p[2] = ACG::Vec3f(outerRadius * sinCache[j+1], outerRadius * cosCache[j+1], 0.0f);
      // normals
      n[0] = ACG::Vec3f(0.0f, 0.0f, 1.0f);
      n[1] = ACG::Vec3f(0.0f, 0.0f, 1.0f);
      n[2] = ACG::Vec3f(0.0f, 0.0f, 1.0f);
      // TODO: proper texture coordinates
      tex[0] = ACG::Vec2f(0.0f, 0.0f);
      tex[1] = ACG::Vec2f(0.0f, 0.0f);
      tex[2] = ACG::Vec2f(0.0f, 0.0f);

      addTriangleToVBO(p, n, tex);

      // second triangle:
      // x        1
      //
      // 0        2
      // vertices
      p2[0] = ACG::Vec3f(innerRadius * sinCache[j], innerRadius * cosCache[j], 0.0f);
      p2[1] = ACG::Vec3f(outerRadius * sinCache[j+1], outerRadius * cosCache[j+1], 0.0f);
      p2[2] = ACG::Vec3f(innerRadius * sinCache[j+1], innerRadius * cosCache[j+1], 0.0f);
      // normals
      n2[0] = ACG::Vec3f(0.0f, 0.0f, 1.0f);
      n2[1] = ACG::Vec3f(0.0f, 0.0f, 1.0f);
      n2[2] = ACG::Vec3f(0.0f, 0.0f, 1.0f);
      // TODO: proper texture coordinates
      tex2[0] = ACG::Vec2f(0.0f, 0.0f);
      tex2[1] = ACG::Vec2f(0.0f, 0.0f);
      tex2[2] = ACG::Vec2f(0.0f, 0.0f);

      addTriangleToVBO(p2, n2, tex2);

    }
  }

  delete[] sinCache;
  delete[] cosCache;
}

//------------------------------------------------------------------------

void GLPartialDisk::draw( GLState& _state, const ACG::Vec3f& _center, ACG::Vec3f _upDir) {
  _state.push_modelview_matrix();

  // translate
  _state.translate(ACG::Vec3d(_center));

  _upDir.normalize();

  // compute rotation matrix mAlign
  //  such that vBindDir rotates to _upDir
  ACG::GLMatrixd mAlign;
  mAlign.identity();

  ACG::Vec3f vBindDir(0.0f, 0.0f, 1.0f);

  ACG::Vec3f vRotAxis = OpenMesh::cross(_upDir, vBindDir);
  vRotAxis.normalize();

  ACG::Vec3f vUp = OpenMesh::cross(_upDir, vRotAxis);

  // rotate
  for (int i = 0; i < 3; ++i) {
    mAlign(i, 0) = vRotAxis[i];
    mAlign(i, 1) = vUp[i];
    mAlign(i, 2) = _upDir[i];
  }

  ACG::Vec3f vDelta = vBindDir - _upDir;
  if ( fabsf(OpenMesh::dot(vDelta, vDelta)) < 1e-3f)
    mAlign.identity();

  ACG::GLMatrixd mAlignInv(mAlign);
  mAlignInv.invert();

  _state.mult_matrix(mAlign, mAlignInv);

  GLPrimitive::draw_primitive();

  _state.pop_modelview_matrix();
}

//========================================================================
// GLDisk
//========================================================================

/**
 * @param[in] _slices the number of slices that subdivide the disk has to be at least 2
 * @param[in] _loops the number of loops that subdivide the slices has to be at least 1
 * @param[in] _innerRadius the inner radius should not be smaller than zero and greater than the outer radius
 * @param[in] _outerRadius the outer radius should not be zero or smaller than zero
 */
GLDisk::GLDisk(int _slices, int _loops, float _innerRadius, float _outerRadius) :
  GLPartialDisk(_slices, _loops, _innerRadius, _outerRadius, 0.0f, 360.0f)
{
}


//========================================================================
// GLBox
//========================================================================

GLBox::GLBox()
{
  updateVBO();
}

GLBox::~GLBox()
{
}

int GLBox::getNumTriangles()
{
  return 12;
}

//------------------------------------------------------------------------

void GLBox::updateVBO()
{
  static const Vec3f pos[8] =
  {
    Vec3f(-0.5f,-0.5f,0.5f), Vec3f(-0.5f,-0.5f,-0.5f), Vec3f(0.5f,-0.5f,-0.5f),
    Vec3f(0.5f,-0.5f,0.5f), Vec3f(-0.5f,0.5f,0.5f), Vec3f(0.5f,0.5f,0.5f),
    Vec3f(0.5f,0.5f,-0.5f), Vec3f(-0.5f,0.5f,-0.5f)
  };

  static const Vec3f norm[6] = 
  {
    Vec3f(0.0f,-1.0f,0.0f), Vec3f(0.0f,1.0f,0.0f), Vec3f(0.0f,0.0f,1.0f),
    Vec3f(1.0f,0.0f,0.0f), Vec3f(0.0f,0.0f,-1.0f), Vec3f(-1.0f,0.0f,0.0f)
  };

  static const Vec2f texc[4] = 
  {
    Vec2f(1.0f,0.0f), Vec2f(1.0f,1.0f), Vec2f(0.0f,1.0f), Vec2f(0.0f,0.0f)
  };

  // tri: p,p,p  ,n,n,n ,t,t,t
  static const int tris[12][9] = 
  {
    {0,1,2 ,0,0,0 ,0,1,2}, {2,3,0 ,0,0,0 ,2,3,0}, {4,5,6 ,1,1,1 ,3,0,1},
    {6,7,4 ,1,1,1 ,1,2,3}, {0,3,5 ,2,2,2 ,3,0,1}, {5,4,0 ,2,2,2 ,1,2,3},
    {3,2,6 ,3,3,3 ,3,0,1}, {6,5,3 ,3,3,3 ,1,2,3}, {2,1,7 ,4,4,4 ,3,0,1},
    {7,6,2 ,4,4,4 ,1,2,3}, {1,0,4 ,5,5,5 ,3,0,1}, {4,7,1 ,5,5,5 ,1,2,3}
  };

  for (int i = 0; i < 12; ++i)
  {
    Vec3f triPos[3] = {  pos[tris[i][0]],  pos[tris[i][1]],  pos[tris[i][2]]  };
    Vec3f triNorm[3] = { norm[tris[i][3]],  norm[tris[i][4]],   norm[tris[i][5]]  };
    Vec2f triTexc[3] =  {  texc[tris[i][6]],  texc[tris[i][7]],   texc[tris[i][8]]  };

    addTriangleToVBO(triPos, triNorm, triTexc);
  }
}

//------------------------------------------------------------------------

GLLineBox::GLLineBox()
{
  updateVBO();
}

GLLineBox::~GLLineBox()
{
}

//------------------------------------------------------------------------

int GLLineBox::getNumTriangles()
{
  return 0;
}

int GLLineBox::getNumLines()
{
  return 12;
}

//------------------------------------------------------------------------

void GLLineBox::updateVBO()
{
  static const Vec3f pos[8] =
  {
    Vec3f(-0.5f,-0.5f,0.5f), Vec3f(-0.5f,-0.5f,-0.5f), Vec3f(0.5f,-0.5f,-0.5f),
    Vec3f(0.5f,-0.5f,0.5f), Vec3f(-0.5f,0.5f,0.5f), Vec3f(0.5f,0.5f,0.5f),
    Vec3f(0.5f,0.5f,-0.5f), Vec3f(-0.5f,0.5f,-0.5f)
  };

  static const Vec3f norm[6] = 
  {
    Vec3f(0.0f,-1.0f,0.0f), Vec3f(0.0f,1.0f,0.0f), Vec3f(0.0f,0.0f,1.0f),
    Vec3f(1.0f,0.0f,0.0f), Vec3f(0.0f,0.0f,-1.0f), Vec3f(-1.0f,0.0f,0.0f)
  };

  static const Vec2f texc[4] = 
  {
    Vec2f(1.0f,0.0f), Vec2f(1.0f,1.0f), Vec2f(0.0f,1.0f), Vec2f(0.0f,0.0f)
  };

  // line: p,p  ,n,n ,t,t
  static const int lines[12][6] = 
  {
    {1,2, 0,0, 0,3}, {0,3, 0,0, 0,3}, {4,5, 0,0, 0,3}, {7,6, 0,0, 0,3},
    {1,7, 0,0, 0,3}, {0,4, 0,0, 0,3}, {2,6, 0,0, 0,3}, {3,5, 0,0, 0,3},
    {1,0, 0,0, 0,3}, {2,3, 0,0, 0,3}, {7,4, 0,0, 0,3}, {6,5, 0,0, 0,3}
  };

  for (int i = 0; i < 12; ++i)
  {
    Vec3f p[2] = {  pos[lines[i][0]],  pos[lines[i][1]]};
    Vec3f n[2] = { norm[lines[i][2]],  norm[lines[i][3]]};
    Vec2f t[2] =  {  texc[lines[i][4]],  texc[lines[i][5]]};

    addLineToVBO(p, n, t);
  }
}

//------------------------------------------------------------------------

//------------------------------------------------------------------------

GLDodecahedron::GLDodecahedron()
{
  updateVBO();
}

GLDodecahedron::~GLDodecahedron()
{
}

//------------------------------------------------------------------------

int GLDodecahedron::getNumTriangles()
{
  return 36;
}

//------------------------------------------------------------------------

void GLDodecahedron::updateVBO()
{
  static const Vec3f pos[20] =
  {Vec3f( 1         , 1         , 1           ),
   Vec3f( 1         , 1         , -1          ),
   Vec3f( 1         , -1        , 1           ),
   Vec3f( 1         , -1        , -1          ),
   Vec3f( -1        , 1         , 1           ),
   Vec3f( -1        , 1         , -1          ),
   Vec3f( -1        , -1        , 1           ),
   Vec3f( -1        , -1        , -1          ),
   Vec3f( 0         , 0.618034  , 1.61803     ),
   Vec3f( 0         , 0.618034  , -1.61803    ),
   Vec3f( 0         , -0.618034 , 1.61803     ),
   Vec3f( 0         , -0.618034 , -1.61803    ),
   Vec3f( 0.618034  , 1.61803   , 0           ),
   Vec3f( 0.618034  , -1.61803  , 0           ),
   Vec3f( -0.618034 , 1.61803   , 0           ),
   Vec3f( -0.618034 , -1.61803  , 0           ),
   Vec3f( 1.61803   , 0         , 0.618034    ),
   Vec3f( 1.61803   , 0         , -0.618034   ),
   Vec3f( -1.61803  , 0         , 0.618034    ),
   Vec3f( -1.61803  , 0         , -0.618034   )
  };

  static const Vec3f norm[20] =
  {
    Vec3f( 0.57735    , 0.57735   , 0.57735     ),
    Vec3f( 0.57735    , 0.57735   , -0.57735    ),
    Vec3f( 0.57735    , -0.57735  , 0.57735     ),
    Vec3f( 0.57735    , -0.57735  , -0.57735    ),
    Vec3f( -0.57735   , 0.57735   , 0.57735     ),
    Vec3f( -0.57735   , 0.57735   , -0.57735    ),
    Vec3f( -0.57735   , -0.57735  , 0.57735     ),
    Vec3f( -0.57735   , -0.57735  , -0.57735    ),
    Vec3f( 0          , 0.356822  , 0.934172    ),
    Vec3f( 0          , 0.356822  , -0.934172   ),
    Vec3f( 0          , -0.356822 , 0.934172    ),
    Vec3f( 0          , -0.356822 , -0.934172   ),
    Vec3f( 0.356822   , 0.934172  , 0           ),
    Vec3f( 0.356822   , -0.934172 , 0           ),
    Vec3f( -0.356822  , 0.934172  , 0           ),
    Vec3f( -0.356822  , -0.934172 , 0           ),
    Vec3f( 0.934172   , 0         , 0.356822    ),
    Vec3f( 0.934172   , 0         , -0.356822   ),
    Vec3f( -0.934172  , 0         , 0.356822    ),
    Vec3f( -0.934172  , 0         , -0.356822   )
  };

  static const Vec2f texc[20] =
  {
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1),
    Vec2f( 1, 1)
  };

  // not the usual p,p,p ,n,n,n ,t,t,t
  // i extracted those from an obj file so its:
  // tri: p,t,n  ,p,t,n ,p,t,n
  static const int tris[36][9] =
  {
    { 20,20,20, 19,19,19, 5,5,5    },
    { 12,12,12, 8,8,8, 20,20,20    },
    { 14,14,14, 3,3,3, 11,11,11    },
    { 17,17,17, 18,18,18, 2,2,2    },
    { 11,11,11, 3,3,3, 17,17,17    },
    { 14,14,14, 4,4,4, 18,18,18    },
    { 16,16,16, 8,8,8, 12,12,12    },
    { 7,7,7, 19,19,19, 20,20,20    },
    { 7,7,7, 11,11,11, 9,9,9       },
    { 1,1,1, 13,13,13, 15,15,15    },
    { 2,2,2, 10,10,10, 6,6,6       },
    { 18,18,18, 4,4,4, 12,12,12    },
    { 15,15,15, 6,6,6, 5,5,5       },
    { 6,6,6, 20,20,20, 5,5,5       },
    { 6,6,6, 10,10,10, 20,20,20    },
    { 10,10,10, 12,12,12, 20,20,20 },
    { 7,7,7, 16,16,16, 11,11,11    },
    { 16,16,16, 14,14,14, 11,11,11 },
    { 13,13,13, 1,1,1, 2,2,2       },
    { 1,1,1, 17,17,17, 2,2,2       },
    { 1,1,1, 9,9,9, 17,17,17       },
    { 9,9,9, 11,11,11, 17,17,17    },
    { 17,17,17, 3,3,3, 18,18,18    },
    { 3,3,3, 14,14,14, 18,18,18    },
    { 4,4,4, 14,14,14, 12,12,12    },
    { 14,14,14, 16,16,16, 12,12,12 },
    { 8,8,8, 16,16,16, 20,20,20    },
    { 16,16,16, 7,7,7, 20,20,20    },
    { 5,5,5, 19,19,19, 9,9,9       },
    { 19,19,19, 7,7,7, 9,9,9       },
    { 5,5,5, 9,9,9, 15,15,15       },
    { 9,9,9, 1,1,1, 15,15,15       },
    { 15,15,15, 13,13,13, 6,6,6    },
    { 13,13,13, 2,2,2, 6,6,6       },
    { 10,10,10, 2,2,2, 12,12,12    },
    { 2,2,2, 18,18,18, 12,12,12    }
  };

  //-1 since the objindices start at 1
  for (int i = 0; i < 36; ++i)
  {
    Vec3f p[3] = {  pos[tris[i][0]-1],  pos[tris[i][3]-1], pos[tris[i][6]-1]};
    Vec3f n[3] = { norm[tris[i][2]-1],  norm[tris[i][5]-1], norm[tris[i][8]-1]};
    Vec2f t[3] =  {  texc[tris[i][1]-1],  texc[tris[i][4]-1], texc[tris[i][7]-1]};

    addTriangleToVBO(p, n, t);
  }
}

//------------------------------------------------------------------------

GLIcosahedron::GLIcosahedron()
{
  updateVBO();
}

GLIcosahedron::~GLIcosahedron()
{
}

//------------------------------------------------------------------------

int GLIcosahedron::getNumTriangles()
{
  return 20;
}

//------------------------------------------------------------------------

void GLIcosahedron::updateVBO()
{
  static const Vec3f pos[12] =
  {
    Vec3f( 0        , -1      , -1.61803),
    Vec3f( 0        , 1       , -1.61803),
    Vec3f( 0        , 1       , 1.61803 ),
    Vec3f( 0        , -1      , 1.61803 ),
    Vec3f( -1       , -1.61803, 0       ),
    Vec3f( 1        , -1.61803, 0       ),
    Vec3f( 1        , 1.61803 , 0       ),
    Vec3f( -1       , 1.61803 , 0       ),
    Vec3f( -1.61803 , 0       , -1      ),
    Vec3f( -1.61803 , 0       , 1       ),
    Vec3f( 1.61803  , 0       , 1       ),
    Vec3f( 1.61803  , 0       , -1      )
  };

  static const Vec3f norm[12] =
  {
    Vec3f( 0          , -0.525731  , -0.850651   ),
    Vec3f( 2.79423e-17, 0.525731   , -0.850651   ),
    Vec3f( 0          , 0.525731   , 0.850651    ),
    Vec3f( 2.79423e-17, -0.525731  , 0.850651    ),
    Vec3f( -0.525731  , -0.850651  , 0           ),
    Vec3f( 0.525731   , -0.850651  , 2.79423e-17 ),
    Vec3f( 0.525731   , 0.850651   , 0           ),
    Vec3f( -0.525731  , 0.850651   , 0           ),
    Vec3f( -0.850651  , 2.79423e-17, 0.525731    ),
    Vec3f( 0.850651   , 0          , 0.525731    ),
    Vec3f( -0.850651  , 0          , -0.525731   ),
    Vec3f( 0.850651   , 2.79423e-17, -0.525731   )
  };

  static const Vec2f texc[12] =
  {
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0)
  };

  // not the usual p,p,p ,n,n,n ,t,t,t
  // i extracted those from an obj file so its:
  // tri: p,t,n  ,p,t,n ,p,t,n
  static const int tris[20][9] =
  {
    { 3,3,3, 7,7,7, 8,8,8,       },
    { 8,8,8, 7,7,7, 2,2,2,       },
    { 12,12,12, 1,1,1, 2,2,2,    },
    { 1,1,1, 9,9,9, 2,2,2,       },
    { 5,5,5, 10,10,10, 9,9,9,    },
    { 9,9,9, 10,10,10, 8,8,8,    },
    { 10,10,10, 4,4,4, 3,3,3,    },
    { 11,11,1,3,3,3, 4,4,4,      },
    { 6,6,6, 12,12,12, 11,11,1,  },
    { 12,12,12, 7,7,7, 11,11,1,  },
    { 1,1,1, 6,6,6, 5,5,5,       },
    { 6,6,6, 4,4,4, 5,5,5,       },
    { 7,7,7, 3,3,3, 11,11,1,     },
    { 7,7,7, 12,12,12, 2,2,2,    },
    { 2,2,2, 9,9,9, 8,8,8,       },
    { 10,10,10, 3,3,3, 8,8,8,    },
    { 4,4,4, 6,6,6, 11,11,1,     },
    { 1,1,1, 12,12,12, 6,6,6,    },
    { 4,4,4, 10,10,10, 5,5,5,    },
    { 1,1,1, 5,5,5, 9,9,9,       }
  };

  //-1 since the objindices start at 1
  for (int i = 0; i < 20; ++i)
  {
    Vec3f p[3] = {  pos[tris[i][0]-1],  pos[tris[i][3]-1], pos[tris[i][6]-1]};
    Vec3f n[3] = { norm[tris[i][2]-1],  norm[tris[i][5]-1], norm[tris[i][8]-1]};
    Vec2f t[3] =  {  texc[tris[i][1]-1],  texc[tris[i][4]-1], texc[tris[i][7]-1]};

    addTriangleToVBO(p, n, t);
  }
}

//------------------------------------------------------------------------

GLOctahedron::GLOctahedron()
{
  updateVBO();
}

GLOctahedron::~GLOctahedron()
{
}

//------------------------------------------------------------------------

int GLOctahedron::getNumTriangles()
{
  return 8;
}

//------------------------------------------------------------------------

void GLOctahedron::updateVBO()
{
  static const Vec3f pos[6] =
  {
    Vec3f( -1.41421 , 0       , 0       ),
    Vec3f( 0        , -1.41421, 0       ),
    Vec3f( 1.41421  , 0       , 0       ),
    Vec3f( 0        , 1.41421 , 0       ),
    Vec3f( 0        , 0       , 1.41421 ),
    Vec3f( 0        , 0       , -1.41421)
  };

  static const Vec3f norm[6] =
  {
    Vec3f(-1, 0 , 0 ),
    Vec3f(0 , -1, 0 ),
    Vec3f(1 , 0 , 0 ),
    Vec3f(0 , 1 , 0 ),
    Vec3f(0 , 0 , 1 ),
    Vec3f(0 , 0 , -1)
  };

  static const Vec2f texc[6] =
  {
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0)
  };

  // not the usual p,p,p ,n,n,n ,t,t,t
  // i extracted those from an obj file so its:
  // tri: p,t,n  ,p,t,n ,p,t,n
  static const int tris[8][9] =
  {
    { 1,1,1, 2,2,2, 5,5,5 },
    { 2,2,2, 3,3,3, 5,5,5 },
    { 3,3,3, 4,4,4, 5,5,5 },
    { 1,1,1, 5,5,5, 4,4,4 },
    { 6,6,6, 2,2,2, 1,1,1 },
    { 6,6,6, 3,3,3, 2,2,2 },
    { 6,6,6, 4,4,4, 3,3,3 },
    { 6,6,6, 1,1,1, 4,4,4 }
  };

  //-1 since the objindices start at 1
  for (int i = 0; i < 8; ++i)
  {
    Vec3f p[3] = {  pos[tris[i][0]-1],  pos[tris[i][3]-1], pos[tris[i][6]-1]};
    Vec3f n[3] = { norm[tris[i][2]-1],  norm[tris[i][5]-1], norm[tris[i][8]-1]};
    Vec2f t[3] =  {  texc[tris[i][1]-1],  texc[tris[i][4]-1], texc[tris[i][7]-1]};

    addTriangleToVBO(p, n, t);
  }
}

//------------------------------------------------------------------------

GLTetrahedron::GLTetrahedron()
{
  updateVBO();
}

GLTetrahedron::~GLTetrahedron()
{
}

//------------------------------------------------------------------------

int GLTetrahedron::getNumTriangles()
{
  return 4;
}

//------------------------------------------------------------------------

void GLTetrahedron::updateVBO()
{
  static const Vec3f pos[4] =
  {
    Vec3f( -1, -1, 1  ),
    Vec3f( 1 , 1 , 1  ),
    Vec3f( -1, 1 , -1 ),
    Vec3f( 1 , -1, -1 )
  };

  static const Vec3f norm[4] =
  {
    Vec3f( -0.57735, -0.57735, 0.57735  ),
    Vec3f( 0.57735 , 0.57735 , 0.57735  ),
    Vec3f( -0.57735, 0.57735 , -0.57735 ),
    Vec3f( 0.57735 , -0.57735, -0.57735 )
  };

  static const Vec2f texc[4] =
  {
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0),
    Vec2f( 0, 0)
  };

  // not the usual p,p,p ,n,n,n ,t,t,t
  // i extracted those from an obj file so its:
  // tri: p,t,n  ,p,t,n ,p,t,n
  static const int tris[4][9] =
  {
    { 1, 1, 1, 2, 2, 2, 3, 3, 3 },
    { 1, 1, 1, 3, 3, 3, 4, 4, 4 },
    { 3, 3, 3, 2, 2, 2, 4, 4, 4 },
    { 4, 4, 4, 2, 2, 2, 1, 1, 1 }
  };

  //-1 since the objindices start at 1
  for (int i = 0; i < 4; ++i)
  {
    Vec3f p[3] = {  pos[tris[i][0]-1],  pos[tris[i][3]-1], pos[tris[i][6]-1]};
    Vec3f n[3] = { norm[tris[i][2]-1],  norm[tris[i][5]-1], norm[tris[i][8]-1]};
    Vec2f t[3] =  {  texc[tris[i][1]-1],  texc[tris[i][4]-1], texc[tris[i][7]-1]};

    addTriangleToVBO(p, n, t);
  }
}

//------------------------------------------------------------------------
GLTorus::GLTorus(GLdouble innerRadius,
                 GLdouble outerRadius,
                 GLint nsides, GLint rings):
  rings_(rings),
  nsides_(nsides),
  innerRadius_(innerRadius),
  outerRadius_(outerRadius)
{

  updateVBO();
}

GLTorus::~GLTorus()
{
}

//------------------------------------------------------------------------

int GLTorus::getNumTriangles()
{
  return rings_*nsides_*2;
}

//------------------------------------------------------------------------

void GLTorus::updateVBO()
{
  // build the Torus in two steps.
  // First build a unit Ring in XZ plane.
  // Then copy and rotate the ring around z axis to form a torus.
  //
  // create faces and normals + texcoordinates.
  // Inner seam is top/bottom of texture
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  std::vector<Vec2f> texCoords;
  std::vector<Vec3f> unitRing;

  float ringSegmentAngle =  2 * M_PI / nsides_;
  float torusSegmentAngle = 2 * M_PI / rings_;

  // generate a unit ring with n sides
  for (int i = 0 ; i < nsides_ ; ++i)
  {
    Vec3f ringPoint = Vec3f(cos(ringSegmentAngle * i),0.0f,sin(ringSegmentAngle *i));
    unitRing.push_back(ringPoint);
        ////cosCache[i] = cos(angle);
  }

  for(int j = 0; j < rings_ ; ++j)
  {
    Vec3f torusPoint;
    int side = 0;
    for(Vec3f point : unitRing)
    {
      //scale the unit Ring
      torusPoint = point * (1.f/innerRadius_);
      //translate the unit Ring
      torusPoint +=Vec3f(outerRadius_ + innerRadius_,0,0);
      //rotate the translated ring around the z axis
      Vec3f pointOnTorus = Vec3f(torusPoint[0] * cos(j*torusSegmentAngle) + torusPoint[1] * sin(j*torusSegmentAngle),
            torusPoint[0] * -sin(j*torusSegmentAngle) + torusPoint[1] * cos(j*torusSegmentAngle),
            torusPoint[2]);

      points.push_back(pointOnTorus);

      //the normal does not have to be translated or scaled
      Vec3f normalOnTorus = Vec3f(point[0] * cos(j*torusSegmentAngle) + point[1] * sin(j*torusSegmentAngle),
          point[0] * -sin(j*torusSegmentAngle) + point[1] * cos(j*torusSegmentAngle),
          point[2]);
      // probably not necessary as we are working with the unit circle
      normalOnTorus.normalize();
      normals.push_back(normalOnTorus);

      // texcoords are a simple uv unwrap of the torus cut open.
      // j/rings_ is directly the u component
      // the ring segment gives us the v component
      texCoords.push_back(Vec2f(j/rings_,side/nsides_));
      ++side;
    }
  }

  // points are now stored one ring after the other in the points vector.
  // creating a triangle is fairly easy adding nsides_ to the current index gives the same point on the next ring.
  // create triangles as point on ring -> same point on next ring -> next point on next ring.
  int end = rings_*nsides_;
  int segmentOffset;
  int oddIndex;
  for(int i = 0 ; i < end ; ++i)
  {
    if((i+1) % nsides_ == 0)
    {
      segmentOffset = 0;
      oddIndex = -nsides_;
    }
    else
    {
      segmentOffset = nsides_;
      oddIndex = 0;
    }
    // first face (lower left triangle)
    Vec3f p[3] = { points[i]    ,  points[(i+nsides_)%end]    , points[(i+segmentOffset+1)%end]    };
    Vec3f n[3] = { normals[i]   ,  normals[(i+nsides_)%end]   , normals[(i+segmentOffset+1)%end]   };
    Vec2f t[3] = { texCoords[i] ,  texCoords[(i+nsides_)%end] , texCoords[(i+segmentOffset+1)%end] };
    addTriangleToVBO(p, n, t);
    // second face (upper right triangle)
    Vec3f p2[3] = { points[(i+1 + oddIndex)%end]    , points[(i)%end]   , points[(i+segmentOffset+1)%end]    };
    Vec3f n2[3] = { normals[(i+1 + oddIndex)%end]   , normals[(i)%end]  , normals[(i+segmentOffset+1)%end]   };
    Vec2f t2[3] = { texCoords[(i+1 + oddIndex)%end] , texCoords[(i)%end], texCoords[(i+segmentOffset+1)%end] };
    addTriangleToVBO(p2, n2, t2);
  }
}

//------------------------------------------------------------------------
}

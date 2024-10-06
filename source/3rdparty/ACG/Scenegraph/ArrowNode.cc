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
//  CLASS ArrowNode - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================
#include <ACG/GL/acg_glew.hh>
#include "ArrowNode.hh"
#include <ACG/GL/IRenderer.hh>
#include <OpenMesh/Core/Utils/vector_cast.hh>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== IMPLEMENTATION ==========================================================

ArrowNode::ArrowNode(BaseNode*    _parent, std::string  _name )
 : MaterialNode(_parent, _name, MaterialNode::BaseColor),
   numVertices_(0),
   numIndices_(0),
   localArrowMin_(0.0f, 0.0f, 0.0f),
   localArrowMax_(0.0f, 0.0f, 0.0f),
   invalidateInstanceData_(true),
   invalidateInstanceBuffer_(true),
   supportsInstancing_(-1)
{
  drawMode(DrawModes::SOLID_FLAT_SHADED);
}

//----------------------------------------------------------------------------

ArrowNode::~ArrowNode() 
{

}
//----------------------------------------------------------------------------

void ArrowNode::boundingBox(Vec3d& _bbMin, Vec3d& _bbMax)
{
  float radius = std::max(localArrowMax_[1] - localArrowMin_[1], localArrowMax_[2] - localArrowMin_[2]) * 0.5f;

  size_t n = arrows_.size();
  for (size_t i = 0; i < n; ++i)
  {
    const Arrow* a = &arrows_[i];

    // start and end point
    Vec3f s = a->start;
    Vec3f e = a->start + a->dir * a->scale[1];

    // conv to double precision
    Vec3d sd = OpenMesh::vector_cast<Vec3d>(s);
    Vec3d ed = OpenMesh::vector_cast<Vec3d>(e);

    // enlarge aabb by some amount to account for the volumetric shape
    Vec3d volEnlargeOffset = OpenMesh::vector_cast<Vec3d>(radius * a->scale);

    _bbMin.minimize(sd - volEnlargeOffset);
    _bbMin.minimize(ed - volEnlargeOffset);

    _bbMax.maximize(sd + volEnlargeOffset);
    _bbMax.maximize(ed + volEnlargeOffset);
  }
}


//----------------------------------------------------------------------------

DrawModes::DrawMode ArrowNode::availableDrawModes() const
{
  return DrawModes::SOLID_FLAT_SHADED;
}

//----------------------------------------------------------------------------

void ArrowNode::createArrowMesh()
{
  if (!numVertices_)
  {
    // arrow mesh data as result from meshcompiler:
    // interleaved float3 pos, float3 normal
    float vdata[] =
    {
      0.681818f, -0.034091f, -0.068182f, 0.000000f, -1.000000f, 0.000000f,
      0.681818f, -0.034091f, 0.068182f, 0.000000f, -1.000000f, 0.000000f,
      0.000000f, -0.034091f, 0.000000f, 0.000000f, -1.000000f, 0.000000f,
      1.000000f, -0.034091f, 0.000000f, 0.000000f, -1.000000f, 0.000000f,
      0.681818f, -0.034091f, -0.227273f, 0.000000f, -1.000000f, 0.000000f,
      0.681818f, -0.034091f, 0.227273f, 0.000000f, -1.000000f, 0.000000f,
      0.681818f, 0.034091f, -0.068182f, 0.000000f, 1.000000f, -0.000000f,
      1.000000f, 0.034091f, 0.000000f, 0.000000f, 1.000000f, 0.000000f,
      0.681818f, 0.034091f, -0.227273f, -0.000000f, 1.000000f, 0.000000f,
      0.681818f, 0.034091f, 0.068182f, 0.000000f, 1.000000f, -0.000000f,
      0.000000f, 0.034091f, 0.000000f, 0.000000f, 1.000000f, -0.000000f,
      0.681818f, 0.034091f, 0.227273f, 0.000000f, 1.000000f, 0.000000f,
      1.000000f, -0.034091f, 0.000000f, 0.581238f, 0.000000f, -0.813734f,
      0.681818f, 0.034091f, -0.227273f, 0.581238f, 0.000000f, -0.813734f,
      1.000000f, 0.034091f, 0.000000f, 0.581238f, 0.000000f, -0.813734f,
      0.681818f, -0.034091f, -0.227273f, 0.581238f, 0.000000f, -0.813734f,
      0.681818f, -0.034091f, 0.227273f, 0.581238f, 0.000000f, 0.813734f,
      1.000000f, 0.034091f, 0.000000f, 0.581238f, 0.000000f, 0.813734f,
      0.681818f, 0.034091f, 0.227273f, 0.581238f, -0.000000f, 0.813734f,
      1.000000f, -0.034091f, 0.000000f, 0.581238f, 0.000000f, 0.813734f,
      0.681818f, -0.034091f, 0.068182f, -1.000000f, -0.000000f, 0.000000f,
      0.681818f, 0.034091f, 0.227273f, -1.000000f, -0.000000f, 0.000000f,
      0.681818f, 0.034091f, 0.068182f, -1.000000f, 0.000000f, 0.000000f,
      0.681818f, -0.034091f, 0.227273f, -1.000000f, -0.000000f, 0.000000f,
      0.681818f, -0.034091f, -0.227273f, -1.000000f, -0.000000f, 0.000000f,
      0.681818f, 0.034091f, -0.068182f, -1.000000f, -0.000000f, 0.000000f,
      0.681818f, 0.034091f, -0.227273f, -1.000000f, 0.000000f, 0.000000f,
      0.681818f, -0.034091f, -0.068182f, -1.000000f, -0.000000f, 0.000000f,
      0.000000f, -0.034091f, 0.000000f, -0.099504f, 0.000000f, 0.995037f,
      0.681818f, 0.034091f, 0.068182f, -0.099504f, 0.000000f, 0.995037f,
      0.000000f, 0.034091f, 0.000000f, -0.099504f, 0.000000f, 0.995037f,
      0.681818f, -0.034091f, 0.068182f, -0.099504f, 0.000000f, 0.995037f,
      0.681818f, -0.034091f, -0.068182f, -0.099504f, -0.000000f, -0.995037f,
      0.000000f, 0.034091f, 0.000000f, -0.099504f, -0.000000f, -0.995037f,
      0.681818f, 0.034091f, -0.068182f, -0.099504f, 0.000000f, -0.995037f,
      0.000000f, -0.034091f, 0.000000f, -0.099504f, -0.000000f, -0.995037f,
    };

    // indices
    int idata[] =
    {
      0, 1, 2,
      0, 3, 1,
      0, 4, 3,
      1, 3, 5,
      6, 7, 8,
      6, 9, 7,
      6, 10, 9,
      7, 9, 11,
      12, 13, 14,
      13, 12, 15,
      16, 17, 18,
      17, 16, 19,
      20, 21, 22,
      21, 20, 23,
      24, 25, 26,
      25, 24, 27,
      28, 29, 30,
      29, 28, 31,
      32, 33, 34,
      33, 32, 35,
    };

    // vertex decl
    if (!vertexDecl_.getNumElements())
    {
      vertexDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
      vertexDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_NORMAL);
    }

    numVertices_ = sizeof(vdata) / vertexDecl_.getVertexStride();
    numIndices_ = sizeof(idata) / sizeof(idata[0]);

    // vertex + index buffer
    vertexBuffer_.upload(sizeof(vdata), vdata, GL_STATIC_DRAW);
    indexBuffer_.upload(sizeof(idata), idata, GL_STATIC_DRAW);

    // compute local aabb
    localArrowMin_ = Vec3f(vdata[0], vdata[1], vdata[2]);
    localArrowMax_ = localArrowMin_;

    for (int i = 1; i < numVertices_; ++i)
    {
      Vec3f v = Vec3f(vdata[i * 6], vdata[i * 6 + 1], vdata[i * 6 + 2]);
      localArrowMin_.minimize(v);
      localArrowMax_.maximize(v);
    }
  }
}

//----------------------------------------------------------------------------

void ArrowNode::draw(GLState&  _state, const DrawModes::DrawMode& _drawMode)
{
  createArrowMesh();
  updateInstanceData();

  vertexBuffer_.bind();
  indexBuffer_.bind();

  vertexDecl_.activateFixedFunction();

  // save model-view matrix 
  GLMatrixf viewMatrix;
  glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat*)&viewMatrix);


  // GL_COLOR_MATERIAL: multiply the glColor value with the ambient and diffuse term
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

  for (size_t i = 0; i < arrows_.size(); ++i)
  {
    // use transform of the individual arrows
    GLMatrixf worldView = viewMatrix * readWorldMatrix(i);
    glLoadMatrixf(worldView.data());

    Vec4uc c = arrows_[i].color;
    glColor4ub(c[0], c[1], c[2], c[3]);

    glDrawElements(GL_TRIANGLES, numIndices_, GL_UNSIGNED_INT, 0);
  }

  // restore model-view matrix
  glLoadMatrixf(viewMatrix.data());

  glDisable(GL_COLOR_MATERIAL);
  vertexDecl_.deactivateFixedFunction();
}

//----------------------------------------------------------------------------

void ArrowNode::getRenderObjects(IRenderer* _renderer, GLState&  _state , const DrawModes::DrawMode&  _drawMode , const ACG::SceneGraph::Material* _mat) 
{
  if (arrows_.empty())
    return;

  createArrowMesh();
  updateInstanceBuffer();

  
  RenderObject obj;
  obj.initFromState(&_state);
  obj.depthTest = true;

  obj.vertexBuffer = vertexBuffer_.id();
  obj.indexBuffer = indexBuffer_.id();


  // check support for instancing if not done yet
  if (supportsInstancing_ < 0)
    supportsInstancing_ = checkExtensionSupported("GL_ARB_instanced_arrays") ? 1 : 0;

  // config shader
  obj.shaderDesc.shadeMode = SG_SHADE_FLAT;

  if (supportsInstancing_)
  {
    // render with instancing
    obj.shaderDesc.vertexColors = true;
    obj.shaderDesc.vertexTemplateFile = "ArrowNode/instancing_vs.glsl";
    obj.vertexDecl = &vertexDeclInstanced_;
    obj.glDrawElementsInstanced(GL_TRIANGLES, numIndices_, GL_UNSIGNED_INT, 0, arrows_.size());

    _renderer->addRenderObject(&obj);
  }
  else
  {
    // no instancing support
    // might want to abort after the first few objects to avoid emitting thousands of objects here and overloading the renderer
    
    obj.shaderDesc.vertexColors = false;
    obj.vertexDecl = &vertexDecl_;

    GLMatrixf viewMatrix = obj.modelview;

    for (size_t i = 0; i < arrows_.size(); ++i)
    {
      const Arrow* a = &arrows_[i];

      obj.modelview = viewMatrix * readWorldMatrix(i);
      obj.diffuse = Vec3f(a->color[0] / 255.0f, a->color[1] / 255.0f, a->color[2] / 255.0f);
      obj.alpha = a->color[3] / 255.0f;

      obj.glDrawElements(GL_TRIANGLES, numIndices_, GL_UNSIGNED_INT, 0);
      _renderer->addRenderObject(&obj);
    }
  }
}

//----------------------------------------------------------------------------

void ArrowNode::reserve(int _n)
{
  arrows_.reserve(_n);
}

//----------------------------------------------------------------------------

void ArrowNode::clear()
{
  arrows_.clear();
}

//----------------------------------------------------------------------------

int ArrowNode::n_arrows() const
{
  return int(arrows_.size());
}

//----------------------------------------------------------------------------

GLMatrixf ArrowNode::computeWorldMatrix(int _arrow) const
{
  const Arrow* a = &arrows_[_arrow];

  GLMatrixf align;
  align.identity();

  align.translate(a->start);

  // orientation
  //  local mesh stored as: dir in +x,  normal in +y
  Vec3f binormal = a->dir % a->normal;
  for (int i = 0; i < 3; ++i)
  {
    align(i, 0) = a->dir[i];
    align(i, 1) = a->normal[i];
    align(i, 2) = binormal[i];
  }

  // scaling vector: width, length, height
  align.scale(a->scale[1], a->scale[2], a->scale[0]);

  return align;
}

//----------------------------------------------------------------------------

int ArrowNode::addArrow(const Vec3f& _start, const Vec3f& _dir, const Vec3f& _normal, const Vec3f& _scale, const Vec4uc& _color)
{
  Arrow a;
  a.start = _start;
  a.dir = _dir;
  a.scale = _scale;
  a.color = _color;
  a.normal = _normal;

  a.orthonormalize();

  arrows_.push_back(a);

  invalidateInstanceData_ = true;
  invalidateInstanceBuffer_ = true;

  return int(arrows_.size()) - 1;
}

//----------------------------------------------------------------------------

int ArrowNode::addArrow(const Vec3f& _start, const Vec3f& _dir, const Vec3f& _normal, const Vec3f& _scale, const Vec4f& _color)
{
  Vec4uc c;
  for (int i = 0; i < 4; ++i)
    c[i] = std::min(std::max(int(_color[i] * 255.0f), 0), 255);
  return addArrow(_start, _dir, _normal, _scale, c);
}

//----------------------------------------------------------------------------

void ArrowNode::Arrow::orthonormalize()
{
  dir.normalize();

  // make sure dir and normal are linearly independent
  if (normal.sqrnorm() < 1e-6f)
    normal = Vec3f(0.0f, 1.0f, 0.0f);

  normal.normalize();

  while (std::fabs(dir | normal) > 0.99f || normal.sqrnorm() < 0.01f)
  {
    for (int i = 0; i < 3; ++i)
      normal[i] = float(rand()) / float(RAND_MAX) * 2.0f - 1.0f;
    normal.normalize();
  }
  // orthogonalize normal dir
  Vec3f binormal = dir % normal;
  normal = (binormal % dir).normalized();
}

//----------------------------------------------------------------------------

int ArrowNode::addArrow(const Vec3d& _start, const Vec3d& _dir, const Vec3d& _normal, const Vec3d& _scale, const Vec4uc& _color)
{
  return addArrow(OpenMesh::vector_cast<Vec3f>(_start), OpenMesh::vector_cast<Vec3f>(_dir), OpenMesh::vector_cast<Vec3f>(_normal), OpenMesh::vector_cast<Vec3f>(_scale), _color);
}

//----------------------------------------------------------------------------

int ArrowNode::addArrow(const Vec3d& _start, const Vec3d& _dir, const Vec3d& _normal, const Vec3d& _scale, const Vec4f& _color)
{
  Vec4uc c;
  for (int i = 0; i < 4; ++i)
    c[i] = std::min(std::max(int(_color[i] * 255.0f), 0), 255);
  return addArrow(_start, _dir, _normal, _scale, c);
}

//----------------------------------------------------------------------------

void ArrowNode::updateInstanceData()
{
  if (invalidateInstanceData_)
  {
    const int numArrows = arrows_.size();

    // size in dwords of data for one instance
    const int instanceSize = instanceDataSize();

    instanceData_.resize(numArrows * instanceSize);

    for (int i = 0; i < numArrows; ++i)
    {
      // compute and store 4x3 world matrix
      GLMatrixf m = computeWorldMatrix(i);
      int offset = instanceDataOffset(i);

      // linearize matrix and store in row-major
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
          instanceData_[offset + r*4 + c] = m(r,c);

      // store inverse transpose to support lighting with non-uniform scaling
      m.invert();
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          instanceData_[offset + 4 * 3 + r * 3 + c] = m(c, r);

      // store color in last dword as rgba8_unorm
//      instanceData_[offset + 4 * 3] = *(float*)(arrows_[i].color.data());
      memcpy(&instanceData_[offset + 4*3 + 3*3], arrows_[i].color.data(), 4);

      // append more data here as needed
    }

    invalidateInstanceData_ = false;
  }
}

//----------------------------------------------------------------------------

ACG::GLMatrixf ArrowNode::readWorldMatrix( int _arrow ) const
{
  int offset = instanceDataOffset(_arrow);
  GLMatrixf m;

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      m(r,c) = instanceData_[offset + r*4 + c];

  // last row not explicitly stored
  m(3,0) = 0.0f;  m(3,1) = 0.0f;  m(3,2) = 0.0f;  m(3,3) = 1.0f;

  return m;
}

//----------------------------------------------------------------------------

void ArrowNode::updateInstanceBuffer()
{
  if (invalidateInstanceBuffer_ || invalidateInstanceData_)
  {
    updateInstanceData();

    if (!instanceData_.empty())
      instanceBuffer_.upload(4 * instanceData_.size(), &instanceData_[0], GL_STATIC_DRAW);

    if (!vertexDeclInstanced_.getNumElements())
    {
      // position and normal from static mesh vbo
      vertexDeclInstanced_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
      vertexDeclInstanced_.addElement(GL_FLOAT, 3, VERTEX_USAGE_NORMAL);

      // world matrix and color from instance data vbo
      vertexDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "inWorld0", 1, instanceBuffer_.id());
      vertexDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "inWorld1", 1, instanceBuffer_.id());
      vertexDeclInstanced_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "inWorld2", 1, instanceBuffer_.id());
      vertexDeclInstanced_.addElement(GL_FLOAT, 3, VERTEX_USAGE_SHADER_INPUT, size_t(0), "inWorldIT0", 1, instanceBuffer_.id());
      vertexDeclInstanced_.addElement(GL_FLOAT, 3, VERTEX_USAGE_SHADER_INPUT, size_t(0), "inWorldIT1", 1, instanceBuffer_.id());
      vertexDeclInstanced_.addElement(GL_FLOAT, 3, VERTEX_USAGE_SHADER_INPUT, size_t(0), "inWorldIT2", 1, instanceBuffer_.id());
      vertexDeclInstanced_.addElement(GL_UNSIGNED_BYTE, 4, VERTEX_USAGE_COLOR, size_t(0), 0, 1, instanceBuffer_.id());
    }

    invalidateInstanceBuffer_ = false;
  }
}

//----------------------------------------------------------------------------

Vec3f ArrowNode::arrowStart(int _arrowID) const
{
  return arrows_[_arrowID].start;
}

void ArrowNode::arrowStart(int _arrowID, const Vec3f& _start)
{
  arrows_[_arrowID].start = _start;
  invalidateInstanceData_ = true;
}

Vec3f ArrowNode::arrowDir(int _arrowID) const
{
  return arrows_[_arrowID].dir;
}

void ArrowNode::arrowDir(int _arrowID, const Vec3f& _dir)
{
  arrows_[_arrowID].dir = _dir;
  arrows_[_arrowID].orthonormalize();
  invalidateInstanceData_ = true;
}

Vec3f ArrowNode::arrowNormal(int _arrowID) const
{
  return arrows_[_arrowID].normal;
}

void ArrowNode::arrowNormal(int _arrowID, const Vec3f& _normal)
{
  arrows_[_arrowID].normal = _normal;
  arrows_[_arrowID].orthonormalize();
  invalidateInstanceData_ = true;
}

Vec3f ArrowNode::arrowScale(int _arrowID) const
{
  return arrows_[_arrowID].scale;
}

void ArrowNode::arrowScale(int _arrowID, const Vec3f& _scale)
{
  arrows_[_arrowID].scale = _scale;
  invalidateInstanceData_ = true;
}

Vec4uc ArrowNode::arrowColor(int _arrowID) const
{
  return arrows_[_arrowID].color;
}

void ArrowNode::arrowColor(int _arrowID, const Vec4uc& _color)
{
  arrows_[_arrowID].color = _color;
  invalidateInstanceData_ = true;
}


//=============================================================================


} // namespace SceneGraph
} // namespace ACG
//=============================================================================

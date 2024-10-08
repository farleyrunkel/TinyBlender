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
//  CLASS PointNode - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include <ACG/GL/acg_glew.hh>
#include "PointNode.hh"
#include <ACG/GL/IRenderer.hh>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ========================================================== 


void
PointNode::
boundingBox(Vec3d& _bbMin, Vec3d& _bbMax)
{
  ConstPointIter p_it=points_.begin(), p_end=points_.end();
  for (; p_it!=p_end; ++p_it) {
    _bbMin.minimize(*p_it); 
    _bbMax.maximize(*p_it);
  }
}


//----------------------------------------------------------------------------

  
DrawModes::DrawMode
PointNode::
availableDrawModes() const
{
  return ( DrawModes::POINTS | 
	   DrawModes::POINTS_SHADED | 
	   DrawModes::POINTS_COLORED );
}


//----------------------------------------------------------------------------


void
PointNode::
draw(GLState& /* _state */ , const DrawModes::DrawMode& _drawMode)
{
  if (points_.empty())
    return;
  
  // points
  if (_drawMode & DrawModes::POINTS)
  {
    ACG::GLState::disable(GL_LIGHTING);
    ACG::GLState::enableClientState(GL_VERTEX_ARRAY);
    ACG::GLState::vertexPointer(&points_[0]);
    glDrawArrays(GL_POINTS, 0, int(points_.size()));
  }


  // points and normals
  if (_drawMode & DrawModes::POINTS_SHADED)
  {
    if (points_.size() == normals_.size())
    {
      ACG::GLState::enable(GL_LIGHTING);
      ACG::GLState::enableClientState(GL_VERTEX_ARRAY);
      ACG::GLState::vertexPointer(&points_[0]);
      ACG::GLState::enableClientState(GL_NORMAL_ARRAY);
      ACG::GLState::normalPointer(&normals_[0]);
      glDrawArrays(GL_POINTS, 0, int(points_.size()));
    }
  }


  // points and colors
  if (_drawMode & DrawModes::POINTS_COLORED)
  {
    if (points_.size() == colors_.size())
    {
      ACG::GLState::disable(GL_LIGHTING);
      ACG::GLState::enableClientState(GL_VERTEX_ARRAY);
      ACG::GLState::vertexPointer(&points_[0]);
      ACG::GLState::enableClientState(GL_COLOR_ARRAY);
      ACG::GLState::colorPointer(&colors_[0]);
      glDrawArrays(GL_POINTS, 0, int(points_.size()));
    } else
      std::cerr << "Mismatch size!" << std::endl;
  }


  // disable arrays
  ACG::GLState::disableClientState(GL_VERTEX_ARRAY);
  ACG::GLState::disableClientState(GL_NORMAL_ARRAY);
  ACG::GLState::disableClientState(GL_COLOR_ARRAY);
}

void
PointNode::
update_vbo()
{
  if (!vbo_needs_update_)
    return;

  if (vbo_ == 0) {
    glGenBuffers(1, &vbo_);
  }

  vertexDecl_.clear();

  vertexDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
  // number of floats per point
  size_t elem_size = 3;
  if (!normals_.empty())
  {
    assert(normals_.size() == points_.size());
    vertexDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_NORMAL, elem_size*sizeof(float));
    elem_size += 3;
  }
  if (!colors_.empty())
  {
    assert(colors_.size() == points_.size());
    vertexDecl_.addElement(GL_FLOAT, 4, VERTEX_USAGE_COLOR, elem_size*sizeof(float));
    elem_size += 4;
  }
  vertexDecl_.setVertexStride(elem_size * sizeof(float));

  vbo_data_.clear();
  vbo_data_.reserve(elem_size * points_.size());

  for (size_t i=0; i < points_.size(); ++i) {
    vbo_data_.push_back(static_cast<float>(points_[i][0]));
    vbo_data_.push_back(static_cast<float>(points_[i][1]));
    vbo_data_.push_back(static_cast<float>(points_[i][2]));
    if (!normals_.empty()) {
      vbo_data_.push_back(static_cast<float>(normals_[i][0]));
      vbo_data_.push_back(static_cast<float>(normals_[i][1]));
      vbo_data_.push_back(static_cast<float>(normals_[i][2]));
    }
    if (!colors_.empty()) {
      vbo_data_.push_back(colors_[i][0]);
      vbo_data_.push_back(colors_[i][1]);
      vbo_data_.push_back(colors_[i][2]);
      vbo_data_.push_back(colors_[i][3]);
    }
  }
  assert(vbo_data_.size() == points_.size() * elem_size);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vbo_data_.size()*sizeof(float) , vbo_data_.data() , GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  vbo_needs_update_ = false;
}

void
PointNode::
getRenderObjects( IRenderer* _renderer, GLState& _state , const DrawModes::DrawMode& _drawMode , const Material* _mat )
{
  if (points_.empty())
    return;

  update_vbo();

  RenderObject ro;
  ro.debugName = "PointNode";

  ro.vertexDecl = &vertexDecl_;
  ro.vertexBuffer = vbo_;

  for (unsigned int i = 0; i < _drawMode.getNumLayers(); ++i)
  {
    const DrawModes::DrawModeProperties* props = _drawMode.getLayer(i);

    if (props->primitive() == DrawModes::PRIMITIVE_POINT)
    {
      // reset renderobject
      ro.initFromState(&_state);
      ro.setMaterial(_mat);
      ro.setupShaderGenFromDrawmode(props);

      ro.priority = 0;
      ro.depthTest = true;
      ro.depthWrite = true;
      ro.depthFunc = GL_LESS;

      // use pointsize shader
      QString geomTemplate = ShaderProgGenerator::getShaderDir();
      geomTemplate += "PointSize/geometry.tpl";

      QString fragTemplate = ShaderProgGenerator::getShaderDir();
      fragTemplate += "PointSize/fragment.tpl";

      ro.shaderDesc.geometryTemplateFile = geomTemplate;
      ro.shaderDesc.fragmentTemplateFile = fragTemplate;

      // shader uniforms
      ro.setUniform("screenSize", Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));
      ro.setUniform("pointSize", _mat->pointSize());

      ro.glDrawArrays(GL_POINTS, 0, (GLsizei)points_.size());
      _renderer->addRenderObject(&ro);
    }
  }
}


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

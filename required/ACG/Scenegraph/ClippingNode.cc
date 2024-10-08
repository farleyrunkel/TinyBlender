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
//  CLASS ClippingNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "ClippingNode.hh"

#include <ACG/GL/IRenderer.hh>

#include <OpenMesh/Core/Utils/vector_cast.hh>

#include <QImage>


//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {

  
//== IMPLEMENTATION ========================================================== 


void
ClippingNode::set_plane(const Vec3f& _position, 
			const Vec3f& _normal, 
			float _eps)
{
  position_    = _position;
  normal_      = _normal; normal_.normalize();
  slice_width_ = _eps;
  

  // one clipping plane
  if (slice_width_ == 0.0)
  {
    plane0_[0] = normal_[0];
    plane0_[1] = normal_[1];
    plane0_[2] = normal_[2];
    plane0_[3] = -(normal_|position_);
  }


  // two planes -> slice
  else 
  {
    float d = -(normal_|position_);
    if (d > 0) { normal_ = -normal_; d = -d; }
    
    plane0_[0] = normal_[0];
    plane0_[1] = normal_[1];
    plane0_[2] = normal_[2];
    plane0_[3] = d + 0.5f*slice_width_;

    plane1_[0] = -normal_[0];
    plane1_[1] = -normal_[1];
    plane1_[2] = -normal_[2];
    plane1_[3] = -(d - 0.5f*slice_width_);
  }


  set_offset(offset_);
}


//----------------------------------------------------------------------------


void 
ClippingNode::set_offset(float _offset)
{
  offset_ = _offset;

  offset_plane0_[0] = plane0_[0];
  offset_plane0_[1] = plane0_[1];
  offset_plane0_[2] = plane0_[2];
  offset_plane0_[3] = plane0_[3] - offset_;

  offset_plane1_[0] = plane1_[0];
  offset_plane1_[1] = plane1_[1];
  offset_plane1_[2] = plane1_[2];
  offset_plane1_[3] = plane1_[3] + offset_;
}


//----------------------------------------------------------------------------
  

void ClippingNode::enter(GLState& /* _state */ , const DrawModes::DrawMode& /* _drawmode */ ) 
{
  // one clipping plane
  if (slice_width_ == 0.0)
  {
    glClipPlane(GL_CLIP_PLANE0, offset_plane0_);
    ACG::GLState::enable(GL_CLIP_PLANE0);
  }

  // two planes -> slice
  else
  {
    glClipPlane(GL_CLIP_PLANE0, offset_plane0_);
    ACG::GLState::enable(GL_CLIP_PLANE0);
    glClipPlane(GL_CLIP_PLANE1, offset_plane1_);
    ACG::GLState::enable(GL_CLIP_PLANE1);
  }
}


//----------------------------------------------------------------------------


void ClippingNode::leave(GLState& /* _state */ , const DrawModes::DrawMode& /* _drawmode */ )
{
  ACG::GLState::disable(GL_CLIP_PLANE0);
  if (slice_width_ > 0.0)
    ACG::GLState::disable(GL_CLIP_PLANE1);
}


//----------------------------------------------------------------------------


void ClippingNode::enter(IRenderer* _renderer, GLState& /* _state */, const DrawModes::DrawMode& /* _drawmode */)
{
  _renderer->addRenderObjectModifier(&mod_);
}


//----------------------------------------------------------------------------


void ClippingNode::leave(IRenderer* _renderer, GLState& /* _state */, const DrawModes::DrawMode& /* _drawmode */)
{
  _renderer->removeRenderObjectModifier(&mod_);
}

//=============================================================================

ClippingNode::ClippingShaderModifier ClippingNode::shaderMod1_(1);
ClippingNode::ClippingShaderModifier ClippingNode::shaderMod2_(2);

ClippingNode::ClippingShaderModifier::ClippingShaderModifier(int _numClipPlanes)
  : numClipPlanes_(_numClipPlanes)
{
  ShaderProgGenerator::registerModifier(this);
}


void ClippingNode::ClippingShaderModifier::modifyVertexIO(ShaderGenerator* _shader)
{
  for (int i = 0; i < numClipPlanes_; ++i)
    _shader->addUniform(QString("vec4 g_SlicePlane%1").arg(i));
}


void ClippingNode::ClippingShaderModifier::modifyVertexEndCode(QStringList* _code)
{
  for (int i = 0; i < numClipPlanes_; ++i)
    _code->push_back(QString("gl_ClipDistance[%1] = dot(inverse(g_mWV) * sg_vPosVS, g_SlicePlane%1);").arg(i));
}

//=============================================================================

ClippingNode::ClippingObjectModifier::ClippingObjectModifier(const ClippingNode* _node)
  : RenderObjectModifier("ClippingNode"), node_(_node)
{
}

//=============================================================================

void ClippingNode::ClippingObjectModifier::apply(RenderObject* _obj)
{
  // set clipping plane equation as uniform and set shader mod id to the object
  Vec4f p0 = OpenMesh::vector_cast<Vec4f, Vec4d>(node_->plane0());

  _obj->setUniform("g_SlicePlane0", p0);
  _obj->clipDistanceMask |= 0x1;

  unsigned int shaderModID = shaderMod1_.getID();

  if (node_->slice_width() > 0.0f)
  {
    Vec4f p1 = OpenMesh::vector_cast<Vec4f, Vec4d>(node_->plane1());
    _obj->setUniform("g_SlicePlane1", p1);
    _obj->clipDistanceMask |= 0x2;

    shaderModID = shaderMod2_.getID();
  }

  // set shader modifier
  _obj->shaderDesc.shaderMods.push_back(shaderModID);

  // enable clip distance pass through in geometry shader
  _obj->shaderDesc.clipDistanceMask = _obj->clipDistanceMask;
}

//=============================================================================


} // namespace SceneGraph
} // namespace ACG
//=============================================================================

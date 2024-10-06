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
//  CLASS BoundingBoxNode - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include "BoundingBoxNode.hh"
#include "SceneGraph.hh"
#include "../GL/GLPrimitives.hh"


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== IMPLEMENTATION ==========================================================

DrawModes::DrawMode
BoundingBoxNode::
availableDrawModes() const
{
  return DrawModes::WIREFRAME;
}

//----------------------------------------------------------------------------

BoundingBoxNode::BoundingBoxNode( BaseNode* _parent, std::string _name ) :
MaterialNode(_parent,
  _name,
  MaterialNode::BaseColor |
  MaterialNode::LineWidth),
  box_(0)
{
  drawMode(DrawModes::WIREFRAME);


  box_ = new GLLineBox();
}

//----------------------------------------------------------------------------
BoundingBoxNode::~BoundingBoxNode() {
  delete box_;
}

//----------------------------------------------------------------------------

void BoundingBoxNode::computeAABB( Vec3d* _outMin, Vec3d* _outMax )
{
  ACG::SceneGraph::BoundingBoxAction act;
  ACG::SceneGraph::traverse(this, act);

  if (_outMin)
    *_outMin = (ACG::Vec3d) act.bbMin();

  if (_outMax)
    *_outMax = (ACG::Vec3d) act.bbMax();
}

//----------------------------------------------------------------------------

void
BoundingBoxNode::
draw(GLState&  _state  , const DrawModes::DrawMode& _drawMode)
{
  if (_drawMode & DrawModes::WIREFRAME)
  {
    ACG::Vec3d bbmin;
    ACG::Vec3d bbmax;
    computeAABB(&bbmin, &bbmax);

    ACG::Vec3d bbcenter = (bbmin + bbmax) * 0.5;
    ACG::Vec3d bbsize = bbmax - bbmin;


    glPushAttrib (GL_ENABLE_BIT);

    ACG::GLState::disable(GL_LIGHTING);

    _state.push_modelview_matrix();

    _state.translate(bbcenter);
    _state.scale(bbsize[0], bbsize[1], bbsize[2]);

    glColor4f(0.0f,1.0f,0.0f,1.0f);

    box_->draw_primitive();

    _state.pop_modelview_matrix();
    glPopAttrib ();
  }
}

//----------------------------------------------------------------------------

void BoundingBoxNode::getRenderObjects(IRenderer* _renderer, GLState& _state , const DrawModes::DrawMode& _drawMode , const ACG::SceneGraph::Material* _mat)
{
  int dmlayerId = _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_WIREFRAME);

  if (dmlayerId >= 0)
  {
    ACG::Vec3d bbmin;
    ACG::Vec3d bbmax;
    computeAABB(&bbmin, &bbmax);

    ACG::Vec3d bbcenter = (bbmin + bbmax) * 0.5;
    ACG::Vec3d bbsize = bbmax - bbmin;

    // create renderobject
    RenderObject ro;
    ro.initFromState(&_state);
    ro.depthTest = true;
    ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;
    ro.emissive = Vec3f(0.0f, 1.0f, 0.0f);

    ro.modelview.translate(bbcenter);
    ro.modelview.scale(bbsize[0], bbsize[1], bbsize[2]);

    box_->addToRenderer_primitive(_renderer, &ro);
  }
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

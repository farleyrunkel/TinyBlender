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
//  CLASS StencilRefNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "StencilRefNode.hh"


//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================


StencilRefNode::StencilRefNode (BaseNode*          _parent,
                                const std::string& _name)
  : BaseNode(_parent, _name),
  reference_ (0)
{}

//----------------------------------------------------------------------------

StencilRefNode::StencilRefNode (BaseNode*          _parent,
                                BaseNode*          _child,
                                const std::string& _name)
  : BaseNode(_parent, _child, _name),
  reference_ (0)
{}

//----------------------------------------------------------------------------

void StencilRefNode::enter(GLState& /*_state*/, const DrawModes::DrawMode& /*_drawmode*/)
{
  if(visible()) {
    glPushAttrib (GL_STENCIL_BUFFER_BIT);
    glStencilFunc (GL_ALWAYS, reference_, ~0);
    glStencilOp (GL_KEEP, GL_KEEP, GL_REPLACE);
  }
}

//----------------------------------------------------------------------------

void StencilRefNode::leave(GLState& /*_state*/, const DrawModes::DrawMode& /*_drawmode*/)
{
  if(visible()) {
    glPopAttrib ();
  }
}

//----------------------------------------------------------------------------

void StencilRefNode::enterPick(GLState& /*_state*/ , PickTarget /*_target*/, const DrawModes::DrawMode& /*_drawMode*/ ) {

}

//----------------------------------------------------------------------------

void StencilRefNode::leavePick(GLState& /*_state*/, PickTarget /*_target*/, const DrawModes::DrawMode& /*_drawMode*/ ) {
}

void StencilRefNode::setReference(GLuint _ref)
{
  reference_ = _ref;
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================


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
//  CLASS TextNode - IMPLEMENTATION
//
//=============================================================================



//== INCLUDES =================================================================

#include <ACG/GL/acg_glew.hh>

#include "TextNode.hh"
#include "../Utils/ImageConversion.hh"


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================

void
TextNode::
enterCompat(GLState& _state, const DrawModes::DrawMode& /*_drawmode*/) {
  if (text_.empty())
    return;

  // store current gl state
  cullFaceEnabled_ = glIsEnabled(GL_CULL_FACE);
  texture2dEnabled_ = glIsEnabled(GL_TEXTURE_2D);
  blendEnabled_ = glIsEnabled(GL_BLEND);
  depthEnabled_ = glIsEnabled(GL_DEPTH_TEST);
  //alphaTest_ = glIsEnabled(GL_ALPHA_TEST);
  if (alphaTest_)
    ACG::GLState::getAlphaFunc(&alphaTestFunc_, &alphaTestValue_);

  glGetIntegerv(GL_BLEND_SRC, &blendSrc_);
  glGetIntegerv(GL_BLEND_DST, &blendDest_);

  // set texture and drawing states
  ACG::GLState::disable(GL_CULL_FACE);
  ACG::GLState::enable(GL_TEXTURE_2D);
  ACG::GLState::enable(GL_BLEND);
  ACG::GLState::enable(GL_ALPHA_TEST);
  ACG::GLState::alphaFunc(GL_GREATER, 0.2f);
  ACG::GLState::blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  if (alwaysOnTop_)
    ACG::GLState::disable(GL_DEPTH_TEST);
}



//----------------------------------------------------------------------------


void
TextNode::
leaveCompat(GLState& /*_state*/, const DrawModes::DrawMode& /*_drawmode*/) {
  if (text_.empty())
      return;

  // restore the GLState as it was when entering TextNode
  if (cullFaceEnabled_)
    ACG::GLState::enable(GL_CULL_FACE);
  if (!texture2dEnabled_)
    ACG::GLState::disable(GL_TEXTURE_2D);
  if (!blendEnabled_)
    ACG::GLState::disable(GL_BLEND);
  if (depthEnabled_)
    ACG::GLState::enable(GL_DEPTH_TEST);
  else
    ACG::GLState::disable(GL_DEPTH_TEST);
  if (!alphaTest_)
    ACG::GLState::disable(GL_ALPHA_TEST);
  else
    ACG::GLState::alphaFunc(alphaTestFunc_, alphaTestValue_);

  ACG::GLState::blendFunc(blendSrc_, blendDest_);
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

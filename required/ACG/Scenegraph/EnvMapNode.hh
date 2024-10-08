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
//  CLASS EnvMapNode
//
//=============================================================================


#ifndef ACG_ENVMAP_NODE_HH
#define ACG_ENVMAP_NODE_HH


//== INCLUDES =================================================================


#include "TextureNode.hh"


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================


/** \class EnvMapNode EnvMapNode.hh <ACG/Scenegraph/EnvMapNode.hh>
    Similar to TextureNode, but sets up a spherical environment map.
**/

class ACGDLLEXPORT EnvMapNode : public TextureNode
{
public:

  /// Default constructor. Applies all properties.
  EnvMapNode( BaseNode*           _parent = 0,
	      const std::string&  _name = "<EnvMapNode>",
              bool                _texture_repeat = true,
              GLint               _texture_filter = GL_LINEAR );

  /// set class name
  ACG_CLASSNAME(EnvMapNode);

  /// add env mapping
  DrawModes::DrawMode  availableDrawModes() const override;

  /// set texture
  void enter(GLState& _state, const DrawModes::DrawMode& _drawmode) override;

  /// restores original texture (or no-texture)
  void leave(GLState& _state, const DrawModes::DrawMode& _drawmode) override;

  /** \brief Do nothing in picking
   */
  void enterPick(GLState& _state, PickTarget _target, const DrawModes::DrawMode& _drawMode ) override;

  /** \brief Do nothing in picking
   */
  void leavePick(GLState& _state, PickTarget _target, const DrawModes::DrawMode& _drawMode ) override;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_ENVMAP_NODE_HH defined
//=============================================================================


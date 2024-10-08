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
//  CLASS Texture3DNode
//
//=============================================================================

#ifndef ACG_TEXTURE3DNODE_HH
#define ACG_TEXTURE3DNODE_HH

//=============================================================================

#include "BaseNode.hh"

#include <string>
#include <QImage>


//=============================================================================

namespace ACG {
namespace SceneGraph {

//=============================================================================


class ACGDLLEXPORT Texture3DNode : public BaseNode
{
public:

  Texture3DNode( BaseNode*           _parent = 0,
		 const std::string&  _name = "<Texture3DNode>" );
  virtual ~Texture3DNode();

  /// set class name
  ACG_CLASSNAME( Texture3DNode );

  void set_texture( unsigned int    _width,
		    unsigned int    _height,
		    unsigned int    _depth,
		    unsigned char * _data );

  /// set texture
  void enter( GLState& _state, const DrawModes::DrawMode& _drawmode ) override;

  /// restores original texture (or no-texture)
  void leave( GLState& _state, const DrawModes::DrawMode& _drawmode ) override;


  void set_texture_mode( GLenum _mode )
  { tex_mode_ = _mode; }


  void set_border_color( const Vec4f & _border_color )
  {
    border_color_ = _border_color;
  }

  void set_wrap_mode( GLenum _wrap_mode )
  {
    // possible modes are:
    // GL_CLAMP
    // GL_CLAMP_TO_EDGE
    // GL_CLAMP_TO_BORDER
    // GL_REPEAT

    wrap_mode_ = _wrap_mode;
  }

  void set_filter( GLenum _filter )
  {
    // possible modes are:
    // GL_NEAREST
    // GL_LINEAR

    filter_ = _filter;
  }


private:

  GLuint  texture_;
  GLenum  tex_mode_;
  Vec4f   border_color_;
  GLenum  wrap_mode_;
  GLenum  filter_;


};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_TEXTURE3DNODE_HH defined
//=============================================================================


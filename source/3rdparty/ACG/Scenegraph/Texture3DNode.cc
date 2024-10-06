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
//  CLASS Texture3DNode - IMPLEMENTATION
//
//=============================================================================

//=============================================================================


#include "Texture3DNode.hh"

//=============================================================================

namespace ACG {
namespace SceneGraph {

//=============================================================================


Texture3DNode::Texture3DNode( BaseNode          * _parent, 
			      const std::string & _name )
  : BaseNode( _parent, _name ),
    texture_( 0 ), 
    tex_mode_( GL_MODULATE ),
    border_color_( 0, 0, 0, 0 ),
    wrap_mode_( GL_CLAMP ),
    filter_( GL_LINEAR )
{}

    
//----------------------------------------------------------------------------


Texture3DNode::~Texture3DNode()
{
  if ( glIsTexture( texture_ ) )
    glDeleteTextures( 1, &texture_ );
}


//----------------------------------------------------------------------------
  

void
Texture3DNode::enter( GLState &    /*_state */, const DrawModes::DrawMode& /* _drawmode */ ) 
{
  if ( glIsTexture( texture_ ) )
  {
    ACG::GLState::bindTexture( GL_TEXTURE_3D, texture_ );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, tex_mode_);
    glTexParameterfv( GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, border_color_.data() );

    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, wrap_mode_ );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, wrap_mode_ );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, wrap_mode_ );

    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, filter_ );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, filter_ );

  }
}


//----------------------------------------------------------------------------


void
Texture3DNode::leave( GLState &    /* _state */ , const DrawModes::DrawMode& /* _drawmode */  )
{
  ACG::GLState::bindTexture( GL_TEXTURE_3D, 0 );
}


//----------------------------------------------------------------------------


void
Texture3DNode::set_texture( unsigned int    _width,
			    unsigned int    _height,
			    unsigned int    _depth,
			    unsigned char * _data )
{
  glTexImage3D( GL_PROXY_TEXTURE_3D, 0, GL_RGBA,
 		_width, _height, _depth, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL );

  GLint test_width;
  glGetTexLevelParameteriv( GL_PROXY_TEXTURE_3D, 0,
			    GL_TEXTURE_WIDTH, & test_width );

  GLint test_height;
  glGetTexLevelParameteriv( GL_PROXY_TEXTURE_3D, 0,
			    GL_TEXTURE_HEIGHT, & test_height );

  GLint test_depth;
  glGetTexLevelParameteriv( GL_PROXY_TEXTURE_3D, 0,
			    GL_TEXTURE_DEPTH, & test_depth );

  if ( ! test_width || ! test_height || ! test_depth ) 
  {
    std::cerr << "Can't load texture.\n";
    return;
  }

  glPixelStorei( GL_UNPACK_ROW_LENGTH,  0 );
  glPixelStorei( GL_UNPACK_SKIP_ROWS,   0 );
  glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0 );
  glPixelStorei( GL_UNPACK_ALIGNMENT,   1 );
  glPixelStorei( GL_PACK_ROW_LENGTH,    0 );
  glPixelStorei( GL_PACK_SKIP_ROWS,     0 );
  glPixelStorei( GL_PACK_SKIP_PIXELS,   0 );
  glPixelStorei( GL_PACK_ALIGNMENT,     1 );    


  if ( glIsTexture( texture_ ) )
    glDeleteTextures( 1, & texture_ );

  glGenTextures( 1, & texture_ );
  ACG::GLState::bindTexture( GL_TEXTURE_3D, texture_ );

  glTexImage3D( GL_TEXTURE_3D, 0, GL_RGBA,
 		_width, _height, _depth, 0, GL_RGBA, GL_UNSIGNED_BYTE, _data );

  ACG::GLState::bindTexture( GL_TEXTURE_3D, 0 );

}

 
//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

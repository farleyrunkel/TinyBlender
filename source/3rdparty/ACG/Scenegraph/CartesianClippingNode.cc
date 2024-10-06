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

#include "CartesianClippingNode.hh"

//=============================================================================

namespace ACG {
namespace SceneGraph {

//=============================================================================


CartesianClippingNode::CartesianClippingNode( BaseNode*           _parent,
					      const std::string&  _name )
  : BaseNode( _parent, _name )
{
  set_cursor( Vec3f( 0, 0, 0 ) );
  enabled_ = NONE;
}

//-----------------------------------------------------------------------------


void
CartesianClippingNode::enter( GLState & _state, const DrawModes::DrawMode& /* _drawmode */ )
{
  Vec3d eye = _state.eye();

  if ( is_enabled( XY_PLANE ) )
  {
    GLdouble xy_plane[4];  
    if ( eye[2] > 0 )
    {  
      xy_plane[0] = 0;
      xy_plane[1] = 0;
      xy_plane[2] = -1;
      xy_plane[3] = cursor_[2];
    }
    else
    {
      xy_plane[0] = 0;
      xy_plane[1] = 0;
      xy_plane[2] = 1;
      xy_plane[3] = -cursor_[2];
    }
    
    glClipPlane( GL_CLIP_PLANE0, xy_plane );
    ACG::GLState::enable( GL_CLIP_PLANE0 );
  }

  if ( is_enabled( YZ_PLANE ) )
  {
    GLdouble yz_plane[4];  
    if ( eye[0] > 0 )
    {  
      yz_plane[0] = -1;
      yz_plane[1] = 0;
      yz_plane[2] = 0;
      yz_plane[3] = cursor_[0];
    }
    else
    {
      yz_plane[0] = 1;
      yz_plane[1] = 0;
      yz_plane[2] = 0;
      yz_plane[3] = -cursor_[0];
    }
    
    glClipPlane( GL_CLIP_PLANE1, yz_plane );
    ACG::GLState::enable( GL_CLIP_PLANE1 );
  }

  if ( is_enabled( XZ_PLANE ) )
  {
    GLdouble xz_plane[4];  
    if ( eye[1] > 0 )
    {  
      xz_plane[0] = 0;
      xz_plane[1] = -1;
      xz_plane[2] = 0;
      xz_plane[3] = cursor_[1];
    }
    else
    {
      xz_plane[0] = 0;
      xz_plane[1] = 1;
      xz_plane[2] = 0;
      xz_plane[3] = -cursor_[1];
    }
    
    glClipPlane( GL_CLIP_PLANE2, xz_plane );
    ACG::GLState::enable( GL_CLIP_PLANE2 );
  }


  
}


//-----------------------------------------------------------------------------


void
CartesianClippingNode::leave( GLState & /* _state */ , const DrawModes::DrawMode& /* _drawmode */ )
{
  ACG::GLState::disable( GL_CLIP_PLANE0 );
  ACG::GLState::disable( GL_CLIP_PLANE1 );
  ACG::GLState::disable( GL_CLIP_PLANE2 );
}


//-----------------------------------------------------------------------------


void
CartesianClippingNode::set_cursor( const Vec3f & _cursor )
{
  cursor_ = _cursor;
  
}


//-----------------------------------------------------------------------------


const
Vec3f &
CartesianClippingNode::cursor() const
{
  return cursor_;
}


//-----------------------------------------------------------------------------


void
CartesianClippingNode::set_enabled( Plane _plane ) 
{
  enabled_ = _plane;
}


//-----------------------------------------------------------------------------


bool
CartesianClippingNode::is_enabled( Plane _plane ) const
{
  return ( enabled_ == _plane );
}


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

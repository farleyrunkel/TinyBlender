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

#include "SliceNode.hh"

//=============================================================================

namespace ACG {
namespace SceneGraph {

//=============================================================================


SliceNode::SliceNode( BaseNode    * _parent,
              const std::string&   _name   ) :
  BaseNode( _parent, _name ),
  view_frame_(false),
  enabled_(NONE)
{
  set_visible_box( Vec3f( 0, 0, 0 ),
		   Vec3f( 1, 1, 1 ) );

  set_texture_box( Vec3f( 0, 0, 0 ),
		   Vec3f( 1, 1, 1 ) );

  set_cursor( Vec3f( 0.5, 0.5, 0.5 ) );
}


//----------------------------------------------------------------------------


SliceNode::~SliceNode()
{
}


//----------------------------------------------------------------------------


void
SliceNode::view_frame( bool _view_frame )
{
  view_frame_ = _view_frame;
}


//----------------------------------------------------------------------------


void
SliceNode::boundingBox( Vec3d & _bbMin, Vec3d & _bbMax )
{
  _bbMin.minimize( visible_min_ );
  _bbMax.maximize( visible_max_ );
}


//----------------------------------------------------------------------------

  
DrawModes::DrawMode 
SliceNode::availableDrawModes() const
{
  return DrawModes::DrawMode(0);
  /*
  return ( DrawModes::POINTS              |
           DrawModes::WIREFRAME           |
           DrawModes::HIDDENLINE          |
           DrawModes::SOLID_FLAT_SHADED   |
           DrawModes::SOLID_SMOOTH_SHADED );
*/
}


//----------------------------------------------------------------------------


void
SliceNode::draw( GLState & /* _state */ , const DrawModes::DrawMode& /* _drawMode */ )
{
  
  if ( is_enabled( NONE ) )
    return;
  
  glPushAttrib(GL_LIGHTING_BIT);
  glPushAttrib(GL_ENABLE_BIT);
  glPushAttrib(GL_COLOR_BUFFER_BIT);
  
  if ( view_frame_ )
    draw_frame();
  
  ACG::GLState::enable( GL_LIGHTING );
  glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );

  ACG::GLState::enable( GL_TEXTURE_3D );    

  ACG::GLState::enable( GL_BLEND );
  ACG::GLState::blendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  draw_planes();

  ACG::GLState::disable( GL_BLEND );
  ACG::GLState::disable( GL_TEXTURE_3D );    
  ACG::GLState::disable( GL_LIGHTING );
  
  glPopAttrib();
  glPopAttrib();
  glPopAttrib();

}


//----------------------------------------------------------------------------


void
SliceNode::draw_frame() const
{
  ACG::GLState::disable( GL_BLEND );
  ACG::GLState::disable( GL_TEXTURE_3D );    
  ACG::GLState::disable( GL_LIGHTING );

  glBegin( GL_LINES );

  glColor3f( 1, 1, 1 );

  glVertex3f( visible_min_[0], visible_min_[1], visible_min_[2] );
  glVertex3f( visible_min_[0], visible_min_[1], visible_max_[2] );
  glVertex3f( visible_min_[0], visible_max_[1], visible_min_[2] );
  glVertex3f( visible_min_[0], visible_max_[1], visible_max_[2] );
  glVertex3f( visible_max_[0], visible_max_[1], visible_min_[2] );
  glVertex3f( visible_max_[0], visible_max_[1], visible_max_[2] );
  glVertex3f( visible_max_[0], visible_min_[1], visible_min_[2] );
  glVertex3f( visible_max_[0], visible_min_[1], visible_max_[2] );

  glVertex3f( visible_min_[0], visible_min_[1], visible_min_[2] );
  glVertex3f( visible_min_[0], visible_max_[1], visible_min_[2] );
  glVertex3f( visible_min_[0], visible_min_[1], visible_max_[2] );
  glVertex3f( visible_min_[0], visible_max_[1], visible_max_[2] );
  glVertex3f( visible_max_[0], visible_min_[1], visible_min_[2] );
  glVertex3f( visible_max_[0], visible_max_[1], visible_min_[2] );
  glVertex3f( visible_max_[0], visible_min_[1], visible_max_[2] );
  glVertex3f( visible_max_[0], visible_max_[1], visible_max_[2] );

  glVertex3f( visible_min_[0], visible_min_[1], visible_min_[2] );
  glVertex3f( visible_max_[0], visible_min_[1], visible_min_[2] );
  glVertex3f( visible_min_[0], visible_min_[1], visible_max_[2] );
  glVertex3f( visible_max_[0], visible_min_[1], visible_max_[2] );
  glVertex3f( visible_min_[0], visible_max_[1], visible_min_[2] );
  glVertex3f( visible_max_[0], visible_max_[1], visible_min_[2] );
  glVertex3f( visible_min_[0], visible_max_[1], visible_max_[2] );
  glVertex3f( visible_max_[0], visible_max_[1], visible_max_[2] );

  glEnd();

  glBegin( GL_LINES );
  
  glColor3f( 1, 1, 1 );

  if ( is_enabled( XY_PLANE ) )
  {
    glVertex3f( visible_min_[0], visible_min_[1], cursor_[2] );
    glVertex3f( visible_max_[0], visible_min_[1], cursor_[2] );
    glVertex3f( visible_max_[0], visible_min_[1], cursor_[2] );
    glVertex3f( visible_max_[0], visible_max_[1], cursor_[2] );
    glVertex3f( visible_max_[0], visible_max_[1], cursor_[2] );
    glVertex3f( visible_min_[0], visible_max_[1], cursor_[2] );
    glVertex3f( visible_min_[0], visible_max_[1], cursor_[2] );
    glVertex3f( visible_min_[0], visible_min_[1], cursor_[2] );
  }


  if ( is_enabled( YZ_PLANE ) )
  {
    glVertex3f( cursor_[0], visible_min_[1], visible_min_[2] );
    glVertex3f( cursor_[0], visible_min_[1], visible_max_[2] );
    glVertex3f( cursor_[0], visible_min_[1], visible_max_[2] );
    glVertex3f( cursor_[0], visible_max_[1], visible_max_[2] );
    glVertex3f( cursor_[0], visible_max_[1], visible_max_[2] );
    glVertex3f( cursor_[0], visible_max_[1], visible_min_[2] );
    glVertex3f( cursor_[0], visible_max_[1], visible_min_[2] );
    glVertex3f( cursor_[0], visible_min_[1], visible_min_[2] );
  }


  if ( is_enabled( XZ_PLANE ) )
  {
    glVertex3f( visible_min_[0], cursor_[1], visible_min_[2] );
    glVertex3f( visible_max_[0], cursor_[1], visible_min_[2] );
    glVertex3f( visible_max_[0], cursor_[1], visible_min_[2] );
    glVertex3f( visible_max_[0], cursor_[1], visible_max_[2] );
    glVertex3f( visible_max_[0], cursor_[1], visible_max_[2] );
    glVertex3f( visible_min_[0], cursor_[1], visible_max_[2] );
    glVertex3f( visible_min_[0], cursor_[1], visible_max_[2] );
    glVertex3f( visible_min_[0], cursor_[1], visible_min_[2] );
  }

  glEnd();
}


//----------------------------------------------------------------------------


void
SliceNode::draw_planes() const
{
  ACG::GLState::shadeModel( GL_FLAT );


  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

  ACG::GLState::depthRange(0.01, 1.0);

  Vec3f rel;
  Vec3f tcmin;
  Vec3f tcmax;

  for ( int i = 0; i < 3; ++i )
  {
    rel[i] = ( cursor_[i]-texture_min_[i] ) / ( texture_max_[i] - texture_min_[i] );
    tcmin[i] = ( visible_min_[i] - texture_min_[i] )
	     / ( texture_max_[i] - texture_min_[i] );
    tcmax[i] = ( visible_max_[i] - texture_min_[i] )
	     / ( texture_max_[i] - texture_min_[i] );
  }

  if ( cursor_[2] >= visible_min_[2] &&
       cursor_[2] <= visible_max_[2] &&
       is_enabled( XY_PLANE ) )
  {
    glBegin( GL_QUADS );
    
    glNormal3f( 0, 0, 1 );

    glTexCoord3f( tcmin[0], tcmin[1], rel[2] );
    glVertex3f( visible_min_[0], visible_min_[1], cursor_[2] );
    
    glTexCoord3f( tcmax[0], tcmin[1], rel[2] );
    glVertex3f( visible_max_[0], visible_min_[1], cursor_[2] );
    
    glTexCoord3f( tcmax[0], tcmax[1], rel[2] );
    glVertex3f( visible_max_[0], visible_max_[1], cursor_[2] );
    
    glTexCoord3f( tcmin[0], tcmax[1], rel[2] );
    glVertex3f( visible_min_[0], visible_max_[1], cursor_[2] );
    
    glEnd();
  }
  
  if ( cursor_[0] >= visible_min_[0] &&
       cursor_[0] <= visible_max_[0] &&
       is_enabled( YZ_PLANE ) )
  {
    glBegin( GL_QUADS );
    
    glNormal3f( -1, 0, 0 );

    glTexCoord3f( rel[0], tcmin[1], tcmin[2] );
    glVertex3f( cursor_[0], visible_min_[1], visible_min_[2] );

    glTexCoord3f( rel[0], tcmin[1], tcmax[2] );
    glVertex3f( cursor_[0], visible_min_[1], visible_max_[2] );
      
    glTexCoord3f( rel[0], tcmax[1], tcmax[2] );
    glVertex3f( cursor_[0], visible_max_[1], visible_max_[2] );
    
    glTexCoord3f( rel[0], tcmax[1], tcmin[2] );
    glVertex3f( cursor_[0], visible_max_[1], visible_min_[2] );
    
    glEnd();
  }

  if ( cursor_[1] >= visible_min_[1] &&
       cursor_[1] <= visible_max_[1] &&
       is_enabled( XZ_PLANE ) )
  {
    glBegin( GL_QUADS );
    
    glNormal3f( 0, -1, 0 );
    
    glTexCoord3f( tcmin[0], rel[1], tcmin[2] );
    glVertex3f( visible_min_[0], cursor_[1], visible_min_[2] );
    
    glTexCoord3f( tcmax[0], rel[1], tcmin[2] );
    glVertex3f( visible_max_[0], cursor_[1], visible_min_[2] );
    
    glTexCoord3f( tcmax[0], rel[1], tcmax[2] );
    glVertex3f( visible_max_[0], cursor_[1], visible_max_[2] );
    
    glTexCoord3f( tcmin[0], rel[1], tcmax[2] );
    glVertex3f( visible_min_[0], cursor_[1], visible_max_[2] );

    glEnd();
  }
  

  ACG::GLState::depthRange(0.0, 1.0);
}

//----------------------------------------------------------------------------


ACG::Vec3f
SliceNode::cursor() const
{
  return cursor_;
}


//----------------------------------------------------------------------------


void
SliceNode::set_cursor( const Vec3f & _cursor )
{
  cursor_ = _cursor;
}


//----------------------------------------------------------------------------


bool
SliceNode::is_enabled( Plane _plane ) const
{
  return enabled_ == _plane;
}


//----------------------------------------------------------------------------


void
SliceNode::set_enabled( Plane _plane )
{
  enabled_ = _plane;
}


//----------------------------------------------------------------------------


void
SliceNode::set_visible_box( const Vec3f & _box_min,
			    const Vec3f & _box_max )
{
  visible_min_ = _box_min;
  visible_max_ = _box_max;
}


//----------------------------------------------------------------------------


void
SliceNode::set_texture_box( const Vec3f & _box_min,
			    const Vec3f & _box_max )
{
  texture_min_ = _box_min;
  texture_max_ = _box_max;
}


//----------------------------------------------------------------------------


void
SliceNode::pick( GLState & _state, PickTarget /* _target */  )
{
  _state.pick_set_maximum (1);
  _state.pick_set_name (0);
  ACG::GLState::disable(GL_LIGHTING);
  glPushMatrix();
  draw_planes();
  glPopMatrix();
}


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

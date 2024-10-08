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
//  CLASS SliceNode
//
//=============================================================================

#ifndef ACG_SCLICENODE_HH
#define ACG_SCLICENODE_HH

//=============================================================================

#include "BaseNode.hh"
#include "DrawModes.hh"

#include <string>

//=============================================================================

namespace ACG {
namespace SceneGraph {

//=============================================================================


class ACGDLLEXPORT SliceNode : public BaseNode
{

public:

  enum Plane {
    NONE     = 1,
    XY_PLANE = 2,
    YZ_PLANE = 3,
    XZ_PLANE = 4
  };

  SliceNode( BaseNode          * _parent = 0,
             const std::string & _name   = "<SliceNode>" );

  virtual ~SliceNode();

  void set_visible_box( const Vec3f & _bmin,
                        const Vec3f & _bmax );
  void set_texture_box( const Vec3f & _bmin,
                        const Vec3f & _bmax );

  ACG_CLASSNAME( SliceNode );

  DrawModes::DrawMode availableDrawModes() const override;

  void boundingBox( Vec3d & _bbMin, Vec3d & _bbMax ) override;

  void draw( GLState& _state, const DrawModes::DrawMode& _drawMode ) override;
  void draw_frame() const;
  void draw_planes() const;

  void pick( GLState& _state, PickTarget _target ) override;

  bool is_enabled ( Plane _plane ) const;
  void set_enabled( Plane _plane );

  Vec3f cursor() const;
  void  set_cursor( const Vec3f & _cursor );

  void view_frame( bool _view_frame );

private:

  Vec3d        visible_min_;
  Vec3d        visible_max_;

  Vec3f        texture_min_;
  Vec3f        texture_max_;

  Vec3f        cursor_;

  bool         view_frame_;

  Plane enabled_;

};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_SCLICENODE_HH
//=============================================================================

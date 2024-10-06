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
//  CLASS TriangleNode
//
//=============================================================================

#ifndef ACG_TRIANGLENODE_HH
#define ACG_TRIANGLENODE_HH

//=============================================================================

#include "BaseNode.hh"

#include <vector>

//=============================================================================

namespace ACG {
namespace SceneGraph {
	      
//=============================================================================


class ACGDLLEXPORT TriangleNode : public BaseNode
{

public:

  typedef std::vector<ACG::Vec3f> PointVector;
   
  TriangleNode( BaseNode*    _parent=0,
        const std::string&   _name="<TriangleNode>" );
  virtual ~TriangleNode();
  
  ACG_CLASSNAME(TriangleNode);


  DrawModes::DrawMode availableDrawModes() const override;

  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;
  void pick(GLState& _state, PickTarget _target) override;

  void add_triangle( const ACG::Vec3f & _p0,
		     const ACG::Vec3f & _p1,
		     const ACG::Vec3f & _p2 )
  {
    point_.push_back( _p0 );
    point_.push_back( _p1 );
    point_.push_back( _p2 );

    ACG::Vec3f n = ( _p1 - _p0 ) % ( _p2 - _p1 );
    if ( n.norm() > 0.00001 )
      n.normalize();
    else
      n = Vec3f( 0, 0, 0 );
    normal_.push_back( n );
  }

  void clear()
  {
    point_.clear();
    normal_.clear();
  }

  size_t n_triangles() const { return point_.size() / 3; }

  void triangle( int _i,
		 ACG::Vec3f & _p0,
		 ACG::Vec3f & _p1,
		 ACG::Vec3f & _p2 ) const
  {
    _p0 = point_[ 3 * _i + 0 ];
    _p1 = point_[ 3 * _i + 1 ];
    _p2 = point_[ 3 * _i + 2 ];
  }

protected:

  enum FaceMode { FACE_NORMALS, FACE_COLORS, PER_VERTEX };

  void draw_vertices();
  void draw_faces();
  void draw_wireframe();

  PointVector point_;
  PointVector normal_;

};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_TRIMESHNODE_HH defined
//=============================================================================

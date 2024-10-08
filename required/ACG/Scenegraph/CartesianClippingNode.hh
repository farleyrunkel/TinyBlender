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
//  CLASS CartesianClippingNode
//
//=============================================================================

#ifndef ACG_CARTESIANCLIPPINGNODE_HH
#define ACG_CARTESIANCLIPPINGNODE_HH

//=============================================================================

#include "BaseNode.hh"
#include <string>
#include <fstream>

//=============================================================================

namespace ACG {
namespace SceneGraph {
  
//=============================================================================

  
class ACGDLLEXPORT CartesianClippingNode : public BaseNode
{
public:

  enum Plane {
    XY_PLANE = 0,
    YZ_PLANE = 1,
    XZ_PLANE = 2,
    NONE     = 3,
  };

  CartesianClippingNode( BaseNode*           _parent = 0,
			 const std::string&  _name = "<ClippingNode>" );

  /// Destructor.
  virtual ~CartesianClippingNode() {}

  /// set class name
  ACG_CLASSNAME(CartesianClippingNode);

  /// begin clipping
  void enter( GLState & _state, const DrawModes::DrawMode& _drawmode ) override;

  /// stop clipping
  void leave( GLState & _state, const DrawModes::DrawMode& _drawmode ) override;

  /// set position
  void set_cursor( const Vec3f & _pos );

  /// get position
  const Vec3f & cursor() const;

  void set_enabled( Plane _plane );
  bool is_enabled ( Plane _plane ) const;

private:

  Vec3f cursor_;
  Plane enabled_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_CARTESIANCLIPPINGNODE_HH defined
//=============================================================================


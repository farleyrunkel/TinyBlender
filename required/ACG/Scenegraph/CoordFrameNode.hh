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
//  CLASS CoordFrameNode
//
//=============================================================================


#ifndef ACG_COORDFRAMENODE_HH
#define ACG_COORDFRAMENODE_HH


//== INCLUDES =================================================================


#include "MaterialNode.hh"
#include <vector>


//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================

	      

/** \class CoordFrameNode CoordFrameNode.hh <ACG/.../CoordFrameNode.hh>

    Scenegraph Node.
  
    A more elaborate description follows.
**/
class ACGDLLEXPORT CoordFrameNode : public MaterialNode
{
public:
   
  /// Default constructor
  CoordFrameNode(BaseNode*  _parent=0,
	     const std::string&  _name="<CoordFrameNode>" );
 
  /// Destructor
  ~CoordFrameNode() {}


  /// implement className()
  ACG_CLASSNAME(CoordFrameNode);
  /// return available draw modes
  DrawModes::DrawMode  availableDrawModes() const override;
  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  /// drawing the primitive
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;


  /// update bounding box (compute in from BB of children)
  void update_bounding_box();
  /// set bounding box
  void set_bounding_box(const Vec3f& _bb_min, const Vec3f& _bb_max);
  /// get bounding box
  const Vec3d& bb_min() const { return bb_min_; }
  /// get bounding box
  const Vec3d& bb_max() const { return bb_max_; }


  /// get x-plane container
  const std::vector<float>&  x_planes() const { return x_planes_; }
  /// get y-plane container
  const std::vector<float>&  y_planes() const { return y_planes_; }
  /// get z-plane container
  const std::vector<float>&  z_planes() const { return z_planes_; }


  /// set x-plane container
  void set_x_planes(const std::vector<float>& _planes) { x_planes_ = _planes; }
  /// set y-plane container
  void set_y_planes(const std::vector<float>& _planes) { y_planes_ = _planes; }
  /// set z-plane container
  void set_z_planes(const std::vector<float>& _planes) { z_planes_ = _planes; }

  
  /// add (x == _x)-plane
  void add_x_plane(float _x) { x_planes_.push_back(_x); }
  /// add (y == _y)-plane
  void add_y_plane(float _y) { y_planes_.push_back(_y); }
  /// add (z == _z)-plane
  void add_z_plane(float _z) { z_planes_.push_back(_z); }


  /// del (x == _x)-plane
  void del_x_plane(float _x) { 
    x_planes_.erase(std::find(x_planes_.begin(), x_planes_.end(), _x));
  }
  /// del (y == _y)-plane
  void del_y_plane(float _y) { 
    y_planes_.erase(std::find(y_planes_.begin(), y_planes_.end(), _y));
  }
  /// del (z == _z)-plane
  void del_z_plane(float _z) { 
    z_planes_.erase(std::find(z_planes_.begin(), z_planes_.end(), _z));
  }



 

private:

  /// Copy constructor (not used)
  CoordFrameNode(const CoordFrameNode& _rhs);
  /// Assignment operator (not used)
  CoordFrameNode& operator=(const CoordFrameNode& _rhs);


  // extend of bounding box
  Vec3d bb_min_, bb_max_;

  // planes in x-, y-, z-direction
  std::vector<float>  x_planes_, y_planes_, z_planes_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_COORDFRAMENODE_HH defined
//=============================================================================


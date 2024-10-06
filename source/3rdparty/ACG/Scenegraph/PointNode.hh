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
//  CLASS PointNode
//
//=============================================================================


#ifndef ACG_POINTNODE_HH
#define ACG_POINTNODE_HH


//== INCLUDES =================================================================

#include "BaseNode.hh"
#include "DrawModes.hh"
#include <ACG/GL/VertexDeclaration.hh>
#include <vector>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== CLASS DEFINITION =========================================================

	      

/** \class PointNode PointNode.hh <ACG/Scenegraph/PointNode.hh>

    PointNode renders points and normals.
    
    These elements are internally stored in arrays and rendered using
    OpenGL vertex and normal arrays.
**/

class ACGDLLEXPORT PointNode : public BaseNode
{
public:
   
  // typedefs
  typedef std::vector<ACG::Vec3d>      PointVector;
  typedef PointVector::iterator        PointIter;
  typedef PointVector::const_iterator  ConstPointIter;
  typedef std::vector<ACG::Vec4f>      ColorVector;
  typedef ColorVector::iterator        ColorIter;
  typedef ColorVector::const_iterator  ConstColorIter;


  /// default constructor
  PointNode( BaseNode*         _parent=0,
         const std::string &  _name="<PointNode>" )
    : BaseNode(_parent, _name)
  {}
 
  /// destructor
  ~PointNode() {}

  /// static name of this class
  ACG_CLASSNAME(PointNode);

  /// return available draw modes
  DrawModes::DrawMode availableDrawModes() const override;

  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;

  /// draw points and normals
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /// draw points and normals via renderer plugin
  void getRenderObjects(IRenderer* _renderer, GLState&  _state , const DrawModes::DrawMode&  _drawMode , const Material* _mat) override;

  /// reserve mem for _np points and _nn normals
  void reserve(unsigned int _np, unsigned int _nn, unsigned int _nc) {
    points_.reserve(_np); normals_.reserve(_nn); colors_.reserve(_nc);
  }

  /// add point
  void add_point(const ACG::Vec3d& _p) { points_.push_back(_p); vbo_needs_update_ = true;}
  /// add normal
  void add_normal(const ACG::Vec3d& _n) { normals_.push_back(_n); vbo_needs_update_ = true;}
  /// add color
  void add_color(const ACG::Vec4f& _c) { colors_.push_back(_c); vbo_needs_update_ = true;}

  /// how many points?
  size_t n_points() const { return points_.size(); }

  /// clear points
  void clear_points() { points_.clear(); vbo_needs_update_ = true;}
  /// clear normals
  void clear_normals() { normals_.clear(); vbo_needs_update_ = true;}
  /// clear colors
  void clear_colors() { colors_.clear(); vbo_needs_update_ = true;}
  /// clear points and normals and colors
  void clear() { clear_points(); clear_normals(); clear_colors(); }

  /// get point container
  const PointVector& points() const { return points_; }
  /// get normal container
  const PointVector& normals() const { return normals_; }
  /// get color container
  const ColorVector& colors() const { return colors_; }


private:

  void update_vbo();
  PointVector  points_, normals_;
  ColorVector  colors_;

  GLuint vbo_ = 0;
  std::vector<float> vbo_data_;
  bool vbo_needs_update_ = true;

  VertexDeclaration vertexDecl_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_POINTNODE_HH defined
//=============================================================================

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
//  CLASS FastMesh
//
//=============================================================================


#ifndef ACG_OBJ_NODE_HH
#define ACG_OBJ_NODE_HH


//== INCLUDES =================================================================

#include "BaseNode.hh"
#include "DrawModes.hh"
#include "../Math/VectorT.hh"
#include <string>
#include <vector>


//== FORWARD DECLARATION ======================================================


//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================


class ACGDLLEXPORT OBJNode : public BaseNode
{

public:

  /// Default constructor.
  OBJNode( BaseNode*         _parent=0,
       const std::string&       _name="<OBJNode>" )
    : BaseNode(_parent, _name)
  {}


  /// destructor
  virtual ~OBJNode() {}


  ACG_CLASSNAME(OBJNode);


  /// return available draw modes
  DrawModes::DrawMode availableDrawModes() const override;

  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;

  /// drawing the primitive
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /// picking
  void pick(GLState& _state, PickTarget _target) override;


  struct Face
  {
    Face(int _i0=-1, int _i1=-1, int _i2=-1,
	 int _t0=-1, int _t1=-1, int _t2=-1)
      : i0(_i0), i1(_i1), i2(_i2),
        t0(_t0), t1(_t1), t2(_t2) {}
    int i0, i1, i2, t0, t1, t2;
  };


  /// number of vertices
  size_t n_vertices() const { return vertices_.size(); }
  /// number of faces
  size_t n_faces()    const { return faces_.size(); }
  /// number of normals
  size_t n_normals()  const { return normals_.size(); }
  /// number of texcoords
  size_t n_texcoords()  const { return texCoords_.size(); }


  /// clear the node
  void clear()
  {
    vertices_.clear();
    faces_.clear();
    normals_.clear();
    texCoords_.clear();
  }


  /// add vertex
  size_t add_vertex(const Vec3f& _v)
  {
    vertices_.push_back(_v);
    return vertices_.size()-1;
  }


  /// add triangle
  size_t add_face(const Face& _f)
  {
    faces_.push_back(_f);
    return faces_.size()-1;
  }

  /// add triangle
  size_t add_face(unsigned int _i0,
			unsigned int _i1,
			unsigned int _i2)
  {
    faces_.push_back(Face(_i0, _i1, _i2));
    return faces_.size()-1;
  }


  /// get i'th vertex
  Vec3f& vertex(unsigned int _i)
  {
    assert(_i < n_vertices());
    return vertices_[_i];
  }
  /// get i'th vertex
  const Vec3f& vertex(unsigned int _i) const
  {
    assert(_i < n_vertices());
    return vertices_[_i];
  }


  /// get i'th face
  const Face&  face(unsigned int _i) const
  {
    assert(_i < n_faces());
    return faces_[_i];
  }
  /// get i'th face
  Face&  face(unsigned int _i)
  {
    assert(_i < n_faces());
    return faces_[_i];
  }


  /// get i'th normal
  const Vec3f& normal(unsigned int _i) const
  {
    assert(_i < n_normals());
    return normals_[_i];
  }
  /// get i'th normal
  Vec3f& normal(unsigned int _i)
  {
    assert(_i < n_normals());
    return normals_[_i];
  }


  /// Read from file. Implemented using OpenMesh loader and OBJNodeExporter.
  bool read(const std::string& _filename);


  /// Update face normals. Call when geometry changes
  void update_face_normals();


private:

  void draw_obj() const;
  void draw_obj_tex() const;

  std::vector<Vec3f> vertices_;
  std::vector<Vec3f> normals_;
  std::vector<Vec2f> texCoords_;
  std::vector<Face>  faces_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_OBJ_NODE_HH
//=============================================================================

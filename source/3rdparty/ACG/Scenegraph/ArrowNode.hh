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
//  CLASS ArrowNode
//
//=============================================================================


#ifndef ACG_ARROWNODE_HH
#define ACG_ARROWNODE_HH


//== INCLUDES =================================================================

#include <ACG/Scenegraph/MaterialNode.hh>
#include <ACG/GL/globjects.hh>
#include <ACG/GL/VertexDeclaration.hh>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== CLASS DEFINITION =========================================================



/** \class ArrowNode ArrowNode.hh <ACG/Scenegraph/ArrowNode.hh>

    ArrowNode renders a set of arrows.
**/

class ACGDLLEXPORT ArrowNode : public MaterialNode
{
public:

  /// default constructor
  ArrowNode(BaseNode*    _parent = 0,
	          std::string  _name="<ArrowNode>" );

  /// destructor
  ~ArrowNode();


  /// static name of this class
  ACG_CLASSNAME(ArrowNode);

  /// return available draw modes
  DrawModes::DrawMode  availableDrawModes() const override;

  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  
  /// reserve mem for _n arrows
  void reserve(int _n);

  /** \brief Add an arrow to the node
  *
  * @param _start  Start position of the arrow
  * @param _dir    Direction of the arrow
  * @param _normal Normal vector on the top surface of the arrow. Gets orthonormalized to _dir internally. If 0 or collinear to _dir, a random normal is chosen. 
  * @param _scale  Uniform scaling factor of the arrow. This is also the length of the arrow.
  * @param _color  Arrow color
  * @return ID of the arrow
  */
  int addArrow(const Vec3f& _start, const Vec3f& _dir, const Vec3f& _normal = Vec3f(0.0f, 1.0f, 0.0f), const Vec3f& _scale = Vec3f(1.0f, 1.0f, 1.0f), const Vec4uc& _color = Vec4uc(82, 82, 82, 255));

  /** \brief Add an arrow to the node
  *
  * @param _start  Start position of the arrow
  * @param _dir    Direction of the arrow
  * @param _normal Normal vector on the top surface of the arrow. Gets orthonormalized to _dir internally. If 0 or collinear to _dir, a random normal is chosen.
  * @param _scale  Uniform scaling factor of the arrow. This is also the length of the arrow.
  * @param _color  Arrow color
  * @return id of the arrow
  */
  int addArrow(const Vec3f& _start, const Vec3f& _dir, const Vec3f& _normal, const Vec3f& _scale, const Vec4f& _color);

  /** \brief Add an arrow to the node
  *
  * @param _start  Start position of the arrow
  * @param _dir    Direction of the arrow
  * @param _normal Normal vector on the top surface of the arrow. Gets orthonormalized to _dir internally. If 0 or collinear to _dir, a random normal is chosen.
  * @param _scale  Uniform scaling factor of the arrow. This is also the length of the arrow.
  * @param _color  Arrow color
  * @return id of the arrow
  */
  int addArrow(const Vec3d& _start, const Vec3d& _dir, const Vec3d& _normal = Vec3d(0.0, 1.0, 0.0), const Vec3d& _scale = Vec3d(1.0, 1.0, 1.0), const Vec4uc& _color = Vec4uc(82, 82, 82, 255));

  /** \brief Add an arrow to the node
  *
  * @param _start  Start position of the arrow
  * @param _dir    Direction of the arrow
  * @param _normal Normal vector on the top surface of the arrow. Gets orthonormalized to _dir internally. If 0 or collinear to _dir, a random normal is chosen.
  * @param _scale  Uniform scaling factor of the arrow. This is also the length of the arrow.
  * @param _color  Arrow color
  * @return id of the arrow
  */
  int addArrow(const Vec3d& _start, const Vec3d& _dir, const Vec3d& _normal, const Vec3d& _scale, const Vec4f& _color);

  /** \brief Return the start position of an arrow
  *
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @return Start position of the arrow
  */
  Vec3f arrowStart(int _arrowID) const;

  /** \brief Set start point of an already added arrow
  *
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @param _start  Start position of the arrow
  */
  void arrowStart(int _arrowID, const Vec3f& _start);


  /** \brief Return the direction of an arrow
  *
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @return  Direction of the arrow
  */
  Vec3f arrowDir(int _arrowID) const;

  /** \brief Set direction of an already added arrow
  *
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @param _dir  Direction of the arrow
  */
  void arrowDir(int _arrowID, const Vec3f& _dir);


  /** \brief Return the normal of an arrow
  *
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @return  Normal of the arrow
  */
  Vec3f arrowNormal(int _arrowID) const;

  /** \brief Set normal of an already added arrow
  *
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @param _dir  Normal of the arrow
  */
  void arrowNormal(int _arrowID, const Vec3f& _normal);

  /** \brief Return the scale of an arrow
  *
  * Scaling vector: width, length, height
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @return Arrow scale
  */
  Vec3f arrowScale(int _arrowID) const;

  /** \brief Set the scale of an already added arrow
  *
  * Scaling vector: width, length, height
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @param _scale  Arrow scale
  */
  void arrowScale(int _arrowID, const Vec3f& _scale);

  /** \brief Return the color of an arrow
  *
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @return Arrow color
  */
  Vec4uc arrowColor(int _arrowID) const;

  /** \brief Set the color of an already added arrow
  *
  * @param _arrowID  ID of the arrow that was returned by addArrow()
  * @param _scale  Arrow color
  */
  void arrowColor(int _arrowID, const Vec4uc& _color);

  /// clear arrows
  void clear();

  /// number of arrows
  int n_arrows() const;

  /// draw arrows
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /** \brief Add the objects to the given renderer
   *
   * @param _renderer The renderer which will be used. Add your geometry into this class
   * @param _state    The current GL State when this object is called
   * @param _drawMode The active draw mode
   * @param _mat      Current material
   */
  void getRenderObjects(IRenderer* _renderer, GLState&  _state , const DrawModes::DrawMode&  _drawMode , const ACG::SceneGraph::Material* _mat) override;


private:
  void createArrowMesh();


  GLMatrixf computeWorldMatrix(int _arrow) const;
  GLMatrixf readWorldMatrix(int _arrow) const;
  void updateInstanceData();

  void updateInstanceBuffer();

  // arrow instances

  struct Arrow 
  {
    Vec3f start, dir;
    Vec3f normal;
    Vec3f scale; // width, length, height
    Vec4uc color;

    void orthonormalize();
  };

  std::vector<Arrow> arrows_;

  // arrow mesh:
  int               numVertices_;
  int               numIndices_;
  GeometryBuffer    vertexBuffer_;
  IndexBuffer       indexBuffer_;
  VertexDeclaration vertexDecl_;
  Vec3f             localArrowMin_;
  Vec3f             localArrowMax_;


  // instance data:
  //  float4x3 world transform
  //  ubyte4_norm color

  // dword offset of the instance data of an arrow ( modify this if more data gets appended )
  int instanceDataOffset(int _arrow) const {return _arrow * (4*3 + 3*3 +1);}

  // size in dwords of instance data ( modify this if more data gets appended )
  int instanceDataSize() const {return 4*3 + 3*3 + 1;}


  std::vector<float> instanceData_;

  // instance vbo
  GeometryBuffer    instanceBuffer_;
  VertexDeclaration vertexDeclInstanced_;


  bool invalidateInstanceData_;
  bool invalidateInstanceBuffer_;
  int supportsInstancing_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_ARROWNODE_HH defined
//=============================================================================


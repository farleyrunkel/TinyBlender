#pragma once
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
//  CLASS MeshNodeT
//
//=============================================================================

//== INCLUDES =================================================================



#include "BaseNode.hh"
#include <vector>
#include <ACG/GL/DrawMesh.hh>
#include <ACG/ShaderUtils/GLSLShader.hh>
#include <OpenMesh/Core/Mesh/DefaultTriMesh.hh>
#include <OpenMesh/Core/Mesh/DefaultPolyMesh.hh>

//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================


/** Non-generic base class intended to be inherited by MeshNodeT.
 */
class ACGDLLEXPORT MeshNodeBase : public BaseNode {
    protected:
        MeshNodeBase(BaseNode* _parent, std::string _name);

        void supplyDrawMesh(DrawMeshBase *drawMeshBase);

    public:
        void updatePolyEdgeBuf();

    protected:

      DrawMeshBase *drawMeshBase_;

      // poly edge buffer used for wireframe/hiddenline rendering with barycentric interpolation in geometry shader
      GLuint polyEdgeBuf_;

      // size in bytes of the poly edge buffer
      int polyEdgeBufSize_;

      // texture object for polyEdgeBuf
      GLuint polyEdgeBufTex_;
};



/** \class MeshNodeT MeshNodeT.hh <ACG/Scenegraph/MeshNodeT.hh>

    This node draws a mesh using triangle strips.
*/

template <class Mesh>
class MeshNodeT : public MeshNodeBase
{
public:
  ACG_CLASSNAME(MeshNode);

  /** \brief Default constructor
  *     
  * The constructor needs a mesh on which this node will work.
  */
  MeshNodeT(      Mesh&         _mesh,
                  BaseNode*     _parent=0,
            const std::string&  _name="<MeshNode>" );

  /// Destructor
  virtual ~MeshNodeT();
  
  
  /** \brief the geometry of the mesh has changed
  *
  * call this function if you changed the geometry of the mesh.
  * All buffers related to the geometry will be updated.
  */
  void update_geometry();
  
  /** \brief the topology of the mesh has changed
  *
  * call this function if you changed the topology of the mesh.
  * All buffers related to the topology will be updated.
  *
  * this also triggers an update for the colors
  */
  void update_topology();  

  /** \brief the colors of the mesh have changed
  *
  * call this function if you changed the colors of the mesh.
  * All buffers related to the color will be updated.
  *
  * if you also updated the topology, the color is updated automatically
  */
  void update_color();
  
  /** \brief force an texture update
   *
   * This function has to be called, when the textures have changed
   */
  void update_textures();

private:

  /** Typedefs of the mesh representation
  * 
  * These typedefs are used to specifiy and convert all input to float for rendering
  */
  typedef typename Mesh::Point         Point;
  typedef typename Point::value_type   PointScalar;
  typedef typename Mesh::Normal        Normal;
  typedef typename Normal::value_type  NormalScalar;
  typedef typename Mesh::Color         Color;
  typedef typename Color::value_type   ColorScalar;
  
//===========================================================================
/** @name Mesh Handling
* @{ */
//===========================================================================  
  
public:
  
  /** \brief get the internal mesh
  */
  const Mesh& mesh() const { return mesh_; }
  
private:  
  /// The mesh this node works on
  Mesh& mesh_;  
  
/** @} */  
  
//===========================================================================
/** @name Draw-mesh handling
* @{ */
//===========================================================================  
private:
  DrawMeshT<Mesh>* drawMesh_;

/** @} */  



//===========================================================================
/** @name Bounding Box
* @{ */
//===========================================================================  

public:  
  
  /** \brief Current bounding box
  *
  * This function returns the bounding box of the node.
  */
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  
private:
  
  /// bounding box lower left corner
  Vec3d bbMin_;
  
  /// bounding box upper right corner
  Vec3d bbMax_;
  
/** @} */
  
//===========================================================================
/** @name Normal Buffer
* @{ */
//===========================================================================

public:
  /// Returns if the normal array is currently activated
  bool normalsEnabled() { return enableNormals_; };
  
  /// Enable or disable the use of the normal array
  void enableNormals(bool _enable) { enableNormals_ = _enable; };

private:
  
  /// Flag if normals should be used
  bool enableNormals_;

/** @} */
  
//===========================================================================
/** @name Color buffer
* @{ */
//===========================================================================  
public:
  /// Returns if the color array is currently activated
  bool colorsEnabled() { return enableColors_; };
  
  /// Enable or disable the use of color array
  void enableColors(bool _enable) { enableColors_ = _enable; };

private:
  
  bool enableColors_;

  /** @} */
  
//===========================================================================
/** @name Array control functions
* @{ */
//===========================================================================

public:
  
  /** \brief enable/disable vertex arrays according to the bits in _arrays
  *
  * Use this function to enable or disable the appropriate array for rendering. Currently
  * the arrays in ArrayType are supported
  */
  void enable_arrays(unsigned int _arrays);    

private:
  
  /// Enum controlling which array should be used for rendering
  enum ArrayType
  {
    NONE                             = 0,
    PER_EDGE_VERTEX_ARRAY            = 1,
    PER_EDGE_COLOR_ARRAY             = 2,
    PER_HALFEDGE_VERTEX_ARRAY        = 4,
    PER_HALFEDGE_COLOR_ARRAY         = 8
  };
  
  /// which arrays are currently enabled?
  unsigned int enabled_arrays_;
  

//===========================================================================
/** @name Rendering functions
* @{ */
//===========================================================================
  
public:
  /** \brief Draws the object
  *
  */
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /** \brief Draws the object deferred
  *
  */
  void getRenderObjects(IRenderer* _renderer, GLState& _state, const DrawModes::DrawMode& _drawMode, const Material* _mat) override;


  /** \brief Get DrawMesh instance
  */
  DrawMeshT<Mesh>* getDrawMesh();


  /** \brief return available draw modes 
  *
  * The drawmodes are constructed based on the mesh properties and the hardware capabilities
  * of the system.
  */
  ACG::SceneGraph::DrawModes::DrawMode  availableDrawModes() const override;
  
private:
  
  /** \brief draws all vertices of the mesh
  *
  */
  inline void draw_vertices();

  inline void add_point_RenderObjects(IRenderer* _renderer, const RenderObject* _baseObj);

  /** \brief draws all edges of the mesh
  *
  */
  inline void draw_lines();

  /** \brief draws all halfedges of the mesh
  *
  */
  inline void draw_halfedges();

  
  /** \brief draws all faces of the mesh 
  *
  */
  void draw_faces();

  void add_face_RenderObjects(IRenderer* _renderer, const RenderObject* _baseObj, bool _nonindexed = false);
  
private:
  
  VertexDeclaration halfedgeDecl;
/** @} */

//===========================================================================
/** @name general picking functions
* @{ */
//===========================================================================
public:
  /** \brief Draws the object in picking mode
  *
  */
  void pick(GLState& _state, PickTarget _target) override;

/** @} */

//===========================================================================
/** @name vertex picking functions
* @{ */
//===========================================================================  
  
private:

  /** \brief Renders picking for vertices
  * _front: Only render front vertices (not occluded by geometry)
  */
  void pick_vertices(GLState& _state, bool _front = false);
  
  /// Flag indicating if the vertex picking has to be updated
  bool updateVertexPicking_;

  /// Index of the first vertex in vertexpicking
  size_t vertexPickingBaseIndex_;

/** @} */
  

//===========================================================================
/** @name edge picking functions
* @{ */
//=========================================================================== 


private:
  /** \brief Renders picking for edges
  * _front: Only render front edges (not occluded by geometry)
  */
  void pick_edges(GLState& _state, bool _front = false);  
  
  /// Flag indicating if the edge picking has to be updated
  bool updateEdgePicking_;
  
  /// Index of the first edge in edgepicking
  size_t edgePickingBaseIndex_;


/** @} */

//===========================================================================
/** @name face picking functions
* @{ */
//=========================================================================== 


private:  
  /** \brief Renders picking for faces
  * _front: Only render front faces (not occluded by geometry)
  */
  void pick_faces(GLState& _state);
  
  /// Flag indicating if the edge picking has to be updated
  bool updateFacePicking_;
  
  /// Index of the first face in facepicking
  size_t facePickingBaseIndex_;


/** @} */

//===========================================================================
/** @name anything picking functions
* @{ */
//=========================================================================== 
  
private:  
  
  /** \brief Renders picking for all primitives
  *
  */
  void pick_any(GLState& _state);  
  
  /// Flag indicating if the any picking has to be updated
  bool updateAnyPicking_;
  
  /// Index of the first face in anypicking
  size_t anyPickingBaseIndex_;
  
/** @} */
  

//===========================================================================
/** @name Texture handling
* @{ */
//=========================================================================== 
public:
  /** \brief set the name of the property used for texture index specification
  *
  * The given property name will define a texture index. This index is used to make
  * a lookup in the texture correspondence map containing for each index the gluint
  * for the texture to be used. A zero in the property means, that no texture will be bound for the 
  * face.
  * If you define a non existing name here, texture switching will be disabled and it
  * is assumed that a texture is bound already.
  *
  *\todo Remove the external texture loading and do it here.
  *
  */
  void setIndexPropertyName( std::string _indexPropertyName );
  
  /// \brief Get current texture index property name
  const std::string& indexPropertyName() const;
  
  
  /** \brief Set the name of the per face texture coordinate property
  *
  * Set this property for per face per vertex texture coordinates. Additionally you have to set
  * the IndexPropertyName to make texturing with multiple textures work.
  */
  void setHalfedgeTextcoordPropertyName( std::string _halfedgeTextcoordPropertyName );
  
  public:
    
    /** \brief Setup a mapping between internal texture ids on the mesh and the ids for the loaded textures in opengl
    *
    * @param _map maps between an int index stored in the Mesh describing which texture to use for a face,
    *             and the GluInt name of the texture bound by the TextureNode. \n
    *             If such a map is not available ( =0 ), assume TextureNode has already bound a texture
    *             And render without switching textures
    */
    void setTextureMap( std::map< int, GLuint>* _map){ textureMap_ = _map; };
  
private:
  
  /// This flag indicates if we have a per Face texture index property
  bool perFaceTextureIndexAvailable_;
  
  /// Mapping of mesh face texture indices to gltexture id ( has to be provided externally )
  std::map< int, GLuint>* textureMap_;

/** @} */
  
  /// \todo Remove all these functions afterwards!
  
public:  
  void set_property_map( std::map< int, std::string>* _map){ };  
    

  /** \brief measures the size in bytes of allocated memory
  */
  unsigned int getMemoryUsage();
  
private:
  bool draw_with_offset_;

public:
  void set_offset(bool enable) { draw_with_offset_ = enable; }
};

// defined in MeshNode2T_impl.cc:
extern template class MeshNodeT<::OpenMesh::TriMesh>;
extern template class MeshNodeT<::OpenMesh::PolyMesh>;

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

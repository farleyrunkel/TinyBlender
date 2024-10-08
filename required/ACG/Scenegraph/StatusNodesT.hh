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
//  Status Nodes
//
//=============================================================================


#ifndef ACG_STATUS_NODES_HH
#define ACG_STATUS_NODES_HH


//== INCLUDES =================================================================


#include <OpenMesh/Core/Mesh/Attributes.hh>

#include "MaterialNode.hh"
#include "DrawModes.hh"

#include <ACG/GL/VertexDeclaration.hh>
#include <ACG/GL/IRenderer.hh>
#include <ACG/GL/DrawMesh.hh>

#include <vector>


//== NAMESPACES ===============================================================


template<class Mod>
class StatusNodes_ModTraits {
    public:
        enum {
            StaticUsage = true
        };
};

namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================

/**
 * @brief The StatusNodesBase class extends StatusNodesT with a halfEdge vbo
 * for coreProfile rendering support.
 *
 * StatusNodesT is supposed to derive from this class.
 */
class ACGDLLEXPORT StatusNodesBase
{
public:
  StatusNodesBase();
  virtual ~StatusNodesBase();

protected:
  //index buffer objects ,previously direct ram access was used in compat
  //now use ibos / vbos on both compat and core profile.
  GLuint heVBO_, eIBO_, fIBO_, vIBO_, pIBO_;

  void updateIBOData(GLuint& bufferName_, size_t numberOfElements_, size_t sizeOfElements_, void* data_);
  void updateHEVBOPoints(size_t numberOfElements_, size_t sizeOfElements_, void* data_);

private:
  GLint prevBuffer;
  void createHEVBO();
  void createIBO(GLuint& _name);
  void bindHEVBO();
  void unbindHEVBO();
  void bindIBO(GLuint& _name);
  void unbindIBO();
};

template<class Mesh, class Mod, const bool StaticUsage> class StatusNodeBaseT;

template<class Mesh, class Mod>

class StatusNodeBaseT<Mesh, Mod, true> : public MaterialNode {
    public:
        StatusNodeBaseT(BaseNode* _parent, const std::string&  _name) :
            MaterialNode(_parent, _name) {}

        virtual ~StatusNodeBaseT() {}

    protected:
        bool is_vertex_selected(
                const Mesh &mesh, typename Mesh::VertexHandle vh) {
            return Mod::is_vertex_selected(mesh, vh);
        }

        bool is_halfedge_selected(
                const Mesh &mesh, typename Mesh::HalfedgeHandle heh) {
            return Mod::is_halfedge_selected(mesh, heh);
        }

        bool is_edge_selected(const Mesh &mesh, typename Mesh::EdgeHandle eh) {
            return Mod::is_edge_selected(mesh, eh);
        }

        bool is_face_selected(const Mesh &mesh, typename Mesh::FaceHandle fh) {
            return Mod::is_face_selected(mesh, fh);
        }
};

template<class Mesh, class Mod>
class StatusNodeBaseT<Mesh, Mod, false> : public MaterialNode {

    public:
        StatusNodeBaseT(BaseNode* _parent, const std::string&  _name) :
            MaterialNode(_parent, _name), modInstance(0) {}

        virtual ~StatusNodeBaseT() {
            delete modInstance;
        }

        /**
         * Provide the actual instance of the Mod. Transfers ownership.
         *
         * @param mod A Mod instance. Has to be created with new. Ownership
         * will be assumed and it will be deleted with delete.
         */
        void provideModInstance(Mod *mod) {
            delete modInstance;
            modInstance = mod;
        }

    protected:
        bool is_vertex_selected(
                const Mesh &mesh, typename Mesh::VertexHandle vh) {
            assert(modInstance);
            return modInstance->is_vertex_selected(mesh, vh);
        }

        bool is_halfedge_selected(
                const Mesh &mesh, typename Mesh::HalfedgeHandle heh) {
            assert(modInstance);
            return modInstance->is_halfedge_selected(mesh, heh);
        }

        bool is_edge_selected(const Mesh &mesh, typename Mesh::EdgeHandle eh) {
            assert(modInstance);
            return modInstance->is_edge_selected(mesh, eh);
        }

        bool is_face_selected(const Mesh &mesh, typename Mesh::FaceHandle fh) {
            assert(modInstance);
            return modInstance->is_face_selected(mesh, fh);
        }

    protected:
        Mod *modInstance;
};


/** \class StatusNodeT StatusNodesT.hh <ACG/Scenegraph/StatusNodesT.hh>
 *
 *     Renders Status flags of Mesh Vertices/Faces/Edges
 *
 *             **/
template <class Mesh, class Mod>
class StatusNodeT :
        public StatusNodeBaseT<Mesh, Mod, ::StatusNodes_ModTraits<Mod>::StaticUsage>,
        StatusNodesBase
{
public:
  typedef StatusNodeBaseT<Mesh, Mod, ::StatusNodes_ModTraits<Mod>::StaticUsage> BaseClass;
  typedef Mod ModType;

  /// constructor
  StatusNodeT( const Mesh&         _mesh,
               BaseNode*           _parent = 0,
               const std::string&  _name   = "<StatusNode>" );

  /// destructor
  virtual ~StatusNodeT() {}

  ACG_CLASSNAME(StatusNode);


  /** \brief set geometry invalid, topology and selection is kept
   */
  void updateGeometry();

  /** \brief set topology invalid (updates everything)
    */
  void updateTopology();

  /** \brief set selection invalid (Only selection changed, rest is kept)
    */
  void updateSelection();

  /** \brief Set drawmesh
   *
   * Selections are then rendered with gpu buffers gathered fro the meshnode for improved performance
   *
   * @param _drawmesh Pointer to the drawmesh for which this status node will work
  */
  void setDrawMesh(DrawMeshT<Mesh>* _drawmesh);


  /** \brief support for shader-pipeline
  @param _renderer Render-Interface, collector for Renderobjects
  @param _state current OpenGL state
  @param _drawMode active Drawmode
  @param _mat active Material
  */
  void getRenderObjects(IRenderer* _renderer, GLState&  _state, const DrawModes::DrawMode&  _drawMode, const class Material* _mat) override;


  DrawModes::DrawMode  availableDrawModes() const override;
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;
  void pick(GLState& /* _state */ , PickTarget /* _target */ ) override {}


private:

  /** build/update cache of active vertices/edges/faces. This function
      automatically enables caching.
  */
  void update_cache();

  typedef typename Mesh::Face           Face;
  typedef typename Mesh::Vertex         Vertex;
  typedef typename Mesh::Halfedge       Halfedge;
  typedef typename Mesh::Edge           Edge;
  typedef typename Mesh::FaceHandle     FaceHandle;
  typedef typename Mesh::HalfedgeHandle HalfedgeHandle;

  typedef typename Mesh::Point       Point;
  typedef typename Mesh::Normal      Normal;

  void draw_points();
  void draw_edges();
  void draw_halfedges();
  void draw_faces(bool _per_vertex);

  Point halfedge_point(const HalfedgeHandle _heh);


private:

  const Mesh&                mesh_;
  DrawMeshT<Mesh>*           drawMesh_;

  //indices used for indexed rendering
  std::vector<unsigned int>  v_cache_, e_cache_, f_cache_, poly_cache_;
  std::vector<FaceHandle>    fh_cache_;

  //halfedges are rendered directly from ram in compat profile
  std::vector<Point>  he_points_;
  std::vector<Normal> he_normals_;

  // bounding box
  Vec3d bbMin_;
  Vec3d bbMax_;

  /// State variables
  bool invalidGeometry_;

  bool vertexIndexInvalid_;
  bool halfedgeCacheInvalid_;
  bool edgeIndexInvalid_;
  bool faceIndexInvalid_;


  // vertex-formats for new renderer
  VertexDeclaration pointVertexDecl_;
  VertexDeclaration halfedgeVertexDecl_;
};



//== CLASS DEFINITION =========================================================


template <class Mesh, unsigned int Bit>
struct StatusModT
{
  static bool is_vertex_selected(const Mesh& _mesh,
                                 typename Mesh::VertexHandle _vh)
  {
    return _mesh.status(_vh).is_bit_set(Bit);
  }

  static bool is_edge_selected(const Mesh& _mesh,
                               typename Mesh::EdgeHandle _eh)
  {
    return _mesh.status(_eh).is_bit_set(Bit);
  }

  static bool is_halfedge_selected(const Mesh& _mesh,
				   typename Mesh::HalfedgeHandle _heh)
  {
    return _mesh.status(_heh).is_bit_set(Bit);
  }

  static bool is_face_selected(const Mesh& _mesh,
                               typename Mesh::FaceHandle _fh)
  {
    return _mesh.status(_fh).is_bit_set(Bit);
  }
};



//== CLASS DEFINITION =========================================================


template <class Mesh>
struct SelectionModT
  : public StatusModT<Mesh, OpenMesh::Attributes::SELECTED>
{};



/** \class SelectionNodeT StatusNodesT.hh <ACG/Scenegraph/StatusNodesT.hh>
 *
 *     Renders the Selection status of Mesh Vertices/Faces/Edges
 *
 *             **/
template <class Mesh>
class SelectionNodeT
  : virtual public StatusNodeT<Mesh, SelectionModT<Mesh> >
{
public:

  /** \brief Constructor
   * @param _mesh reference to mesh with status property
   * @param _parent Parent node in the scenegraph
   * @param _name Name of the Node
   */
  SelectionNodeT( const Mesh&         _mesh,
                  BaseNode*           _parent = 0,
                  const std::string&  _name   = "<SelectionNode>" )
    : StatusNodeT<Mesh, SelectionModT<Mesh> > (_mesh, _parent, _name)
  {}
};


//== CLASS DEFINITION =========================================================


template <class Mesh>
struct LockModT
  : public StatusModT<Mesh, OpenMesh::Attributes::LOCKED>
{};


template <class Mesh>
class LockNodeT : public StatusNodeT<Mesh, LockModT<Mesh> >
{
public:

  LockNodeT( const Mesh&         _mesh,
             BaseNode*           _parent = 0,
             const std::string&  _name   = "<LockNode>" )
    : StatusNodeT<Mesh, LockModT<Mesh> > (_mesh, _parent, _name)
  {}
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(ACG_STATUS_NODES_C)
#define ACG_STATUS_NODES_TEMPLATES
#include "StatusNodesT_impl.hh"
#endif
//=============================================================================
#endif // ACG_STATUS_NODES_HH defined
//=============================================================================


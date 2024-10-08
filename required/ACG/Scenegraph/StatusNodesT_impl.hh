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
//  CLASS StatusNodeT - IMPLEMENTATION
//
//=============================================================================


#define ACG_STATUS_NODES_C


//== INCLUDES =================================================================


#include "StatusNodesT.hh"
#include "../GL/gl.hh"

//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================

template <class Mesh, class Mod>
StatusNodeT<Mesh, Mod>::
StatusNodeT( const Mesh&         _mesh,
             BaseNode*           _parent,
             const std::string&  _name )
  : BaseClass(_parent, _name), mesh_(_mesh),
  drawMesh_(NULL),
  bbMin_(FLT_MAX,  FLT_MAX,  FLT_MAX),
  bbMax_(-FLT_MAX, -FLT_MAX, -FLT_MAX),
  invalidGeometry_(true),
  vertexIndexInvalid_(true),
  halfedgeCacheInvalid_(true),
  edgeIndexInvalid_(true),
  faceIndexInvalid_(true)
{
  this->set_line_width(3);
  this->set_point_size(5);
}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
void
StatusNodeT<Mesh, Mod>::
boundingBox(Vec3d& _bbMin, Vec3d& _bbMax)
{
  _bbMin.minimize(bbMin_);
  _bbMax.maximize(bbMax_);
}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
DrawModes::DrawMode
StatusNodeT<Mesh, Mod>::
availableDrawModes() const
{
  return (DrawModes::POINTS |
	  DrawModes::WIREFRAME |
	  DrawModes::EDGES |
	  DrawModes::SOLID_FLAT_SHADED);
}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
void
StatusNodeT<Mesh, Mod>::
update_cache()
{
  if (invalidGeometry_) {

    // Vertices, Edges and Faces use the mesh geometry.
    // However Halfedge selection buffers are computed.
    // Therefore we have to invalidate them when
    // the geometry changes to force a recomputation.
    halfedgeCacheInvalid_ = true;

    bbMin_ = Vec3d(FLT_MAX, FLT_MAX, FLT_MAX);
    bbMax_ = Vec3d(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    typename Mesh::ConstVertexIter v_it(mesh_.vertices_sbegin()), v_end(mesh_.vertices_end());

    for (; v_it != v_end; ++v_it) {
      bbMin_.minimize(mesh_.point(*v_it));
      bbMax_.maximize(mesh_.point(*v_it));
    }

    invalidGeometry_ = false;

  }

  /*
   * Hack: Force rebuild of buffers so that mapVertexToVBOIndex call doesn't SEGFAULT.
   */
  if (vertexIndexInvalid_ || edgeIndexInvalid_ || halfedgeCacheInvalid_ || faceIndexInvalid_)
    if (drawMesh_)
      drawMesh_->getVBO();

  // Update the indices for selected vertices
  if (vertexIndexInvalid_) {
    v_cache_.clear();
    v_cache_.reserve(mesh_.n_vertices()/4);

    typename Mesh::ConstVertexIter v_it(mesh_.vertices_sbegin()), v_begin(mesh_.vertices_sbegin()), v_end(mesh_.vertices_end());
    for (v_it = v_begin; v_it != v_end; ++v_it) {
      if (this->is_vertex_selected(mesh_, *v_it)) {
        unsigned int vertexIndex = v_it->idx();
        // use correct index for vbo, if available
        v_cache_.push_back(drawMesh_ ? drawMesh_->mapVertexToVBOIndex(vertexIndex) : vertexIndex);
      }
    }
    std::vector<unsigned int>(v_cache_.begin(), v_cache_.end()).swap(v_cache_);
    if(v_cache_.size() > 0)
        updateIBOData(vIBO_, v_cache_.size(), sizeof(v_cache_[0]), v_cache_.data());
    vertexIndexInvalid_ = false;
  }

  // Update index list of selected edges
  if (edgeIndexInvalid_) {

    e_cache_.clear();
    e_cache_.reserve(mesh_.n_edges()/4);

    typename Mesh::ConstEdgeIter e_it(mesh_.edges_sbegin()), e_begin(mesh_.edges_sbegin()), e_end(mesh_.edges_end());
    for (e_it = e_begin; e_it != e_end; ++e_it) {
      if (this->is_edge_selected(mesh_, *e_it)) {
        typename Mesh::VertexHandle vh = mesh_.to_vertex_handle(mesh_.halfedge_handle(*e_it, 0));
        unsigned int vidx = vh.idx();

        e_cache_.push_back(drawMesh_ ? drawMesh_->mapVertexToVBOIndex(vidx) : vidx);

        vh = mesh_.to_vertex_handle(mesh_.halfedge_handle(*e_it, 1));
        vidx = vh.idx();

        e_cache_.push_back(drawMesh_ ? drawMesh_->mapVertexToVBOIndex(vidx) : vidx);
      }
    }

    std::vector<unsigned int>(e_cache_.begin(), e_cache_.end()).swap(e_cache_);
    // update edge index buffer
    if(e_cache_.size() > 0)
      updateIBOData(eIBO_, e_cache_.size() , sizeof(e_cache_[0]) , e_cache_.data());
    edgeIndexInvalid_ = false;
  }


  // Update index list of selected halfedges
  if (halfedgeCacheInvalid_) {
    he_points_.clear();
    he_points_.reserve(mesh_.n_halfedges()/4);
    he_normals_.clear();
    he_normals_.reserve(he_points_.size());

    typename Mesh::ConstHalfedgeIter he_it(mesh_.halfedges_sbegin()), he_begin(mesh_.halfedges_sbegin()), he_end(mesh_.halfedges_end());
    for (he_it = he_begin; he_it != he_end; ++he_it) {
      if (this->is_halfedge_selected(mesh_, *he_it)) {
        // add vertices
        he_points_.push_back(halfedge_point(*he_it));
        he_points_.push_back(halfedge_point(mesh_.prev_halfedge_handle(*he_it)));

        // add normals
        FaceHandle fh;
        if (!mesh_.is_boundary(*he_it))
          fh = mesh_.face_handle(*he_it);
        else
          fh = mesh_.face_handle(mesh_.opposite_halfedge_handle(*he_it));

        he_normals_.push_back(mesh_.normal(fh));
        he_normals_.push_back(mesh_.normal(fh));
      }
    }

    std::vector<Point>(he_points_.begin(), he_points_.end()).swap(he_points_);
    std::vector<Normal>(he_normals_.begin(), he_normals_.end()).swap(he_normals_);
    //update the Halfedge VBO
    if(he_points_.size() > 0)
      updateHEVBOPoints(he_points_.size() , sizeof(he_points_[0]) , he_points_.data());
    halfedgeCacheInvalid_ = false;
  }


  // update index list of selected faces
  if (faceIndexInvalid_) {

    fh_cache_.clear(); //constant time, facehandle is trivially destructible
    fh_cache_.reserve(mesh_.n_faces()/4);// maximum 2 new allocations will be performed using push_back

    typename Mesh::ConstFaceIter f_it(mesh_.faces_sbegin()), f_begin(mesh_.faces_sbegin()), f_end(mesh_.faces_end());
    for (f_it = f_begin; f_it != f_end; ++f_it)
      if (this->is_face_selected(mesh_, *f_it))
        fh_cache_.push_back(*f_it);

    std::vector<FaceHandle>(fh_cache_.begin(), fh_cache_.end()).swap(fh_cache_);//shrink to fit

    if (mesh_.is_trimesh())
    {
      f_cache_.resize(fh_cache_.size()*3);
      for (size_t i = 0; i < fh_cache_.size(); ++i)
      {
        typename Mesh::ConstFaceVertexIter fv_it = mesh_.cfv_iter(fh_cache_[i]);
        unsigned int vidx = fv_it->idx();
        f_cache_[i*3] = (drawMesh_ ? drawMesh_->mapVertexToVBOIndex(vidx) : vidx);

        ++fv_it;
        vidx = fv_it->idx();
        f_cache_[i*3+1] = (drawMesh_ ? drawMesh_->mapVertexToVBOIndex(vidx) : vidx);

        ++fv_it;
        vidx = fv_it->idx();
        f_cache_[i*3+2] = (drawMesh_ ? drawMesh_->mapVertexToVBOIndex(vidx) : vidx);
      }
    }
    else {
      // triangulate poly-list
      poly_cache_.clear();
      poly_cache_.reserve(fh_cache_.size()*4);

      typename std::vector<FaceHandle>::const_iterator fh_it(fh_cache_.begin()), fh_end(fh_cache_.end());
      for (fh_it = fh_cache_.begin(); fh_it != fh_end; ++fh_it) {
        typename Mesh::CFVIter fv_it = mesh_.cfv_iter(*fh_it);

        // 1. polygon vertex
        unsigned int v0 = fv_it->idx();

        // go to next vertex
        ++fv_it;
        unsigned int vPrev = fv_it->idx();
        ++fv_it;

        // create triangle fans pointing towards v0
        for (; fv_it.is_valid(); ++fv_it) {
          poly_cache_.push_back(drawMesh_ ? drawMesh_->mapVertexToVBOIndex(v0) : v0);
          poly_cache_.push_back(drawMesh_ ? drawMesh_->mapVertexToVBOIndex(vPrev) : vPrev);

          vPrev = fv_it->idx();
          poly_cache_.push_back(drawMesh_ ? drawMesh_->mapVertexToVBOIndex(vPrev) : vPrev);
        }
      }

      std::vector<unsigned int>(poly_cache_.begin(), poly_cache_.end()).swap(poly_cache_);//shrink to fit
    }
    // update trimesh face index buffer
    if(f_cache_.size() > 0)
        updateIBOData(fIBO_, f_cache_.size(), sizeof(f_cache_[0]), f_cache_.data());
    // update polymesh face index buffer
    if(poly_cache_.size() > 0)
        updateIBOData(pIBO_, poly_cache_.size(), sizeof(poly_cache_[0]), poly_cache_.data());
    faceIndexInvalid_ = false;
  }

}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
void
StatusNodeT<Mesh, Mod>::
draw(GLState& _state, const DrawModes::DrawMode& _drawMode)
{
  // Call updater function before doing anything
  update_cache();

  // using static bitflags for drawmodes is no longer recommended
  //  read from properties instead:

  bool shaded = false;
  bool smooth = false;
  bool
    wires = ((this->drawMode() == DrawModes::DEFAULT) ||
            this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_WIREFRAME) >= 0 ||
            _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_WIREFRAME) >= 0),
    points = ((this->drawMode() == DrawModes::DEFAULT) ||
            this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POINT) >= 0 ||
            _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POINT) >= 0),
    edges = (this->drawMode() == DrawModes::DEFAULT ||
            this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_EDGE) >= 0 ||
            _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_EDGE) >= 0),
    halfedges = ((this->drawMode() == DrawModes::DEFAULT) ||
            this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_HALFEDGE) >= 0 ||
            _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_HALFEDGE) >= 0),
    faces = ((this->drawMode() == DrawModes::DEFAULT) ||
            this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POLYGON) >= 0 ||
            _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POLYGON) >= 0);

  for (unsigned int i = 0; i < _drawMode.getNumLayers(); ++i)
  {
    const DrawModes::DrawModeProperties* props = _drawMode.getLayer(i);

    if (props->lighting())
      shaded = true;

    if (props->normalSource() == DrawModes::NORMAL_PER_VERTEX ||
      props->normalSource() == DrawModes::NORMAL_PER_HALFEDGE)
      smooth = true;
  }

  // force shaded selections
  shaded = true;


  GLenum prev_depth = _state.depthFunc();

  ACG::GLState::depthFunc(GL_LEQUAL);

  if (shaded)  ACG::GLState::enable(GL_LIGHTING);
  else         ACG::GLState::disable(GL_LIGHTING);

  if (smooth)  ACG::GLState::shadeModel(GL_SMOOTH);
  else         ACG::GLState::shadeModel(GL_FLAT);

  if (drawMesh_)
  {
    ACG::GLState::bindBuffer(GL_ARRAY_BUFFER, drawMesh_->getVBO());
    drawMesh_->getVertexDeclaration()->activateFixedFunction();

    // disable unwanted attributes from drawmesh vbo
    ACG::GLState::disableClientState(GL_COLOR_ARRAY);
    ACG::GLState::disableClientState(GL_TEXTURE_COORD_ARRAY);

    ACG::GLState::disable(GL_TEXTURE_2D);
    ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);
  }
  else
  {
    // use buffers from open mesh
    if (shaded && mesh_.has_vertex_normals()) {
      ACG::GLState::enableClientState(GL_NORMAL_ARRAY);
      ACG::GLState::normalPointer(mesh_.vertex_normals());
    }

    ACG::GLState::enableClientState(GL_VERTEX_ARRAY);
    ACG::GLState::vertexPointer(mesh_.points());
  }


  // points
  if (points)
    draw_points();


  // edges
  if (edges)
    draw_edges();


  if (shaded && !smooth)
    ACG::GLState::disableClientState(GL_NORMAL_ARRAY);


  // faces
  if (faces)  {
    if (wires) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      draw_faces(smooth);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    } else {

      glPushAttrib( GL_ENABLE_BIT );

      ACG::GLState::enable(GL_POLYGON_OFFSET_FILL);

      glPolygonOffset(0.001f, 0.0f);
      draw_faces(smooth);

      glPopAttrib();
    }
  }

  // disable gpu buffer for halfedges
  ACG::GLState::bindBuffer(GL_ARRAY_BUFFER, 0);
  ACG::GLState::disableClientState(GL_NORMAL_ARRAY);


  // half edges
  if (halfedges)
    draw_halfedges();

  ACG::GLState::disableClientState(GL_NORMAL_ARRAY);
  ACG::GLState::disableClientState(GL_VERTEX_ARRAY);

  ACG::GLState::depthFunc(prev_depth);
}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
void
StatusNodeT<Mesh, Mod>::
draw_points()
{
  if ( !v_cache_.empty() )
    glDrawElements(GL_POINTS,
  		           int( v_cache_.size() ),
		           GL_UNSIGNED_INT,
		           &v_cache_[0]);
}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
void
StatusNodeT<Mesh, Mod>::
draw_edges()
{
  if ( !e_cache_.empty() )
    glDrawElements( GL_LINES,
  		            int(e_cache_.size()),
		            GL_UNSIGNED_INT,
                    &e_cache_[0]);
}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
void
StatusNodeT<Mesh, Mod>::
draw_halfedges() {
  if ( !he_points_.empty()) {
    ACG::GLState::enableClientState(GL_NORMAL_ARRAY);

    ACG::GLState::vertexPointer(&he_points_[0]);

    if ( !he_normals_.empty())
      ACG::GLState::normalPointer(&he_normals_[0]);

    glDrawArrays(GL_LINES, 0, int(he_points_.size() ) );
  }

}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
void
StatusNodeT<Mesh, Mod>::
draw_faces(bool _per_vertex)
{
  typename std::vector<FaceHandle>::const_iterator  fh_it(fh_cache_.begin()),
                                                    fh_end(fh_cache_.end());
  typename Mesh::CFVIter                            fv_it;


  // TRIANGLES
  if (mesh_.is_trimesh()) {

    if (!_per_vertex) {
      glBegin(GL_TRIANGLES);
      for (; fh_it!=fh_end; ++fh_it) {
        glNormal(mesh_.normal(*fh_it));
        glVertex(mesh_.point(*(fv_it=mesh_.cfv_iter(*fh_it))));
        glVertex(mesh_.point(*(++fv_it)));
        glVertex(mesh_.point(*(++fv_it)));
      }

      glEnd();
    } else {

      if ( !f_cache_.empty() )
        glDrawElements(GL_TRIANGLES,
                       int(f_cache_.size()),
                       GL_UNSIGNED_INT,
                       &f_cache_[0]);
    }

    // POLYGONS
  } else {

    if (!_per_vertex) {

      for (; fh_it!=fh_end; ++fh_it) {
        glBegin(GL_POLYGON);
        glNormal(mesh_.normal(*fh_it));
        for (fv_it=mesh_.cfv_iter(*fh_it); fv_it.is_valid(); ++fv_it)
          glVertex(mesh_.point(*fv_it));
        glEnd();
      }

    } else {

      for (; fh_it!=fh_end; ++fh_it) {
        glBegin(GL_POLYGON);
        for (fv_it=mesh_.cfv_iter(*fh_it); fv_it.is_valid(); ++fv_it) {
          glNormal(mesh_.normal(*fv_it));

          if (drawMesh_) // map to vbo index
            glArrayElement(drawMesh_->mapVertexToVBOIndex(fv_it->idx()));
          else
            glArrayElement(fv_it->idx());
        }
        glEnd();
      }

    }
  }
}


//----------------------------------------------------------------------------


template <class Mesh, class Mod>
typename Mesh::Point
StatusNodeT<Mesh, Mod>::
halfedge_point(const HalfedgeHandle _heh) 
{
  typename Mesh::Point p  = mesh_.point(mesh_.to_vertex_handle  (_heh));
  typename Mesh::Point pp = mesh_.point(mesh_.from_vertex_handle(_heh));
  typename Mesh::Point pn = mesh_.point(mesh_.to_vertex_handle(mesh_.next_halfedge_handle(_heh)));

  //  typename Mesh::Point n  = (p-pp)%(pn-p);
  typename Mesh::Point fn;
  if( !mesh_.is_boundary(_heh))
    fn = mesh_.normal(mesh_.face_handle(_heh));
  else
    fn = mesh_.normal(mesh_.face_handle(mesh_.opposite_halfedge_handle(_heh)));

  typename Mesh::Point upd = ((fn%(pn-p)).normalize() + (fn%(p-pp)).normalize()).normalize();

  upd *= ((pn-p).norm()+(p-pp).norm())*0.08;


  return (p+upd);

  // double alpha = 0.1;
  // // correct weighting for concave triangles (or at concave boundaries)
  // if( (fn | n)  < 0.0) alpha *=-1.0;

  // return (p*(1.0-2.0*alpha) + pp*alpha + pn*alpha);
}

//----------------------------------------------------------------------------

template <class Mesh, class Mod>
void StatusNodeT<Mesh, Mod>::getRenderObjects(IRenderer* _renderer,
                                              GLState& _state,
                                              const DrawModes::DrawMode& _drawMode,
                                              const class Material* _mat)
{
  // Call updater function before doing anything
  update_cache();

  bool shaded = false,
    points = ((this->drawMode() == DrawModes::DEFAULT) ||
    this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POINT) >= 0 ||
    _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POINT) >= 0),
    edges = (this->drawMode() == DrawModes::DEFAULT ||
    this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_EDGE) >= 0 ||
    _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_EDGE) >= 0),
    halfedges = ((this->drawMode() == DrawModes::DEFAULT) ||
    this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_HALFEDGE) >= 0 ||
    _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_HALFEDGE) >= 0),
    faces = ((this->drawMode() == DrawModes::DEFAULT) ||
    this->drawMode().getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POLYGON) >= 0 ||
    _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POLYGON) >= 0);

  RenderObject ro;
  ro.debugName = "StatusNode";
  ro.initFromState(&_state);

  ro.depthTest = true;
  ro.depthFunc = GL_LEQUAL;

  // Use the material from the underlying materialnode
  ro.setMaterial(&MaterialNode::material());

  pointVertexDecl_.clear();
  pointVertexDecl_.addElement(GL_DOUBLE, 3, VERTEX_USAGE_POSITION, mesh_.points());

  if (shaded && mesh_.has_vertex_normals()) 
    pointVertexDecl_.addElement(GL_DOUBLE, 3, VERTEX_USAGE_NORMAL, mesh_.vertex_normals());

  pointVertexDecl_.setVertexStride(24); // separated buffers, double3:  24 bytes


  // draw status later than scene
  ro.priority = 1;


  // enable lighting
  if (shaded)
    ro.shaderDesc.shadeMode = SG_SHADE_GOURAUD;

  if (drawMesh_)
  {
    ro.vertexBuffer = drawMesh_->getVBO();
    ro.vertexDecl = drawMesh_->getVertexDeclaration();
    //ro.indexBuffer = drawMesh_->getIBO();
  }
  else
    ro.vertexDecl = &pointVertexDecl_;

  // point list
  if (points && !v_cache_.empty())
  {
    // use shaders to simulate line width
    QString geomTemplate = ShaderProgGenerator::getShaderDir();
    geomTemplate += "PointSize/geometry.tpl";

    QString fragTemplate = ShaderProgGenerator::getShaderDir();
    fragTemplate += "PointSize/fragment.tpl";

    ro.shaderDesc.geometryTemplateFile = geomTemplate;
    ro.shaderDesc.fragmentTemplateFile = fragTemplate;
    ro.indexBuffer = vIBO_;

    ro.setUniform("screenSize", Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));
    ro.setUniform("pointSize", MaterialNode::point_size());

    ro.glDrawElements(GL_POINTS, static_cast<GLsizei>(v_cache_.size()), GL_UNSIGNED_INT, (void *)0);
    _renderer->addRenderObject(&ro);

    ro.shaderDesc.geometryTemplateFile = "";
    ro.shaderDesc.fragmentTemplateFile = "";
  }

  // edge list
  if (edges && !e_cache_.empty())
  {
    // use shaders to simulate line width
    QString geomTemplate = ShaderProgGenerator::getShaderDir();
    geomTemplate += "Wireframe/geom_line2quad.tpl";

    ro.shaderDesc.geometryTemplateFile = geomTemplate;
    ro.indexBuffer = eIBO_;

    ro.setUniform("screenSize", Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));
    ro.setUniform("lineWidth", MaterialNode::line_width());

    ro.glDrawElements(GL_LINES, static_cast<GLsizei>(e_cache_.size()), GL_UNSIGNED_INT, (void *)0);
    _renderer->addRenderObject(&ro);

    ro.shaderDesc.geometryTemplateFile = "";
    ro.shaderDesc.fragmentTemplateFile = "";
  }


  if (faces)
  {
    if (mesh_.is_trimesh() && !f_cache_.empty())
    {
      ro.indexBuffer = fIBO_;
      ro.glDrawElements(GL_TRIANGLES,  static_cast<GLsizei>(f_cache_.size()), GL_UNSIGNED_INT,  (void *)0);
      _renderer->addRenderObject(&ro);
    }
    else if (!poly_cache_.empty()) //if mesh is not a triangle mesh, poly_cache is always empty
    {
        ro.indexBuffer = pIBO_;
      ro.glDrawElements(GL_TRIANGLES,  static_cast<GLsizei>(poly_cache_.size()), GL_UNSIGNED_INT,  (void *)0);
      _renderer->addRenderObject(&ro);
    }
  }


  // halfedge list
  if (halfedges && !he_points_.empty())
  {
    ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;

    halfedgeVertexDecl_.clear();
    halfedgeVertexDecl_.addElement(GL_DOUBLE, 3, VERTEX_USAGE_POSITION, (void *)0);

    ro.vertexBuffer = heVBO_;
    ro.vertexDecl = &halfedgeVertexDecl_;
    ro.indexBuffer = 0;

    ro.glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(he_points_.size()) );
    _renderer->addRenderObject(&ro);
  }


}


//----------------------------------------------------------------------------

template <class Mesh, class Mod>
void StatusNodeT<Mesh, Mod>::updateGeometry() {
  invalidGeometry_ = true;
}


template <class Mesh, class Mod>
void StatusNodeT<Mesh, Mod>::updateTopology() {
  vertexIndexInvalid_   = true;
  halfedgeCacheInvalid_ = true;
  edgeIndexInvalid_     = true;
  faceIndexInvalid_     = true;

}

template <class Mesh, class Mod>
void StatusNodeT<Mesh, Mod>::updateSelection() {
  vertexIndexInvalid_   = true;
  halfedgeCacheInvalid_ = true;
  edgeIndexInvalid_     = true;
  faceIndexInvalid_     = true;

}

template <class Mesh, class Mod>
void StatusNodeT<Mesh, Mod>::setDrawMesh(DrawMeshT<Mesh>* _drawmesh){
  drawMesh_ = _drawmesh;
}


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

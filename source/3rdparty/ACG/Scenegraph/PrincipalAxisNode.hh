/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2016, RWTH-Aachen University                 *
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
//  CLASS PrincipalAxisNode
//
//=============================================================================


#ifndef ACG_PRINCIPAL_AXIS_NODE_HH
#define ACG_PRINCIPAL_AXIS_NODE_HH


//== INCLUDES =================================================================

#include <ACG/Config/ACGDefines.hh>
#include <ACG/GL/VertexDeclaration.hh>
#include <ACG/GL/GLPrimitives.hh>
#include <ACG/GL/globjects.hh>

#include "BaseNode.hh"
#include "DrawModes.hh"
#include <string>
#include <vector>


//== FORWARD DECLARATIONS =========================================================

namespace ACG {
class QtPrincipalAxisDialog;
}
//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== CLASS DEFINITION =========================================================


struct ACGDLLEXPORT PrincipalComponent
{
  // position
  Vec3d p;

  // main axis (normalized) multiplied with eigenvalues
  Vec3d a[3];

  // positive=1 or negative=0 eigenvalue ?
  bool sign[3];

  // Constructor
  PrincipalComponent() {}

  PrincipalComponent( Vec3d _p,
		      Vec3d _a0,
		      Vec3d _a1,
		      Vec3d _a2,
		      bool  _s0,
		      bool  _s1,
		      bool  _s2 )
  {
    p = _p;

    a[0] = _a0;
    a[1] = _a1;
    a[2] = _a2;

    sign[0] = _s0;
    sign[1] = _s1;
    sign[2] = _s2;
  }

  // Copy Constructor
  PrincipalComponent( const PrincipalComponent& _pc)
  {
    // use defined = operator
    *this = _pc;
  }

  // = operator
  PrincipalComponent& operator=(const PrincipalComponent& _pc)
  {
    p = _pc.p;

    a[0] = _pc.a[0];
    a[1] = _pc.a[1];
    a[2] = _pc.a[2];

    sign[0] = _pc.sign[0];
    sign[1] = _pc.sign[1];
    sign[2] = _pc.sign[2];

    return *this;
  }

};




class ACGDLLEXPORT PrincipalAxisNode : public BaseNode
{

public:

  typedef PrincipalComponent PrincipalComponentT;

  // Option enums
  enum DrawStyle { DS_3D   = 1, DS_2D   = 2};
  enum ColorMode { CM_Axis = 1, CM_Sign = 2};


  /// Default constructor.
  PrincipalAxisNode( BaseNode*         _parent=0,
               const std::string&      _name="<PrincipalAxis>" );

  /// destructor
  virtual ~PrincipalAxisNode();

  // show Qt-Options-Dialog
  void show_options_dialog();

  // draw settings
  void set_draw_style(DrawStyle _ds) { draw_style_ = _ds;}
  void set_color_mode(ColorMode _cm);
  void show_tensor_component(unsigned int _i, unsigned char _show);

  // number of tensors to display
  size_t size() {return pc_.size();}

  void resize( size_t _n);

	void clear() { pc_.clear(); invalidateInstanceData_ = true; }

  // enable/disable drawing the _i'th PC
  void enable ( size_t _i);
  void disable( size_t _i);
  void disable_all();

  // set properties of Principal component
  template<class VectorT>
  void set_vector( unsigned int _i, const Vec3d _p, const VectorT& _v);
  template<class MatrixT>
  void set_matrix( unsigned int _i, const Vec3d _p, const MatrixT& _m);
  void set( size_t _i, const PrincipalComponent& _pc);
  void get( size_t _i,       PrincipalComponent& _pc);
  void add( const PrincipalComponent& _pc, bool _enable = true);
  // enable automatic range clamping
  void set_auto_range( bool _b);

  void set_min_abs_value( double _v);
  void set_max_abs_value( double _v);

  void set_min_draw_radius( double _v);
  void set_max_draw_radius( double _v);

  double get_min_draw_radius() const { return min_draw_radius_; }
  double get_max_draw_radius() const { return max_draw_radius_; }

  double get_min_spacing() const { return min_spacing_; }

  /// Indicates whether the min/max draw radius has been changed from its default setting.
  bool is_default_radius() const { return default_radius_; }

  void auto_update_range();

  void update_bounding_box();

  ACG_CLASSNAME(PrincipalAxisNode);

  /// return available draw modes
  DrawModes::DrawMode availableDrawModes() const override;

  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  
  /// drawing the primitive
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  void draw_principal_component( const PrincipalComponent& _pc);

  void draw_arrow( const Vec3d& _axis, double _r);

  void draw_line( const Vec3d& _axis, double _w);

  /// picking
  void pick(GLState& _state, PickTarget _target) override;


  // set drawing parameters
  void set_draw_quality(double _q) { slices_ = int(_q); }

  void set_cylinder_radius_scale(double _s) { cylinder_radius_scale_ = _s;}

  void set_axes_colors(const Vec4f colors[3]);
  void get_axes_colors(Vec4f out_colors[3]) const;

  /// Overriding BaseNode::getRenderObjects.
  void getRenderObjects(IRenderer* _renderer, GLState&  _state , const DrawModes::DrawMode&  _drawMode , const ACG::SceneGraph::Material* _mat) override;

  /// world transform of an axis (orientation and translation)
  GLMatrixd axisTransform(const PrincipalComponent& _pc, int _axis, double* _outSize = 0) const;

  /// scaled axis
  Vec3d axisScaled(const PrincipalComponent& _pc, int _axis) const;

  /// emit individual objects for each axis for each principal component (slow if tensor count high)
  void emitIndividualRenderobjects(IRenderer* _renderer, GLState& _state, const DrawModes::DrawMode& _drawMode,  const ACG::SceneGraph::Material* _mat);

  void updateVBO() { updateVBO_ = true; };

private:

  /// creates the vbo only if update was requested
  void createVBO();

  void diagonalize(const double (&A)[3][3], double (&Q)[3][3], double (&D)[3][3]);

  // vector of Principal Components
  std::vector< PrincipalComponent > pc_;

  // is enabled ?
  std::vector< bool > draw_pc_;

  // determine rescaling properties automatically
  bool auto_range_;
  
  // min/max eigenvalue clamping
  double max_abs_value_;
  double min_abs_value_;

  // min/max drawing size
  double max_draw_radius_;
  double min_draw_radius_;

  /// Indicates whether the min/max draw radius has been changed from its default setting.
  bool default_radius_;

  // precomputed boundingbox
  Vec3d bbMin_;
  Vec3d bbMax_;

  // drawing parameters
  int    slices_;
  double cylinder_radius_scale_;

  // minimum distance between two crosses
  double min_spacing_;

  DrawStyle draw_style_;
  ColorMode color_mode_;

  // 0: no drawing // 1:given direction // 2:both directions
  unsigned char show_tensor_component_[3];

  friend class ACG::QtPrincipalAxisDialog;

  const float cone_height_factor_; // cone_height / base_radius
  GLCylinder cylinder_;

  GLCone cone_;

  GeometryBuffer    lineBuffer_;
  VertexDeclaration lineDecl_;
  VertexDeclaration lineDeclInstanced_;

  // data per instance:
  //  float4x3     axisTransform
  //  float        size
  //  byte4_unorm  color
  GeometryBuffer lineInstanceBuffer_;
  bool invalidateInstanceData_;

  VertexDeclaration cylinderDeclInstanced_;

  int supportsInstancing_;

  GLfloat axes_colors[3][4];

  // Vertex buffer object used in this node
  unsigned int vbo_;

  ACG::VertexDeclaration vertexDecl_;

  // True if points changed and the vbo has to be updated
  bool         updateVBO_;

  std::string nodeName_;

};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(ACG_PRINCIPAL_AXIS_NODE_C)
#define ACG_PRINCIPAL_AXIS_NODE_TEMPLATES
#include "PrincipalAxisNodeT_impl.hh"
#endif
//=============================================================================
#endif // ACG_PRINCIPAL_AXIS_NODE_HH
//=============================================================================

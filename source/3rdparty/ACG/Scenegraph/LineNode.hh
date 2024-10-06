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
//  CLASS LineNode
//
//=============================================================================


#ifndef ACG_LINENODE_HH
#define ACG_LINENODE_HH


//== INCLUDES =================================================================

#include <ACG/Scenegraph/MaterialNode.hh>
#include "DrawModes.hh"
#include <ACG/GL/VertexDeclaration.hh>
#include <vector>
#include <limits>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== CLASS DEFINITION =========================================================



/** \class LineNode LineNode.hh <ACG/Scenegraph/LineNode.hh>

    LineNode renders a set of line segments or polylines.

    LineNode renders a set of line segments or one connected polyline,
    depending on the LineMode, that can be set using the
    set_line_mode(LineMode) method.
**/

class ACGDLLEXPORT LineNode : public MaterialNode
{
public:

  // typedefs
  typedef ACG::Vec3uc                    Color;
  typedef ACG::Vec4f                     Color4f;
  typedef std::vector<Vec3d>             PointVector;
  typedef PointVector::iterator          PointIter;
  typedef PointVector::const_iterator    ConstPointIter;
  typedef std::vector<ACG::Vec3uc>       ColorVector;
  typedef ColorVector::iterator          ColorIter;
  typedef ColorVector::const_iterator    ConstColorIter;
  typedef std::vector<Color4f>           Color4fVector;
  typedef Color4fVector::iterator        Color4fIter;
  typedef Color4fVector::const_iterator  ConstColor4fIter;

  /// Line mode: draw line segments (every 2 points) or ONE polyline.
  enum LineMode { LineSegmentsMode, PolygonMode };



  /// default constructor
  LineNode( LineMode     _mode,
	          BaseNode*    _parent=0,
	          std::string  _name="<LineNode>" );

  /// destructor
  ~LineNode();

  /// set line mode (see LineNode::LineMode)
  void set_line_mode(LineMode _mode);


  /// static name of this class
  ACG_CLASSNAME(LineNode);

  /// return available draw modes
  DrawModes::DrawMode  availableDrawModes() const override;

  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  
  
  /// set depth function (needed for lasso selection so that the line can be draw in pseudo-2D)
  void enter(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /// draw lines and normals
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;
  void drawCompat(GLState& _state, const DrawModes::DrawMode& _drawMode);
  
  /// reset depth function to what it was before enter()
  void leave(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /// Draw the line using the GL picking name stack
  void pick(GLState&  _state , PickTarget _target) override;
  void pickCompat(GLState&  _state , PickTarget _target);

  /// reserve mem for _n lines
  void reserve_lines(unsigned int _n) { points_.reserve(2*_n); }

  /// reserve mem for _n points
  void reserve_points(unsigned int _n) { points_.reserve(_n); }

  /// clear points/lines and colors
  void clear();

  /// clear points/lines
  void clear_points();

  /// clear colors
  void clear_colors();
  
  /// Override material node's set color function in order to locally add color
  void set_color(const Vec4f& _c);

  /// add point (for LineMode == PolygonMode)
  void add_point(const Vec3d& _v);

  /// add line (for LineMode == LineSegmentsMode)
  void add_line(const Vec3d& _v0, const Vec3d& _v1);

  /// add color (only for LineMode == LineSegmentsMode)
  void add_color(const ACG::Vec3uc& _c);

  /// add color 4f (only for LineMode == LineSegmentsMode)
  void add_color(const Color4f _c);

  /// set line width used by the picking renderer
  void set_picking_line_width(float _width) { picking_line_width_ = _width; }
  /// get line width used by the picking renderer. Defaults to line_width().
  float picking_line_width() const
  {
      return (picking_line_width_ != std::numeric_limits<float>::infinity()) ? picking_line_width_ : line_width();
  }

  /// number of points
  size_t n_points() const { return points_.size(); }

  /**\brief return reference to point vector
   *
   * If you change something here, you need to call updateVBO() in order
   * to tell the system, that your data arrays changed
   */
  const PointVector& points() const { return points_; }

  /**\brief get and set color container
   *
   * If you change something here, you need to call updateVBO() in order
   * to tell the system, that your data arrays changed
   */
  ColorVector& colors() { return colors_; }
  
  /// get and set always on top
  bool& alwaysOnTop() { updateVBO_ = true; return draw_always_on_top;  }

  void updateVBO() { updateVBO_ = true; };

  /// STL conformance
  void push_back(const Vec3d& _v) { points_.push_back(_v); updateVBO_ = true; }
  typedef Vec3d         value_type;
  typedef Vec3d&        reference;
  typedef const Vec3d&  const_reference;

  /** \brief Add the objects to the given renderer
   *
   * @param _renderer The renderer which will be used. Add your geometry into this class
   * @param _state    The current GL State when this object is called
   * @param _drawMode The active draw mode
   * @param _mat      Current material
   */
  void getRenderObjects(IRenderer* _renderer, GLState&  _state , const DrawModes::DrawMode&  _drawMode , const ACG::SceneGraph::Material* _mat) override;

protected:

  void pick_vertices(GLState& _state);
  void pick_edges (GLState& _state, unsigned int _offset);
  void pick_edgesCompat (GLState& _state, unsigned int _offset);

  /// creates the vbo only if update was requested
  void createVBO();

  /// Line width used by the picking renderer. If this is not set (i.e. NAN),
  /// line_width() is used instead.
  float picking_line_width_;

  PointVector   points_;
  ColorVector   colors_;
  Color4fVector colors4f_;

  LineMode     line_mode_;
  
  bool	       draw_always_on_top;
  GLint	       prev_depth_;

  // Vertex buffer object used in this node
  unsigned int vbo_;

  // True if points changed and the vbo has to be updated
  bool         updateVBO_;

  ACG::VertexDeclaration vertexDecl_;

  std::string lineNodeName_;

};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_LINENODE_HH defined
//=============================================================================


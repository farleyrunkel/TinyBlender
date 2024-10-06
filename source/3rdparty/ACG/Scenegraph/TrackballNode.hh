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
//  CLASS TrackballNode
//
//=============================================================================


#ifndef ACG_TRACKBALL_NODE_HH
#define ACG_TRACKBALL_NODE_HH


//== INCLUDES =================================================================

// GMU
#include "BaseNode.hh"
#include "TransformNode.hh"

// Qt
#include <QEvent>
#include <QMouseEvent>

#ifdef max
#  undef max
#endif

#ifdef min
#  undef min
#endif



//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================

	      
/** \class TrackballNode TrackballNode.hh <ACG/Scenegraph/TrackballNode.hh>

    Use a virtual trackball to rotate, translate and scale the children
    of this node. If you want to do translations use the ManipulatorNode.

    \see ACG::SceneGraph::ManipulatorNode
**/
  
class ACGDLLEXPORT TrackballNode : public TransformNode
{
public:
   
  
  /// Default constructor.
  TrackballNode( BaseNode* _parent=0,
		 const std::string& _name="<TrackballNode>" )
    : TransformNode(_parent, _name),
      drawTrackball_(false),
      drawAxes_(false),
      radius_(1.0),
      xAxis_(1.0, 0.0, 0.0),
      yAxis_(0.0, 1.0, 0.0),
      zAxis_(0.0, 0.0, 1.0)
  {}

  
  /// Destructor.
  ~TrackballNode() {}

    
  /// Class name macro
  ACG_CLASSNAME(TrackballNode);

  
  //
  // METHODS
  //


  /// Set trackball radius
  void set_radius(double _r) { radius_ = _r; }
  /// Get trackball radius
  double radius() const { return radius_; }

  
  /// override TransformNode::setIdentity() (update radius_)
  virtual void setIdentity()  override{
    Vec3d vec (1.0, 1.0, 1.0);
    
    vec = scale ().transform_point(vec);
    radius_ *= vec.max ();
    TransformNode::setIdentity();
  }

  
  /// Turn on/off drawing of the trackball
  void set_draw_trackball(bool _b) { drawTrackball_ = _b; }
  /// Is trackball-drawing on?
  bool draw_trackball() const { return drawTrackball_; }


  /// Turn drawing the axes on/off
  void set_draw_axes(bool _b) { drawAxes_ = _b; }
  /// Is axes-drawing on?
  bool draw_axes() const { return drawAxes_; }
  
  
  /// Draw the trackball + axes (if enabled)
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;


  /// get mouse events
  virtual void mouseEvent(GLState& _state, QMouseEvent* _event) override;
  
  
private:

  /// Map 2D-screen-coords to trackball
  bool mapToSphere(const GLState &_state, const Vec2i& _v2, Vec3d& _v3);

  
  // ELEMENTS
  bool                                 drawTrackball_,
                                       drawAxes_;
  double                               radius_;
  Vec3d                                xAxis_, yAxis_, zAxis_;
  Vec2i                                oldPoint2D_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_TRACKBALL_NODE_HH defined
//=============================================================================


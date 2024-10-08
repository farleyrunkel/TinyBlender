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
//  CLASS ManipulatorNode
//
//=============================================================================


#ifndef ACG_MANIPULATOR_NODE_HH
#define ACG_MANIPULATOR_NODE_HH


//== INCLUDES =================================================================

// GMU
#include "BaseNode.hh"
#include "TransformNode.hh"
#include <ACG/GL/GLPrimitives.hh>

// Qt
#include <QEvent>
#include <QMouseEvent>




//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================


/** \class ManipulatorNode ManipulatorNode.hh <ACG/Scenegraph/ManipulatorNode.hh>
    Use a manipulator to rotate, translate and scale the children of
    this node.
*/

class ACGDLLEXPORT ManipulatorNode : public TransformNode
{
public:


  /// Default constructor.
  ManipulatorNode( BaseNode* _parent=0,
		   const std::string& _name="<ManipulatorNode>" );

  /// Destructor.
  ~ManipulatorNode();


  /// class name
  ACG_CLASSNAME(ManipulatorNode);


  //
  // METHODS
  //



  virtual void setIdentity() override;

  /// set draw_cylinder_
  void set_draw_cylinder(bool _b) { draw_cylinder_ = _b; }

  /// get drawCylinder
  bool draw_cylinder() const { return draw_cylinder_; }

  /** Set direction in world coordinates. Will take the current
      rotation into account. **/
  void set_direction(Vec3d& _v);

  /** Get direction in world coordinates. Will take the current
      rotation into account. **/
  Vec3d direction() const;

  /// set cylindersize  (height + radius)
  void set_size( double _size) { cylinder_height_ = _size;
                                 cylinder_radius_ = _size/10.0;}

  /// get cylindersize
  double size() const { return cylinder_height_; }

  /// translate in cylinder direction
  void translate(double _s) {
    Vec3d rel_dir = rotation().transform_vector(direction_);
    TransformNode::translate(_s * rel_dir);
  }

  /// draw the cylinder (if enabled)
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /// picking
  void pick(GLState& _state, PickTarget _target) override;

  /// get mouse events
  virtual void mouseEvent(GLState& _state, QMouseEvent* _event) override;

  /** if the manipulator node has been clicked by the mouse, this function
  *   will return true. Use reset_touched to reset this flag
  */
  bool touched() { return touched_; };

  /// Reset the touched flag ( see touched for details )
  void reset_touched() { touched_ = false; };


private:

  void setup_sphere_system(GLState& _state);
  void setup_cylinder_system(GLState& _state);

  bool mapToSphere(GLState& _state, const Vec2i& _v2, Vec3d& _v3);
  bool hitSphere(GLState& _state, const Vec2i& _v2);
  bool mapToCylinder (GLState& _state, const Vec2i& _v2, double& axis_hit);





  // ELEMENTS
  bool               draw_cylinder_;

  Vec3d              direction_;
  GLCylinder*        cylinder_;
  double             cylinder_radius_;
  double             cylinder_height_;
  int                cylinder_slices_;
  int                cylinder_stacks_;


  bool               cylinder_clicked_;
  bool               sphere_clicked_;

  Vec2i              oldPoint2D_;

  /// if the manipulator is cklicked with a mouse, this flag will be set to true
  bool 	             touched_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_MANIPUKATOR_NODE_HH defined
//=============================================================================


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
//  CLASS TranslationManipulatorNode
//
//=============================================================================


#ifndef ACG_TRANSLATIONMANIPULATOR_NODE_HH
#define ACG_TRANSLATIONMANIPULATOR_NODE_HH


//== INCLUDES =================================================================

// GMU
#include "BaseNode.hh"
#include "TransformNode.hh"

// Qt
#include <QEvent>
#include <QMouseEvent>




//== NAMESPACES ===============================================================

namespace ACG {

class GLCone;
class GLDisk;
class GLSphere;


namespace SceneGraph {

//== CLASS DEFINITION =========================================================


/** \class TranslationManipulatorNode TranslationManipulatorNode.hh <ACG/Scenegraph/TranslationManipulatorNode.hh>
    Use a manipulator to translate and scale the children of this node.
*/

class ACGDLLEXPORT TranslationManipulatorNode : public TransformNode
{
public:

  enum AutoSizeMode {
    Never,
    Once,
    Always
  };

  /** \brief enum to define the manipulator mode
   *
   * This enum defines several modes of the manipulator.
   *  - TranslationRotation: Full featured mode, rotations and translations will be possible
   *  - LocalRotation:       The manipulator can be rotated, but no transformation is applied to the child node (use to re-align manipulator)
   *  - Resize:              No rings, no rotation. The manipulator scales along each axis seperatly. Center sphere scales uniformly in all directions
   *  - Place:               --
   */
  enum ManipulatorMode {
    Rotation,
    TranslationRotation,
    LocalRotation,
    Resize,
    Place
  };

  /** \brief enum to define which rotation axis should be enabled
   *
   * This enum defines which rotation axis rings will be visible and available for picking.
   * The modes can be combined.
   */
  enum ActiveRotations {
    X_AXIS   = 1,
    Y_AXIS   = 1 << 1,
    Z_AXIS   = 1 << 2,
    ALL_AXIS = X_AXIS | Y_AXIS | Z_AXIS
  };

  /// Default constructor.
  TranslationManipulatorNode( BaseNode* _parent=0,
		   const std::string& _name="<TranslationTranslationManipulatorNode>" );

  /// Destructor.
  ~TranslationManipulatorNode();


  /// class name
  ACG_CLASSNAME(TranslationManipulatorNode);


  //
  // METHODS
  //

  virtual void setIdentity() override;

  /// set draw_cylinder_
  void set_draw_cylinder(bool _b) { draw_manipulator_ = _b; }

  /// get drawCylinder
  bool draw_manipulator() const { return draw_manipulator_; }

  /// Set direction in world coordinates
  void set_direction(const Vec3d& _directionX, const Vec3d& _directionY);

  /// Get current direction of x-Axis in world coordinates
  Vec3d directionX() const;
  /// Get current direction of y-Axis in world coordinates
  Vec3d directionY() const;
  /// Get current direction of z-Axis in world coordinates
  Vec3d directionZ() const;

  /// set cylindersize  (height + radius). _size parameter has to be greater than zero
  void set_size( double _size) {
    if (_size > 0.0)
    {
      set_manipulator_height_ = _size;
      set_manipulator_radius_ = _size/10.0;
      setDirty ();
    }
  }

  /// get cylindersize
  double size() const { return set_manipulator_height_; }

  /// set auto size mode
  void set_autosize (AutoSizeMode _mode) { auto_size_ = _mode; }

  /// get autosize mode
  AutoSizeMode autosize () const { return auto_size_; }

  /** \brief Enable or disable manipulator handles for specific orientations
   *
   * This enum defines which rotation axis rings will be visible and available for picking.
   * The modes can be combined.
   *
   * @param _active The activated axis
   */
  void enable_rotations(ActiveRotations _active) {
    activeRotations_ = _active;
  }

  /** \brief Get the current setting, which rotation handles are active
    *
    *
    * @return _active The activated axis
    */
  ActiveRotations enabled_rotations() const {
    return activeRotations_;
  }

  void rotate (double _angle, Vec3d _axis)
  {
    TransformNode::rotate(_angle, _axis);
  }

  /// draw the cylinder (if enabled)
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /// create renderobjects for shaderpipeline renderer
  void getRenderObjects(IRenderer* _renderer, GLState& _state, const DrawModes::DrawMode& _drawMode, const Material* _mat) override;

  /// computes world matrix, transforms from model to world space
  GLMatrixd computeWorldMatrix();

  /// leave node
  // void leave(GLState& _state, const DrawModes::DrawMode& _drawMode);

  /// picking
  void pick(GLState& _state, PickTarget _target) override;

  /// get mouse events
  virtual void mouseEvent(GLState& _state, QMouseEvent* _event) override;

  /// bounding box of node
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;

  /// set current operation mode
  void setMode (ManipulatorMode _mode);

  /// get current mode
  ManipulatorMode getMode () const { return mode_; };


protected:
  /// stores if this manipulator was used in order to avoid emitting manipulatorMoved unnecessarily
  bool touched_;

private:

  enum StateUpdates {
    None,
    Over,
    Click
  };

  enum Elements {
    Origin = 0,
    XTop,
    YTop,
    ZTop,
    XAxis,
    YAxis,
    ZAxis,
    XRing,
    YRing,
    ZRing,
    NumElements
  };

  class Element {
    public:
      Element ();

      Vec4f active_target_color_;
      Vec4f active_current_color_;
      Vec4f inactive_target_color_;
      Vec4f inactive_current_color_;

      bool  clicked_;
      bool  over_;
  };

  /// set the current state to follow manipulator transformation
  void update_manipulator_system(GLState& _state);

  /// update the internal rotation matrix ( internal rotation may be changed without modifiing children of this node )
  void update_rotation(GLState& _state);

  bool hitSphere(GLState& _state, const Vec2i& _v2);
  bool hitOuterSphere(GLState& _state, const Vec2i& _v2);

  void drawManipulator (GLState& _state, bool _active);

  void addManipulatorToRenderer (IRenderer* _renderer, RenderObject* _baseRO, bool _active);
  void addAxisToRenderer (IRenderer* _renderer, RenderObject* _baseRO, bool _active, int _axis);


  bool mapToCylinder (GLState& _state, const Vec2i& _v2, StateUpdates _updateStates = None);
  bool mapToCylinderTop (GLState& _state, const Vec2i& _v2, StateUpdates _updateStates = None);

  bool mapToSphere (GLState& _state, const Vec2i& _v2, Vec3d& _v3, StateUpdates _updateStates = None);

  void updateTargetColors ();
  bool updateCurrentColors (GLState& _state);

  double get_screen_length (const GLState &_state, const Vec3d &_point) const;

  void updateSize (const GLState& _state);

  // ELEMENTS
  bool               draw_manipulator_;

  Vec3d              dirX_;
  Vec3d              dirY_;
  Vec3d              dirZ_;

  ACG::GLCone*       axisBottom_; // axis split up for deferred draw call rendering,
  ACG::GLCone*       axisCenter_; // cone vbo data must be consistent within one frame
  ACG::GLCone*       axisTop_;
  ACG::GLDisk*       circle_;
  ACG::GLSphere*     sphere_;

  double             manipulator_radius_;
  double             manipulator_height_;
  double             set_manipulator_radius_;
  double             set_manipulator_height_;
  int                manipulator_slices_;
  int                manipulator_stacks_;

  bool               any_axis_clicked_;
  bool               any_top_clicked_;
  bool               outer_ring_clicked_;

  bool               any_axis_over_;
  bool               any_top_over_;
  bool               outer_ring_over_;


  Element            element_[NumElements];
  float              resize_current_;

  ManipulatorMode    mode_;
  bool               ignoreTime_;

  Vec2i              oldPoint2D_;
  Vec3d              draggingOrigin3D_;
  bool               dragging_;

  Vec3d              currentScale_;

  GLMatrixd          localTransformation_;

  AutoSizeMode       auto_size_;
  double             auto_size_length_;

  ActiveRotations    activeRotations_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_TRANSLATIONMANIPULATOR_NODE_HH defined
//=============================================================================


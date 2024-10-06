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
//  CLASS QtManipulatorNode
//
//=============================================================================


#ifndef ACG_QTMANIPULATOR_NODE_HH
#define ACG_QTMANIPULATOR_NODE_HH


//== INCLUDES =================================================================

// GMU
#include "ACG/Scenegraph/ManipulatorNode.hh"

// Qt
#include <QEvent>
#include <QMouseEvent>


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================


/** \class QtManipulatorNode QtManipulatorNode.hh <ACG/QtScenegraph/QtManipulatorNode.hh>
Additional overlay class for emitting signals if manipulator is moved
*/

class ACGDLLEXPORT QtManipulatorNode : public QObject , public ManipulatorNode
{
  Q_OBJECT

  signals:
    /** This signal is emitted everytime the manipulator is moved
     * Do not traverse Scenegraph when called by this function!!!!!!!
     */
    void manipulatorMoved( QtManipulatorNode* _node, QMouseEvent* _event);

    /** The visibility of this node has changed
     * @param _node    The node whose visibility has been changed
     * @param _visible Is it now visible?
     */
    void visibilityChanged( QtManipulatorNode* _node, bool _visible );

    /** this signal is emited every time the position of the manipulator is changed
     */
    void positionChanged( QtManipulatorNode * _node );

  public:


  /// Default constructor.
  QtManipulatorNode( BaseNode* _parent=0,
		               const std::string& _name="<QtManipulatorNode>" );

  /// Destructor.
  ~QtManipulatorNode();


  /// class name
  ACG_CLASSNAME(QtManipulatorNode);

  //
  // METHODS
  //

  /// get mouse events
  void mouseEvent(GLState& _state, QMouseEvent* _event) override;

  /// Set an identifier for that manipulator
  void setIdentifier( int _id) { identifier_ = _id; };

  /// Get an identifier for that manipulator
  int getIdentifier( ) { return identifier_; };

  void show() { ManipulatorNode::show(); emit visibilityChanged( this, visible() );  };
  void hide() { ManipulatorNode::hide(); emit visibilityChanged( this, visible() );  };

  void rotate(double _angle, const Vec3d& _axis) { TransformNode::rotate(_angle, _axis); emit positionChanged( this ); };

  void translate( double _s ) { ManipulatorNode::translate(_s); emit positionChanged( this ); };

  void set_center( const Vec3d& _c ) { ManipulatorNode::set_center(_c); emit positionChanged( this ); };

  private:
    int identifier_;

};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_QTMANIPULATOR_NODE_HH defined
//=============================================================================


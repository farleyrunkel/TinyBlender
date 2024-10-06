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
//  CLASS QtTranslationManipulatorNode
//
//=============================================================================


#ifndef ACG_QTTRANSLATIONMANIPULATOR_NODE_HH
#define ACG_QTTRANSLATIONMANIPULATOR_NODE_HH


//== INCLUDES =================================================================

// GMU
#include "ACG/Scenegraph/TranslationManipulatorNode.hh"

// Qt
#include <QVariant>
#include <QEvent>
#include <QMouseEvent>


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================


/** \class QtTranslationManipulatorNode QtTranslationManipulatorNode.hh <ACG/QtScenegraph/QtTranslationManipulatorNode.hh>
Additional overlay class for emitting signals if manipulator is moved
*/

class ACGDLLEXPORT QtTranslationManipulatorNode : public QObject , public TranslationManipulatorNode
{
  Q_OBJECT

  signals:
    /** \brief This signal is emitted every time the manipulator is moved by the user via mouse
     *
     * \note Do not traverse Scenegraph when called by this function or you end up in an endless loop!!
     */
    void manipulatorMoved( QtTranslationManipulatorNode * _node, QMouseEvent* _event);

    /** The visibility of this node has changed
     * @param _node    The node whose visibility has been changed
     * @param _visible Is it now visible?
     */
    void visibilityChanged( QtTranslationManipulatorNode * _node, bool _visible );

    /** \brief This signal is emitted every time the position of the manipulator is changed
     *
     * This slot is only triggered, when the center of the manipulator has been changed.
     * Changes in orientation are not triggering this signal
     */
    void positionChanged( QtTranslationManipulatorNode * _node );

  public:


  /// Default constructor.
  QtTranslationManipulatorNode( BaseNode* _parent=0,
		               const std::string& _name="<QtTranslationManipulatorNode>" );

  /// Destructor.
  ~QtTranslationManipulatorNode();


  /// class name
  ACG_CLASSNAME(QtTranslationManipulatorNode);

  //
  // METHODS
  //

  /// get mouse events
  void mouseEvent(GLState& _state, QMouseEvent* _event) override;

  /// Set an identifier for that manipulator
  void setIdentifier( int _id) { identifier_ = _id; };

  /// Get an identifier for that manipulator
  int getIdentifier( ) { return identifier_; };

  /// Set additional data for the node
  void setData(QVariant _data) {data_ = _data;};
  
  /// Get additional data for the node
  QVariant getData() {return data_;};
  
  void show() { TranslationManipulatorNode::show(); emit visibilityChanged( this, visible() );  };
  void hide() { TranslationManipulatorNode::hide(); emit visibilityChanged( this, visible() );  };

  void set_center( const Vec3d& _c ) { TranslationManipulatorNode::set_center(_c); emit positionChanged( this ); };

  private:
    int identifier_;
    QVariant  data_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_QTTRANSLATIONMANIPULATOR_NODE_HH defined
//=============================================================================


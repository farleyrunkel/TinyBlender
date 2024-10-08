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



//================================================================
//
//  CLASS OSDTransformNode
//
//================================================================


#ifndef ACG_OSDTRANSFORMNODE_HH
#define ACG_OSDTRANSFORMNODE_HH


//== INCLUDES ====================================================


#include "BaseNode.hh"
#include "DrawModes.hh"


//== NAMESPACES ==================================================


namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION ============================================


/** \class OSDTransformNode OSDTransformNode.hh <ACG/Scenegraph/OSDTransformNode.hh>

	OSDTransformNode sets up a transformation used to render on screen display geometry.

**/

class ACGDLLEXPORT OSDTransformNode : public BaseNode
{
public:

	/// default constructor
	inline OSDTransformNode( BaseNode *_parent = 0, const std::string &_name = "<OSDTransformNode>" ) : BaseNode( _parent, _name ), center_( 0.0, 0.0 ) { }

	/// destructor
	inline ~OSDTransformNode() { }

	/// static name of this class
	ACG_CLASSNAME( OSDTransformNode );

	/// set up transformation used for drawing
	inline void enter( GLState &_state, const DrawModes::DrawMode &_drawMode ) override { enterOSD( _state, _drawMode ); }

	/// restore transformation used for drawing
	inline void leave( GLState &_state, const DrawModes::DrawMode &_drawMode ) override { leaveOSD( _state, _drawMode ); }

	/// set up transformation used for picking
	inline void enterPick( GLState &_state, PickTarget _target, const DrawModes::DrawMode &_drawMode ) override { enterOSD( _state, _drawMode ); }

	/// restore transformation used for picking
	inline void leavePick( GLState &_state, PickTarget _target, const DrawModes::DrawMode &_drawMode ) override { leaveOSD( _state, _drawMode ); }

	/// set center of OSD
	inline void setCenter( double _x, double _y ) { center_[0] = _x;    center_[1] = _y;    }
	inline void setCenter( ACG::Vec2d _c )        { center_[0] = _c[0]; center_[1] = _c[1]; }

	/// get center of OSD
	inline ACG::Vec2d getCenter() const { return center_; }

private:

	void enterOSD( GLState &_state, const DrawModes::DrawMode &_drawMode );
	void leaveOSD( GLState &_state, const DrawModes::DrawMode &_drawMode );

	ACG::Vec2d center_;
};


//================================================================


} // namespace SceneGraph
} // namespace ACG


//================================================================


#endif // ACG_OSDTRANSFORMNODE_HH

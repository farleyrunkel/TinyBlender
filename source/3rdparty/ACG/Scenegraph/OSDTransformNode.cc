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
//  CLASS OSDTransformNode - IMPLEMENTATION
//
//================================================================


//== INCLUDES ====================================================


#include "OSDTransformNode.hh"

#include <ACG/Scenegraph/SceneGraph.hh>


//== NAMESPACES ==================================================


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==============================================


void OSDTransformNode::enterOSD( GLState &_state, const DrawModes::DrawMode &_drawMode )
{
	_state.push_modelview_matrix();

	int left, bottom, width, height;
	_state.get_viewport( left, bottom, width, height );

	double aspect = _state.aspect();

	_state.push_projection_matrix();
	_state.reset_projection();

	_state.perspective( 45.0, aspect, 0.8, 20.0 );

	_state.push_modelview_matrix();
	_state.reset_modelview();

	double posX = left   + center_[0] * width;
	double posY = bottom + center_[1] * height;

	ACG::Vec3d pos = _state.unproject( ACG::Vec3d( posX, posY, 0.1 ) );

	_state.pop_modelview_matrix();

	GLMatrixd modelview = _state.modelview();

	modelview(0,3) = 0.0;
	modelview(1,3) = 0.0;
	modelview(2,3) = 0.0;

	_state.set_modelview( modelview );
	_state.translate( pos[0], pos[1], pos[2], MULT_FROM_LEFT );
}


//----------------------------------------------------------------


void OSDTransformNode::leaveOSD( GLState &_state, const DrawModes::DrawMode &_drawMode )
{
	_state.pop_projection_matrix();
	_state.pop_modelview_matrix();
}


//================================================================


} // namespace SceneGraph
} // namespace ACG

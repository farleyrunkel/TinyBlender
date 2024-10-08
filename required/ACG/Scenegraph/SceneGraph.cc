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
//  CLASS SceneGraph - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================


#include <ACG/Scenegraph/SceneGraph.hh>


//== NAMESPACES ============================================================== 

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ========================================================== 

/** \brief Find a node in the scene graph
 *
 *
 * Traverse scenegraph starting at _root, looking for a node whose id
 * is _node_idx . Returns 0 if node wasn't found.
 *
 * @param _root     The root node where the traversal starts (not necessary the root node of the scenegraph)
 * @param _node_idx The node index this function should look for
 * @return 0 if the node was not found, otherwise a pointer to the node.
**/
BaseNode* find_node(BaseNode* _root, unsigned int _node_idx) 
{
  FindNodeAction action(_node_idx);
  traverse(_root, action);
  return action.node_ptr();
}


//----------------------------------------------------------------------------

/** \brief Find a node in the scene graph
 *
 *
 * Traverses scenegraph just like the find_node function, but includes hidden nodes.
 *
**/
BaseNode* find_hidden_node(BaseNode* _root, unsigned int _node_idx)
{
  FindNodeAction action(_node_idx);
  traverse_all(_root, action);
  return action.node_ptr();
}


//----------------------------------------------------------------------------


bool PickAction::operator()(BaseNode* _node)
{
  // If picking is disabled for the given Node, return here
  // As this is not a failure return true;
  if ( !_node->pickingEnabled() )
    return true;
  
  ACG::GLState::disable(GL_DITHER);

  if (state_.compatibilityProfile())
  {
    ACG::GLState::disable(GL_LIGHTING);
    ACG::GLState::shadeModel(GL_FLAT);
  }

  state_.pick_push_name ((GLuint) _node->id());

  _node->pick(state_, pickTarget_);

  state_.pick_pop_name ();
  return true;
}

bool PickAction::operator()(BaseNode* _node, GLState& /*_state*/) {
  return operator()(_node);
}


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

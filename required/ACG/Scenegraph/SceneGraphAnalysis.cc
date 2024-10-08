/*===========================================================================*\
*                                                                            *
*                              OpenFlipper                                   *
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
*                                                                            *
\*===========================================================================*/



#include "SceneGraphAnalysis.hh"

namespace ACG {
namespace SceneGraph {

/** \brief Analyze the SceneGraph <ACG/Scenegraph/SceneGraphAnalysis.hh>
 *
 * @param _root       Root node of the scenegraph
 * @param _maxPasses  Returns the maximal number of render passes
 * @param _bbmin      Returns lower left of the bounding box
 * @param _bbmax      Returns upper right of the bounding box
 */
void analyzeSceneGraph(ACG::SceneGraph::BaseNode* _root,
                       unsigned int&              _maxPasses,
                       ACG::Vec3d&                _bbmin,
                       ACG::Vec3d&                _bbmax)
{
  // Get max number of render pass
  // Single pass action, this info is static during multipass
  ACG::SceneGraph::MultiPassInfoAction info_act;
  ACG::SceneGraph::traverse(_root, info_act);

  _maxPasses = info_act.getMaxPasses();

  // get scene size
  // Single pass action, as the bounding box is not influenced by multipass traversal
  ACG::SceneGraph::BoundingBoxAction act;
  ACG::SceneGraph::traverse(_root, act);

  _bbmin = (ACG::Vec3d) act.bbMin();
  _bbmax = (ACG::Vec3d) act.bbMax();
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

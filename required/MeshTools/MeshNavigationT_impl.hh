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






//=============================================================================
//
//  IMPLEMENTATION
//
//=============================================================================

#define MESHNAVIGATIONT_C

//== INCLUDES =================================================================

#include "MeshNavigationT.hh"

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/MathDefs.hh>

#include <queue>
//== NAMESPACES ===============================================================

namespace MeshNavigation {

//== IMPLEMENTATION ==========================================================

template < typename MeshT >
inline
typename MeshT::HalfedgeHandle
opposite_halfedge(MeshT& _mesh , typename MeshT::HalfedgeHandle _he) {
   typename MeshT::VertexOHalfedgeIter opp_he( _mesh , _he);
   ++opp_he;
   ++opp_he;
   ++opp_he;
   return (*opp_he);
}

template < typename MeshT >
inline
typename MeshT::VertexHandle
findClosestBoundary(MeshT* _mesh , typename MeshT::VertexHandle _vh){

  //add visited property
  OpenMesh::VPropHandleT< bool > visited;
  _mesh->add_property(visited,"Visited Property" );
  
  //init visited property
  typename MeshT::VertexIter v_it, v_end = _mesh->vertices_end();
  for( v_it = _mesh->vertices_begin(); v_it != v_end; ++v_it )
    _mesh->property( visited, *v_it ) = false;

  std::queue< typename MeshT::VertexHandle > queue;
  queue.push( _vh );

  while(!queue.empty()){
    typename MeshT::VertexHandle vh = queue.front();
    queue.pop();
    if (_mesh->property(visited, vh)) continue;

    for (typename MeshT::VertexOHalfedgeIter voh_it(*_mesh,vh); voh_it.is_valid(); ++voh_it){

      if ( _mesh->is_boundary(*voh_it) ){
        _mesh->remove_property(visited);
        return _mesh->to_vertex_handle(*voh_it);
      }else{
        queue.push( _mesh->to_vertex_handle(*voh_it) );
      }
    }
    _mesh->property(visited, vh) = true;
  }

  _mesh->remove_property(visited);
  return typename MeshT::VertexHandle(-1);
}


//=============================================================================
} // MeshNavigation Namespace 
//=============================================================================

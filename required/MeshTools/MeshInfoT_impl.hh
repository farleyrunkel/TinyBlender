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

#define MESHINFOT_C

//== INCLUDES =================================================================

#include "MeshInfoT.hh"

#include <iostream>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/MathDefs.hh>


//== NAMESPACES ===============================================================

namespace MeshInfo {

//== IMPLEMENTATION ===========================================================

template < typename MeshT >
inline
int boundaryCount(MeshT* _mesh ) {
  
  int numOfBounds = 0;
  
  OpenMesh::HPropHandleT< bool > visited;
  _mesh->add_property(visited);
  
  typename MeshT::VertexHandle last;
  const typename MeshT::VertexHandle begin;
  
  //Iteration �ber Halfedges
  for (typename MeshT::HalfedgeIter he_it=_mesh->halfedges_begin(); he_it!=_mesh->halfedges_end() ; ++he_it ) 
    _mesh->property(visited,*he_it) = false;
  
  for (typename MeshT::HalfedgeIter he_it=_mesh->halfedges_begin(); he_it!=_mesh->halfedges_end() ; ++he_it ) {
    if ( _mesh->property(visited,*he_it) )
      continue;
    
    if( !_mesh->is_boundary(*he_it ) )
      continue;
     
    _mesh->property(visited,*he_it) = true;
    typename MeshT::HalfedgeHandle he = _mesh->next_halfedge_handle(*he_it);
    
    while( _mesh->is_boundary(*he_it) && ! _mesh->property(visited,he) ) {
      _mesh->property(visited,he) = true;
      he = _mesh->next_halfedge_handle(he);
    }
    
    ++numOfBounds;
    
  }
  
  _mesh->remove_property(visited);
  
  return numOfBounds;
}
  
//=============================================================================

template < typename MeshT >
inline
int componentCount(MeshT* _mesh ) {
  
  int numOfComps = 0;
  
  OpenMesh::VPropHandleT< bool > visited;
  _mesh->add_property(visited);
  
  typename MeshT::VertexIter v_it;
  typename MeshT::VertexIter v_end = _mesh->vertices_end();

  //iterate over all vertices
  for (v_it = _mesh->vertices_begin(); v_it != v_end; ++v_it)
    _mesh->property(visited, *v_it) = false;

  typename MeshT::VertexHandle vh;
  typename MeshT::VertexIter current_pos = _mesh->vertices_begin();

  while( true ){
    //find an unvisited vertex
    bool found = false;
    for (v_it = current_pos ; v_it != v_end; ++v_it)
      if ( !_mesh->property(visited, *v_it) ){
        found = true;
        vh = *v_it;
        _mesh->property(visited, *v_it) = true;
        current_pos = v_it;
        break;
      }

    //if none was found -> finished
    if (!found) break;

    numOfComps++;

    std::vector< typename MeshT::VertexHandle > handles;
    handles.push_back( vh );

    //grow from found vertex
    while( ! handles.empty() ){
      typename MeshT::VertexHandle current = handles.back();
      handles.pop_back();

      typename MeshT::VertexVertexIter vv_it;
  
      for (vv_it=_mesh->vv_iter( current ); vv_it.is_valid(); ++vv_it)
        if ( !_mesh->property(visited, *vv_it) ){
          _mesh->property(visited, *vv_it) = true;
          handles.push_back( *vv_it );
        }
    }
  }
  
  _mesh->remove_property(visited);
  
  return numOfComps;
}
  
//=============================================================================

template < typename MeshT >
inline    
void getBoundingBox( MeshT*   _mesh,
                     typename MeshT::Point& _min , 
                     typename MeshT::Point& _max) {
  if ( _mesh->n_vertices() == 0 ) {
    std::cerr << "Unable to compute Bounding Box: No points in Mesh!" << std::endl; 
  }
  // Use any point as initial value                       
  _min = _mesh->point(_mesh->vertex_handle(0));
  _max = _mesh->point(_mesh->vertex_handle(0));
  
  for (typename MeshT::VertexIter v_it = _mesh->vertices_begin() ; v_it != _mesh->vertices_end() ; ++v_it ) {
    _min.minimize( _mesh->point(*v_it) );
    _max.maximize( _mesh->point(*v_it) );
  }
  
}

//=============================================================================

template < typename MeshT >
inline    
typename MeshT::Point
cog ( const MeshT* _mesh ) {
      typename MeshT::ConstVertexIter v_it, v_end=_mesh->vertices_end();
      typename MeshT::Point cog(0.0,0.0,0.0);
      
      for (v_it = _mesh->vertices_begin(); v_it != v_end ; ++v_it)
         cog += _mesh->point(*v_it);
      cog = 1.0 / (typename MeshT::Scalar)_mesh->n_vertices()  * cog;
      
      return cog;
}

//=============================================================================
} // MeshInfo Namespace 
//=============================================================================

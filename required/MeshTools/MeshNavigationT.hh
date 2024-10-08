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
//
//=============================================================================


#ifndef MESHNAVIGATIONT_HH
#define MESHNAVIGATIONT_HH


/*! \file MeshNavigationT.hh
    \brief Functions for Navigation on a Mesh
    
*/

//== INCLUDES =================================================================

#include <vector>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace MeshNavigation {

//== DEFINITIONS ==============================================================

/**
   For Valence 6 Vertices Only!! iterates 3 Halfedges around from Vertex of _he. Returns the he on the other side
   @param _mesh  Mesh to work on
   @param _he Outgoing halfedge of Vertex to iterate around
   @return Halfedge on the opposite side
  */
template < typename MeshT >
inline
typename MeshT::HalfedgeHandle
opposite_halfedge(MeshT& _mesh , typename MeshT::HalfedgeHandle _he);

/** \brief Find the closest boundary vertex to another vertex.
 *
 * This function will start at a single vertex and walk across the edges of the mesh to find a
 * vertex on the boundary. The vertex at the boundary with the minimal number of edges to be traversed
 * to reach the starting vertex is returned. If no vertex is found (i.e. no boundary), an invalid
 * VertexHandle is returned.
 *
 * @param _mesh Mesh to work on
 * @param _vh   Start vertex handle
 * @return Closest vertex at the boundary
 */
template < typename MeshT >
typename MeshT::VertexHandle
findClosestBoundary(MeshT* _mesh , typename MeshT::VertexHandle _vh);

//=============================================================================
} // MeshNavigation Namespace 
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(MESHNAVIGATIONT_C)
#define MESHNAVIGATIONT_TEMPLATES
#include "MeshNavigationT_impl.hh"
#endif
//=============================================================================
#endif // MESHNAVIGATIONT_HH defined
//=============================================================================


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


#ifndef MESHINFOT_HH
#define MESHINFOT_HH


/*! \file MeshNavigationT.hh
    \brief Functions for getting information about a mesh
    
*/

//== INCLUDES =================================================================

#include <vector>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace MeshInfo {

//== DEFINITIONS ==============================================================

/** \brief Count boundaries
 * 
 * Get the number of independent boundaries
 *
 */
template < typename MeshT >
inline
int boundaryCount(MeshT* _mesh );

/** \brief Count components
 * 
 * Get the number of independent components
 *
 */
template < typename MeshT >
inline
int componentCount(MeshT* _mesh );

/**
    Calculate the axis aligned Boundingbox of a given Mesh
    @param _mesh Mesh to work on
    @param _min lower left corner of the box
    @param _max upper right corner of the box
*/ 
template < typename MeshT >
inline    
void getBoundingBox( MeshT*   _mesh,
                     typename MeshT::Point& _min , 
                     typename MeshT::Point& _max);

/** Compute the center of gravity for a given Mesh
 * @param _mesh The mesh to work on
 * @return The cog of the mesh
 */
template < typename MeshT >
inline    
typename MeshT::Point
cog ( const MeshT* _mesh );

//=============================================================================
} // MeshInfo Namespace 
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(MESHINFOT_C)
#define MESHINFOT_TEMPLATES
#include "MeshInfoT_impl.hh"
#endif
//=============================================================================
#endif // MESHINFOT_HH defined
//=============================================================================


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


#ifndef CURVATURE_HH
#define CURVATURE_HH


/*! \file Curvature.hh
    \brief Functions for calculating curvatures
    
*/

//== INCLUDES =================================================================

#include <vector>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace curvature {

//== DEFINITIONS ==============================================================

/*! compute consistent dirscrete gaussian curvature (vertex is a small sphere patch, edges are small cylinders)
*/
template< typename MeshT >
double
gauss_curvature(MeshT& _mesh, const typename MeshT::VertexHandle& _vh);
   
/**
* Mean Curvature Normal Operator
* warning: if mean curvature < 0 _n points to the inside
* warning: if mean curvature = 0 -> no normal information
@param _n       mean_curvature(vit)*n(vit)
@param _area global vertex area
*/
template<class MeshT, class VectorT, class REALT>
void discrete_mean_curv_op( const MeshT&                           _m,
                                                const typename MeshT::VertexHandle& _vh, 
                                                VectorT&                         _n,
                                                REALT&                           _area );   

   
//=============================================================================
} 
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(CURVATURE_C)
#define CURVATURE_TEMPLATES
#include "CurvatureT_impl.hh"
#endif
//=============================================================================
#endif // CURVATURE_HH defined
//=============================================================================


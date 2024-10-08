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

#define CURVATURE_C

//== INCLUDES =================================================================

#include <ACG/Geometry/Algorithms.hh>
#include "Math_Tools/Math_Tools.hh"

#include <iostream>
#include <OpenMesh/Core/Geometry/MathDefs.hh>

#include <cmath>

//== NAMESPACES ===============================================================

namespace curvature {

//== IMPLEMENTATION ==========================================================

/*! compute consistent discrete gaussian curvature (vertex is a small sphere patch, edges are small cylinders)
*/
template< typename MeshT >
double
gauss_curvature(MeshT& _mesh, const typename MeshT::VertexHandle& _vh) {
   if (_mesh.status(_vh).deleted())
     return 0.0;
  
  double gauss_curv = 2.0 * M_PI;

  /*

  TODO: Check the boundary case.

  If the vertex is a boundary vertex
  if ( _mesh.is_boundary(_vh) )
    gauss_curv = M_PI;

  */

  const typename MeshT::Point p0 = _mesh.point(_vh);

  typename MeshT::CVOHIter voh_it( _mesh.cvoh_iter(_vh));

  if ( ! voh_it->is_valid() )
     return 0.0;
   


  for (auto voh_it : _mesh.voh_range(_vh))
  {
    typename MeshT::Point p1 = _mesh.point(voh_it.to());
    typename MeshT::Point p2 = _mesh.point(voh_it.opp().next().to());

    gauss_curv -= acos(OpenMesh::sane_aarg( ((p1-p0).normalize() | (p2-p0).normalize()) ));
  }

  return gauss_curv;
}


template<class MeshT, class VectorT, class REALT>
void discrete_mean_curv_op( const MeshT& _m, const typename MeshT::VertexHandle& _vh, VectorT& _n, REALT& _area )
{
  _n = VectorT(0,0,0);
  _area = 0.0;

   if ( ! _m.cvoh_iter(_vh)->is_valid() )
     return;
   
  for(auto voh_it : _m.voh_range(_vh))
  {
    if ( voh_it.edge().is_boundary() )
      continue;
    
    const typename MeshT::Point p0 = _m.point( _vh );
    const typename MeshT::Point p1 = _m.point(voh_it.to());
    const typename MeshT::Point p2 = _m.point(voh_it.prev().from());

    const typename MeshT::Point p3 = _m.point(voh_it.opp().next().to());

    const REALT alpha = acos( OpenMesh::sane_aarg((p0-p2).normalize() | (p1-p2).normalize()) );
    const REALT beta  = acos( OpenMesh::sane_aarg((p0-p3).normalize() | (p1-p3).normalize()) );
      
    REALT cotw = 0.0;
    
    if ( !OpenMesh::is_eq(alpha,M_PI/2.0) )
      cotw += (REALT(1.0))/tan(alpha);
    
    if ( !OpenMesh::is_eq(beta,M_PI/2.0) )
      cotw += (REALT(1.0))/tan(beta);

#ifdef WIN32
	if ( OpenMesh::is_zero(cotw) )
		continue;
#else
    if ( OpenMesh::is_zero(cotw) || std::isinf(cotw) )
		continue;
#endif

    // calculate area
    const int obt = ACG::Geometry::isObtuse(p0,p1,p2);
    if(obt == 0)
    {
      REALT gamma = acos( OpenMesh::sane_aarg((p0-p1).normalize() | (p2-p1).normalize()) );
      
      REALT tmp = 0.0;
      if ( !OpenMesh::is_eq(alpha,M_PI/2.0) )
        tmp += (p0-p1).sqrnorm()*1.0/tan(alpha);
    
      if ( !OpenMesh::is_eq(gamma,M_PI/2.0) )
        tmp += (p0-p2).sqrnorm()*1.0/tan(gamma);
      
#ifdef WIN32
	  if ( OpenMesh::is_zero(tmp) )
		continue;
#else
      if ( OpenMesh::is_zero(tmp) || std::isinf(tmp) )
		continue;
#endif

      _area += 0.125*( tmp );
    }
    else
    {
      if(obt == 1)
      {
         _area += ((p1-p0) % (p2-p0)).norm() * 0.5 * 0.5;
      }
      else
      {
         _area += ((p1-p0) % (p2-p0)).norm() * 0.5 * 0.25;
      }
    }
    
    _n += ((p0-p1)*cotw);

    // error handling
    //if(_area < 0) std::cerr << "error: triangle area < 0\n";
//    if(isnan(_area))
//    {
//      REALT gamma = acos( ((p0-p1).normalize() | (p2-p1).normalize()) );

/*
      std::cerr << "***************************\n";
      std::cerr << "error : trianlge area = nan\n";
      std::cerr << "alpha : " << alpha << std::endl;
      std::cerr << "beta  : " << beta  << std::endl;
      std::cerr << "gamma : " << gamma << std::endl;
      std::cerr << "cotw  : " << cotw  << std::endl;
      std::cerr << "normal: " << _n << std::endl;
      std::cerr << "p0    : " << p0 << std::endl;
      std::cerr << "p1    : " << p1 << std::endl;
      std::cerr << "p2    : " << p2 << std::endl;
      std::cerr << "p3    : " << p3 << std::endl;
      std::cerr << "***************************\n";
*/
//    }
  }

  _n /= 4.0*_area;
}



//=============================================================================
} // curvature Namespace 
//=============================================================================

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
//  CLASS ICP
//
//=============================================================================


#ifndef ICP_HH
#define ICP_HH

/*! \file ICP.hh
    \brief Classes for ICP Algorithm
    
    Classes for doing Principal component analysis
*/

//== INCLUDES =================================================================

#include <gmm/gmm.h>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

/// Namespace for ICP
namespace ICP {
 
//== CLASS DEFINITION =========================================================
  
/** \brief Compute rigid transformation from first point set to second point set
   * 
   * Compute ICP Parameters ( No iteration is done ) Points are unchanged, only parameters are computed.
   *
   * To transform pointset 1 into pointset 2 do the folowing:\n
   *  - substract cog1 from pointset 1 \n
   *  - scale points with 1/scale1 \n
   *  - rotate with given rotation \n
   *  - scale with scale2 \n
   *  - add cog2  \n
   *
   * @param _points1 first set of points
   * @param _points2 second set of points
   * @param  _cog1 center of gravity first point set
   * @param  _cog2 center of gravity second point set
   * @param _scale1 scaling factor of first point set
   * @param _scale2 scaling factor of second point set
   * @param _rotation Rotation between point sets ( rotation(_points1) -> _points2
   * 
   *
*/
template < typename VectorT , typename QuaternionT >
void icp(const std::vector< VectorT >& _points1 , const std::vector< VectorT >& _points2  , VectorT& _cog1 ,  VectorT& _cog2 , double& _scale1 , double& _scale2 , QuaternionT& _rotation );

/** \brief Compute rigid transformation from first point set to second point set without scaling
   *
   * Compute ICP Parameters ( No iteration is done ) Points are unchanged, only parameters are computed.
   *
   * To transform pointset 1 into pointset 2 do the folowing:\n
   *  - substract cog1 from pointset 1 \n
   *  - rotate with given rotation \n
   *  - add cog2  \n
   *
   * @param _points1 first set of points
   * @param _points2 second set of points
   * @param  _cog1 center of gravity first point set
   * @param  _cog2 center of gravity second point set
   * @param _rotation Rotation between point sets ( rotation(_points1) -> _points2
   *
   *
*/
template < typename VectorT , typename QuaternionT >
void icp(const std::vector< VectorT >& _points1 , const std::vector< VectorT >& _points2  , VectorT& _cog1 ,  VectorT& _cog2 , QuaternionT& _rotation );

//=============================================================================
} //namespace ICP
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(ICP_C)
#define ICP_TEMPLATES
#include "ICPT_impl.hh"
#endif
//=============================================================================
#endif // ICP_HH defined
//=============================================================================


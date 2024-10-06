/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2016, RWTH-Aachen University                 *
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
//  CLASS PrincipalAxisNode - IMPLEMENTATION
//
//=============================================================================

#define ACG_PRINCIPAL_AXIS_NODE_C

//== INCLUDES =================================================================

#include "PrincipalAxisNode.hh"
#include <ACG/GL/gl.hh>


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ========================================================== 


template<class VectorT>
void 
PrincipalAxisNode::
set_vector( unsigned int _i, const Vec3d _p, const VectorT& _v)
{
  Matrix4x4T<double> m;

  m(0,0) = _v[0];
  m(1,1) = _v[1];
  m(2,2) = _v[2];
  m(0,1) = m(1,0) = _v[3];
  m(1,2) = m(2,1) = _v[4];
  m(0,2) = m(2,0) = _v[5];

  set_matrix(_i, _p, m);
}


//----------------------------------------------------------------------------


template<class MatrixT>
void
PrincipalAxisNode::
set_matrix( unsigned int _i, const Vec3d _p, const MatrixT& _m)
{
  // create matrix
  double s[3][3];

  // copy values
  for(unsigned int i=0; i<3; ++i)
    for(unsigned int j=0; j<3; ++j)
      s[i][j] = _m(i,j);

  // compute eigenvalues and eigenvectors
  double Q[3][3];
  double D[3][3];

  diagonalize(s, Q, D);

  std::vector<double> eigval;
  for (int i = 0; i < 3; ++i)
    eigval.push_back(D[i][i]);


  unsigned int pstress_idx[3];
  if( fabs(eigval[0]) >= fabs(eigval[1]) && fabs(eigval[0]) >= fabs(eigval[2])) pstress_idx[0] = 0;
  if( fabs(eigval[1]) >= fabs(eigval[0]) && fabs(eigval[1]) >= fabs(eigval[2])) pstress_idx[0] = 1;
  if( fabs(eigval[2]) >= fabs(eigval[0]) && fabs(eigval[2]) >= fabs(eigval[1])) pstress_idx[0] = 2;

  unsigned int i0 = (pstress_idx[0] + 1)%3;
  unsigned int i1 = (pstress_idx[0] + 2)%3;
  if( fabs(eigval[i0]) > fabs(eigval[i1]) )
  {
    pstress_idx[1] = i0;
    pstress_idx[2] = i1;
  }
  else
  {
    pstress_idx[1] = i1;
    pstress_idx[2] = i0;
  }

  Vec3d a[3];
  bool  sign[3];

  for(unsigned int j=0; j<3; ++j)
  {
    // set sign
    if( eigval[ pstress_idx[j]] < 0 )
      sign[j] = false;
    else
      sign[j] = true;

    for(unsigned int k=0; k<3; ++k)
      a[j][k] = eigval[ pstress_idx[j]]*Q[k][pstress_idx[j]];
  }

  set(_i,
      PrincipalComponent(_p,
       a[0],
       a[1],
       a[2],
       sign[0],
       sign[1],
       sign[2] ) );

}


//----------------------------------------------------------------------------


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

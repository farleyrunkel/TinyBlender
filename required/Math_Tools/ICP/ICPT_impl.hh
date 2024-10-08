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
//  CLASS ICP - IMPLEMENTATION
//
//=============================================================================

#define ICP_C

//== INCLUDES =================================================================

#include "ICP.hh"
#include <cfloat>


//== NAMESPACES ===============================================================

namespace ICP {

//== IMPLEMENTATION ==========================================================

template < typename VectorT >
inline
VectorT  
center_of_gravity(const std::vector< VectorT >& _points ) {
   VectorT cog(0.0,0.0,0.0);
   
   for (uint i = 0 ; i < _points.size() ; ++i ) {
     cog = cog + _points[i];
   }
   
   cog = cog / ( typename VectorT::value_type )_points.size();

   return cog;
}


template < typename VectorT , typename QuaternionT >
void 
icp(const std::vector< VectorT >& _points1 , const std::vector< VectorT >& _points2  , VectorT& _cog1 ,  VectorT& _cog2 , double& _scale1 , double& _scale2 , QuaternionT& _rotation )
{
   // compute Mean of Points
   _cog1 = center_of_gravity(_points1);
   _cog2 = center_of_gravity(_points2);
   
   
   VectorT diff;
   _scale1 = 0.0;
   _scale2 = 0.0;
   for ( uint i = 0 ; i < _points1.size() ; ++i ) {
       diff = _points1[i] - _cog1;
       _scale1 = std::max( _scale1 , sqrt( diff.norm() ) );
       diff = _points2[i] - _cog2;
       _scale2 = std::max( _scale2 , sqrt( diff.norm() ) );       
   }
   
   double Sxx = 0.0;
   double Sxy = 0.0;
   double Sxz = 0.0;
   
   double Syx = 0.0;
   double Syy = 0.0;
   double Syz = 0.0;
   
   double Szx = 0.0;
   double Szy = 0.0;
   double Szz = 0.0;
   
    for ( uint i = 0 ; i < _points1.size() ; ++i  ) {
      Sxx += ( _points1[i][0] - _cog1[0] ) * ( _points2[i][0] - _cog2[0] );
      Sxy += ( _points1[i][0] - _cog1[0] ) * ( _points2[i][1] - _cog2[1] );
      Sxz += ( _points1[i][0] - _cog1[0] ) * ( _points2[i][2] - _cog2[2] );
      
      Syx += ( _points1[i][1] - _cog1[1] ) * ( _points2[i][0] - _cog2[0] );
      Syy += ( _points1[i][1] - _cog1[1] ) * ( _points2[i][1] - _cog2[1] );
      Syz += ( _points1[i][1] - _cog1[1] ) * ( _points2[i][2] - _cog2[2] );
      
      Szx += ( _points1[i][2] - _cog1[2] ) * ( _points2[i][0] - _cog2[0] );
      Szy += ( _points1[i][2] - _cog1[2] ) * ( _points2[i][1] - _cog2[1] );
      Szz += ( _points1[i][2] - _cog1[2] ) * ( _points2[i][2] - _cog2[2] );
   }
   
   const double scale = _scale1 * _scale2;
   
   Sxx = Sxx * 1.0 / scale;
   Sxy = Sxy * 1.0 / scale;
   Sxz = Sxz * 1.0 / scale;
   Syx = Syx * 1.0 / scale;
   Syy = Syy * 1.0 / scale;
   Syz = Syz * 1.0 / scale;
   Szx = Szx * 1.0 / scale;
   Szy = Szy * 1.0 / scale;
   Szz = Szz * 1.0 / scale;
   
   gmm::dense_matrix<double> N; gmm::resize(N,4,4); gmm::clear(N);
   N(0,0) = Sxx + Syy + Szz;
   N(0,1) = Syz - Szy;
   N(0,2) = Szx - Sxz;
   N(0,3) = Sxy - Syx;
   
   N(1,0) = Syz - Szy;
   N(1,1) = Sxx - Syy - Szz;
   N(1,2) = Sxy + Syx;
   N(1,3) = Szx + Sxz;
   
   N(2,0) = Szx - Sxz;
   N(2,1) = Sxy + Syx;
   N(2,2) = -Sxx + Syy - Szz;
   N(2,3) = Syz + Szy;
   
   N(3,0) = Sxy - Syx;
   N(3,1) = Szx + Sxz;
   N(3,2) = Syz + Szy;
   N(3,3) = -Sxx - Syy + Szz;
   
   std::vector< double > eigval; eigval.resize(4);
   gmm::dense_matrix< double > eigvect; gmm::resize(eigvect, 4,4); gmm::clear(eigvect);
   gmm::symmetric_qr_algorithm(N, eigval, eigvect);
   
   int gr = 0;
   double max = -FLT_MAX;
   for (int i = 0; i < 4; i++)
   if (eigval[i] > max)
   {
      gr = i;
      max = eigval[i];
   }
   
   _rotation[0] = eigvect(0,gr);
   _rotation[1] = eigvect(1,gr);
   _rotation[2] = eigvect(2,gr);
   _rotation[3] = eigvect(3,gr);
   
   _scale1 *= _scale1;
   _scale2 *= _scale2;
}

template < typename VectorT , typename QuaternionT >
void
icp(const std::vector< VectorT >& _points1 , const std::vector< VectorT >& _points2  , VectorT& _cog1 ,  VectorT& _cog2 , QuaternionT& _rotation )
{
   // compute Mean of Points
   _cog1 = center_of_gravity(_points1);
   _cog2 = center_of_gravity(_points2);

   double Sxx = 0.0;
   double Sxy = 0.0;
   double Sxz = 0.0;
   double Syx = 0.0;
   double Syy = 0.0;
   double Syz = 0.0;
   double Szx = 0.0;
   double Szy = 0.0;
   double Szz = 0.0;

    for ( uint i = 0 ; i < _points1.size() ; ++i  ) {
      Sxx += ( _points1[i][0] - _cog1[0] ) * ( _points2[i][0] - _cog2[0] );
      Sxy += ( _points1[i][0] - _cog1[0] ) * ( _points2[i][1] - _cog2[1] );
      Sxz += ( _points1[i][0] - _cog1[0] ) * ( _points2[i][2] - _cog2[2] );

      Syx += ( _points1[i][1] - _cog1[1] ) * ( _points2[i][0] - _cog2[0] );
      Syy += ( _points1[i][1] - _cog1[1] ) * ( _points2[i][1] - _cog2[1] );
      Syz += ( _points1[i][1] - _cog1[1] ) * ( _points2[i][2] - _cog2[2] );

      Szx += ( _points1[i][2] - _cog1[2] ) * ( _points2[i][0] - _cog2[0] );
      Szy += ( _points1[i][2] - _cog1[2] ) * ( _points2[i][1] - _cog2[1] );
      Szz += ( _points1[i][2] - _cog1[2] ) * ( _points2[i][2] - _cog2[2] );
   }

   gmm::dense_matrix<double> N; gmm::resize(N,4,4); gmm::clear(N);
   N(0,0) = Sxx + Syy + Szz;
   N(0,1) = Syz - Szy;
   N(0,2) = Szx - Sxz;
   N(0,3) = Sxy - Syx;

   N(1,0) = Syz - Szy;
   N(1,1) = Sxx - Syy - Szz;
   N(1,2) = Sxy + Syx;
   N(1,3) = Szx + Sxz;

   N(2,0) = Szx - Sxz;
   N(2,1) = Sxy + Syx;
   N(2,2) = -Sxx + Syy - Szz;
   N(2,3) = Syz + Szy;

   N(3,0) = Sxy - Syx;
   N(3,1) = Szx + Sxz;
   N(3,2) = Syz + Szy;
   N(3,3) = -Sxx - Syy + Szz;

   std::vector< double > eigval; eigval.resize(4);
   gmm::dense_matrix< double > eigvect; gmm::resize(eigvect, 4,4); gmm::clear(eigvect);
   gmm::symmetric_qr_algorithm(N, eigval, eigvect);

   int gr = 0;
   double max = -FLT_MAX;
   for (int i = 0; i < 4; i++)
   if (eigval[i] > max)
   {
      gr = i;
      max = eigval[i];
   }

   _rotation[0] = eigvect(0,gr);
   _rotation[1] = eigvect(1,gr);
   _rotation[2] = eigvect(2,gr);
   _rotation[3] = eigvect(3,gr);

}


//=============================================================================

} //namespace ICP

//=============================================================================

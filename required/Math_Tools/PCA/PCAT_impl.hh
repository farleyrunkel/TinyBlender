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
//  CLASS PCA - IMPLEMENTATION
//
//=============================================================================

#define PCA_C

//== INCLUDES =================================================================

#include "PCA.hh"

extern "C" {

    typedef void (*U_fp)(...);

    int dsyev_( char *jobz, char *uplo, long int *n, double *a,
                long int *lda, double *w, double *work, long int *lwork,
                long int *info);

}

// typedef gmm::dense_matrix< double > Matrix;
// typedef std::vector< double > Vector;

//! Module-wide pointer to working array
static double * p_work_double = 0;

//! Module-wide size of working array
static int m_workSize_double = 0;

//== NAMESPACES ===============================================================

namespace Pca {

//== IMPLEMENTATION ==========================================================

template < typename VectorT >
PCA< VectorT >::PCA( std::vector< VectorT >& _points , VectorT& _first , VectorT& _second , VectorT& _third)
{
  pca(_points,_first,_second,_third);
}

template < typename VectorT >
PCA< VectorT >::~PCA()
{
  if ( p_work_double != 0){
    delete [] p_work_double;
    p_work_double = 0;
    m_workSize_double = 0;
  }
}

template < typename VectorT >
inline
VectorT  
PCA< VectorT >::
center_of_gravity(const std::vector< VectorT >& _points ) {
   // compute cog of Points
   VectorT cog(0.0, 0.0, 0.0);
   
   for (uint i = 0 ; i < _points.size() ; ++i ) {
     cog = cog + _points[i];
   }
   
   cog = cog / ( typename VectorT::value_type )_points.size();

   return cog;
}

template < typename VectorT >
bool 
PCA< VectorT >::SymRightEigenproblem( Matrix &_mat_A, Matrix & _mat_VR,
                           std::vector< double > & _vec_EV )
{
    char jobz, uplo;
    long int n, lda;
    long int lwork, info;
    double size;

    jobz = 'V';
    uplo = 'U';
    n = gmm::mat_nrows( _mat_A );
    lda = n;

    info = 0;
    // compute optimal size of working array
    lwork = -1;
    dsyev_( &jobz, &uplo, &n, &_mat_A( 0, 0 ), &lda, &_vec_EV[ 0 ],
            &size, &lwork, &info );

    if( info != 0 )
        return false;


    if( (long int) size > m_workSize_double )
    {
        if( p_work_double)
            delete [] p_work_double;

        m_workSize_double = (long int) size;

        p_work_double= new double[ m_workSize_double ];
    }
    lwork = m_workSize_double;

    // compute eigenvalues / eigenvectors
    dsyev_( &jobz, &uplo, &n, &_mat_A( 0, 0 ), &lda, &_vec_EV[ 0 ],
            p_work_double, &lwork, &info );

    if( info != 0 )
        return false;

    // copy eigenvectors to matrix
    gmm::copy( _mat_A, _mat_VR );

    return true;
}


template < typename VectorT >
void 
PCA< VectorT >::pca(std::vector< VectorT >& _points , VectorT& _first , VectorT& _second , VectorT& _third)
{
   // compute Mean of Points
   const VectorT cog = center_of_gravity(_points);
   
   //build covariance Matrix
   Matrix points(3 , _points.size() );
  
   for ( uint i = 0 ; i < _points.size() ; ++i)
   {
       points(0 , i ) = ( _points[i] - cog) [0];
       points(1 , i ) = ( _points[i] - cog) [1];
       points(2 , i ) = ( _points[i] - cog) [2];
   }
  
  Matrix cov(3,3);
  gmm::mult(points,gmm::transposed(points),cov );
  
  Matrix EV(3,3);
  std::vector< double > ew(3);
  

  if ( !SymRightEigenproblem( cov, EV ,ew ) )
    std::cerr << "Error: Could not compute Eigenvectors for PCA" << std::endl;

  int maximum,middle,minimum;
  
  if ( (ew[0] > ew[1]) && (ew[0] > ew[2]) ) {
    maximum = 0;
  } else {
    if ( (ew[1] > ew[0]) && (ew[1] > ew[2]) )
         maximum = 1;
       else
         maximum = 2;
  }
  
  if ( (ew[0] < ew[1]) && (ew[0] < ew[2]) )
    minimum = 0;
  else if ( (ew[1] < ew[0]) && (ew[1] < ew[2]) )
         minimum = 1;
       else
         minimum = 2;         

  if ( (minimum != 0 ) && ( maximum != 0 ) )       
    middle = 0;
  else  
    if ( (minimum != 1 ) && ( maximum != 1 ) )       
      middle = 1;
    else
      middle = 2;
  
  _first  = VectorT( EV(0,maximum) , EV(1,maximum) , EV(2,maximum) );
  _second = VectorT( EV(0,middle)  , EV(1,middle)  , EV(2,middle ) );

  _first  = _first.normalize();
  _second = _second.normalize();
  _third  = _first % _second; 

  //remember the eigenvalues
  lastEigenValues_.clear();
  lastEigenValues_.push_back( ew[maximum] );
  lastEigenValues_.push_back( ew[middle] );
  lastEigenValues_.push_back( ew[minimum] );
}
//-----------------------------------------------------------------------------

template < typename VectorT >
inline
std::vector<double>&
PCA< VectorT >::getLastEigenValues()
{
  return lastEigenValues_;
}


//=============================================================================

} //namespace Pca

//=============================================================================

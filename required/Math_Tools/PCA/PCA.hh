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
//  CLASS PCA
//
//=============================================================================


#ifndef PCA_HH
#define PCA_HH

/*! \file PCA.hh
    \brief Classes for doing Principal component analysis
    
    Classes for doing Principal component analysis
*/

//== INCLUDES =================================================================

#include <gmm/gmm.h>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

/// Namespace for principal Component Analysis
namespace Pca {
 
//== CLASS DEFINITION =========================================================
	      
/** \class PCA PCA.hh </PCA.hh>
    \brief Class for principal component Analysis

    Class for principal component Analysis
*/

template < typename VectorT >
class PCA
{
  typedef gmm::dense_matrix<double>  Matrix;
  typedef std::vector< double >      Vector;

public:
   
  /// Constructor
  PCA() {}
  
  /** Extended constructor
    Parameters : see pca()
  */
  PCA( std::vector< VectorT >& _points, VectorT& _first , VectorT& _second , VectorT& _third);
 
  /// Destructor
  ~PCA();
  
  /**
    Compute center of gravity for a vector of points
    @param _points set of points
  */
  inline VectorT center_of_gravity(const std::vector< VectorT >& _points );
  
  /**
    Compute the principal component analysys for a vector of points
    @param _points set of points
    @param _first  First main axis
    @param _second second main axis
    @param _third  third main axis
  */
  void pca(std::vector< VectorT >& _points , VectorT& _first , VectorT& _second , VectorT& _third);


  std::vector<double>& getLastEigenValues();

  bool SymRightEigenproblem( Matrix &_mat_A, Matrix & _mat_VR,
                             std::vector< double > & _vec_EV );
private:

  std::vector<double> lastEigenValues_;

};


//=============================================================================
} //namespace Pca
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(PCA_C)
#define PCA_TEMPLATES
#include "PCAT_impl.hh"
#endif
//=============================================================================
#endif // PCA_HH defined
//=============================================================================


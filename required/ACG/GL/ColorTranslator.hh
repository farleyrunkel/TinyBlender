/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
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
 *                                                                           *
\*===========================================================================*/


//=============================================================================
//
//  CLASS ColorTranslator
//
//=============================================================================

#ifndef ACG_COLORTRANSLATOR_HH
#define ACG_COLORTRANSLATOR_HH


//== INCLUDES =================================================================


#include "gl.hh"
#include <ACG/Config/ACGDefines.hh>
#include <ACG/Math/VectorT.hh>


//== NAMESPACES ===============================================================


namespace ACG {

class GLState;

//== CLASS DEFINITION =========================================================


/** This class can be used to translate colors to integers. Use this
    e.g. for color index picking, i.e. drawing objects using a color
    corresponding to an unique index.
*/
class ACGDLLEXPORT ColorTranslator
{
public:
   
  /// Default constructor.
  ColorTranslator() : initialized_(false),
                      red_bits_(0), green_bits_(0), blue_bits_(0), alpha_bits_(0),
                      red_mask_(0), green_mask_(0), blue_mask_(0), alpha_mask_(0),
                      red_shift_(0), green_shift_(0), blue_shift_(0), alpha_shift_(0),
                      red_round_(0), green_round_(0), blue_round_(0), alpha_round_(0)
   {};

   /// Destructor.
  ~ColorTranslator() {};

  
  /// init (takes current GL context to get the component sizes)
  /// Can't use constructor as we might not have a context at this point.
  void initialize(ACG::GLState*);

  /// has it been initialized?
  bool initialized() const { return initialized_; }


  /// index -> color (one buffer)
  Vec4uc index2color(const size_t _idx) const;

  /// color -> index (one buffer)
  size_t color2index(const Vec4uc _rgba) const;


  /// returns maximal convertible index
  size_t max_index() const;
  
  
private:

  bool     initialized_;
  GLint    red_bits_, green_bits_, blue_bits_, alpha_bits_;
  GLuint   red_mask_, green_mask_, blue_mask_, alpha_mask_;
  GLuint   red_shift_, green_shift_, blue_shift_, alpha_shift_;
  GLuint   red_round_, green_round_, blue_round_, alpha_round_;
};


//=============================================================================
} // namespace ACG
//=============================================================================
#endif // ACG_COLORTRANSLATOR_HH defined
//=============================================================================


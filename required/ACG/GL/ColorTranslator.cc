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
//  CLASS ColorTranslator - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "ColorTranslator.hh"
#include <ACG/GL/GLState.hh>
#include <iostream>
#include <limits>


//== NAMESPACES ===============================================================


namespace ACG {


//== IMPLEMENTATION ========================================================== 


void
ColorTranslator::
initialize(ACG::GLState* _state)
{
    if(_state->compatibilityProfile())
    {
        glGetIntegerv( GL_RED_BITS,   &red_bits_   );
        glGetIntegerv( GL_GREEN_BITS, &green_bits_ );
        glGetIntegerv( GL_BLUE_BITS,  &blue_bits_  );
        glGetIntegerv( GL_ALPHA_BITS,  &alpha_bits_ );
    }
    else
    {   //only up to 8 bits are supported as of now, so hardcode them for core profiles
        red_bits_   = 8;
        green_bits_ = 8;
        blue_bits_  = 8;
        alpha_bits_ = 8;
    }
  // We currently only support up to 8 bits per channel (
  if (red_bits_   > 8)  red_bits_   = 8;
  if (green_bits_ > 8)  green_bits_ = 8;
  if (blue_bits_  > 8)  blue_bits_  = 8;
  if (alpha_bits_ > 8)  alpha_bits_ = 8;

  // Compute the mask to extract the component
  red_mask_    = ((1 << red_bits_)   - 1);
  green_mask_  = ((1 << green_bits_) - 1);
  blue_mask_   = ((1 << blue_bits_)  - 1);
  alpha_mask_  = ((1 << alpha_bits_) - 1);

  // Shift required to move the component to the lowest bits
  red_shift_   = 8 - red_bits_;
  green_shift_ = 8 - green_bits_;
  blue_shift_  = 8 - blue_bits_;
  alpha_shift_ = 8 - alpha_bits_;


  red_round_   = 1 << (red_shift_   - 1);
  green_round_ = 1 << (green_shift_ - 1);
  blue_round_  = 1 << (blue_shift_  - 1);
  alpha_round_ = 1 << (alpha_shift_ - 1);

  initialized_ = true;
}

  
//-----------------------------------------------------------------------------


Vec4uc
ColorTranslator::
index2color(const size_t _idx) const
{
  assert(initialized());
  unsigned char  r, g, b, a;
  
  // Make sure that the number fits
  if ( _idx > std::numeric_limits<unsigned int>::max() ) {
    std::cerr << "Can't convert index " << _idx << " to RGBA. Number too large for unsigned int \n";
    return Vec4uc(0, 0, 0, 0);
  }

  unsigned int idx = ( static_cast<unsigned int>(_idx) + 1);

  b = ((idx & blue_mask_)  << blue_shift_)  | blue_round_;  
  idx >>= blue_bits_;
  g = ((idx & green_mask_) << green_shift_) | green_round_;  
  idx >>= green_bits_;
  r = ((idx & red_mask_)   << red_shift_)   | red_round_;  
  idx >>= red_bits_;
  a = ((idx & alpha_mask_) << alpha_shift_) | alpha_round_;
  idx >>= alpha_bits_;

  if (!idx) 
    return  Vec4uc(r, g, b, a);

  else 
  {
    std::cerr << "Can't convert index " << _idx << " to RGBA\n";
    return Vec4uc(0, 0, 0, 0);
  }
}

  
//-----------------------------------------------------------------------------


size_t
ColorTranslator::
color2index(const Vec4uc _rgba) const
{


  assert(initialized());

  // Work internally with#include <iostream> unsigned int for now
  unsigned int result;

  // Combine the single unsigned chars according to masks
  result =   _rgba[3] >> alpha_shift_;
  result <<= red_bits_;
  result |=  _rgba[0] >> red_shift_;
  result <<= green_bits_;
  result |=  _rgba[1] >> green_shift_;
  result <<= blue_bits_;
  result |=  _rgba[2] >> blue_shift_;    

  // Return size_t Here
  return ( static_cast<size_t>(result-1) );
}


//-----------------------------------------------------------------------------


size_t
ColorTranslator::max_index() const 
{
  assert(initialized());
  if (red_bits_+green_bits_+blue_bits_+alpha_bits_ == 32)
    return 0xffffffff;
  else
    return (1 << (red_bits_+green_bits_+blue_bits_+alpha_bits_))-1;
}


//=============================================================================
} // namespace ACG
//=============================================================================


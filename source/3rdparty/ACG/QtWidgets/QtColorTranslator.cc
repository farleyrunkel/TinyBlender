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
//  CLASS QtColorTranslator - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "QtColorTranslator.hh"
#include <iostream>
#include <cassert>


//== NAMESPACES ===============================================================

namespace ACG {


//== IMPLEMENTATION ========================================================== 


void
QtColorTranslator::initialize()
{
  glGetIntegerv( GL_RED_BITS,   &redBits_ );
  glGetIntegerv( GL_GREEN_BITS, &greenBits_ );
  glGetIntegerv( GL_BLUE_BITS,  &blueBits_ );    

  if (redBits_ > 8)   redBits_ = 8;
  if (greenBits_ > 8) greenBits_ = 8;
  if (blueBits_ > 8)  blueBits_ = 8;

  redMask_    = ((1 << redBits_)   - 1);
  greenMask_  = ((1 << greenBits_) - 1);
  blueMask_   = ((1 << blueBits_)  - 1);    
  redShift_   = 8 - redBits_;
  greenShift_ = 8 - greenBits_;
  blueShift_  = 8 - blueBits_;    
  redRound_   = 1 << (redShift_   - 1);
  greenRound_ = 1 << (greenShift_ - 1);
  blueRound_  = 1 << (blueShift_  - 1);

  initialized_ = true;

  if (redBits_   > 8) std::cerr << "error: more than 8 red bits\n";
  if (greenBits_ > 8) std::cerr << "error: more than 8 green bits\n";
  if (blueBits_  > 8) std::cerr << "error: more than 8 blue bits\n";
}

  
//-----------------------------------------------------------------------------


bool
QtColorTranslator::index2color(unsigned int _idx, QRgb& _col) const
{
  assert(initialized());
  unsigned char  r, g, b;
  unsigned int idx(_idx+1);
  
  b = ((idx & blueMask_)  << blueShift_)  | blueRound_;  
  idx >>= blueBits_;
  g = ((idx & greenMask_) << greenShift_) | greenRound_;  
  idx >>= greenBits_;
  r = ((idx & redMask_)   << redShift_)   | redRound_;  
  idx >>= redBits_;
  
  if (!idx)
  {
    _col = qRgb(r, g, b);
    return true;
  }
  else {
    std::cerr << "Can't convert index " << _idx << " to RGB\n";
    _col = qRgb(0,0,0);
    return false;
  }
}

  
//-----------------------------------------------------------------------------


bool
QtColorTranslator::index2color(unsigned int _idx, QRgb& _fc, QRgb& _bc) const
{
  assert(initialized());
  unsigned char  r, g, b;
  unsigned int idx(_idx+1);
  
  b = ((idx & blueMask_)  << blueShift_)  | blueRound_;  
  idx >>= blueBits_;
  g = ((idx & greenMask_) << greenShift_) | greenRound_;  
  idx >>= greenBits_;
  r = ((idx & redMask_)   << redShift_)   | redRound_;  
  idx >>= redBits_;
  
  if (!idx)
  {
    _bc = qRgb(r, g, b);
    _fc = qRgb(0,0,0);
    return true;
  }
  else
  {
    _bc = qRgb(r, g, b);

    b = ((idx & blueMask_)  << blueShift_)  | blueRound_;  
    idx >>= blueBits_;
    g = ((idx & greenMask_) << greenShift_) | greenRound_;  
    idx >>= greenBits_;
    r = ((idx & redMask_)   << redShift_)   | redRound_;  
    idx >>= redBits_;

    if (!idx)
    {
      _fc = qRgb(r,g,b);
      return true;
    }
    else
    {
      std::cerr << "Can't convert index " << _idx << " to RGB\n";
      _bc = qRgb(0,0,0);
      _fc = qRgb(0,0,0);
      return false;
    }
  }
}

  
//-----------------------------------------------------------------------------


int
QtColorTranslator::color2index(QRgb _c) const
{
  assert(initialized());
  unsigned int result;

  result =   qRed(_c) >> redShift_;
  result <<= greenBits_;
  result |=  qGreen(_c) >> greenShift_;
  result <<= blueBits_;
  result |=  qBlue(_c) >> blueShift_;    

  return (result-1);
}


//-----------------------------------------------------------------------------


int
QtColorTranslator::color2index(QRgb _fc, QRgb _bc) const
{
  assert(initialized());
  unsigned int result;

  result =   qRed(_fc) >> redShift_;
  result <<= greenBits_;
  result |=  qGreen(_fc) >> greenShift_;
  result <<= blueBits_;
  result |=  qBlue(_fc) >> blueShift_;    

  result <<= redBits_;
  result |=  qRed(_bc) >> redShift_;
  result <<= greenBits_;
  result |=  qGreen(_bc) >> greenShift_;
  result <<= blueBits_;
  result |=  qBlue(_bc) >> blueShift_;
  
  return (result-1);
}


//-----------------------------------------------------------------------------


unsigned int
QtColorTranslator::maxIndex() const
{
  assert(initialized());
  unsigned int result(~0);
  result >>= 32 - redBits_ - greenBits_ - blueBits_;
  return (result-1);
}


//=============================================================================
} // namespace ACG
//=============================================================================


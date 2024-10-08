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
//  CLASS PBuffer
//
//=============================================================================
#ifdef ARCH_LINUX
//=============================================================================


#ifndef ACG_PBUFFER_HH
#define ACG_PBUFFER_HH


//== INCLUDES =================================================================

#include <GL/glxew.h>
#include <../GL/gl.hh>
#include <vector>
#include "../Config/ACGDefines.hh"


//== CLASS DEFINITION =========================================================



/** \class PBuffer PBuffer.hh <ACG/Utils/PBuffer.hh>
  Sets up a pbuffer and provides functions for working with it. Per
  default the pbuffer is double-buffered, 8-bit rgba values, 24-bit
  depth and preserved contents.
*/
class ACGDLLEXPORT PBuffer
{
public:

  /// Default constructor
  explicit PBuffer(int _bits);

  /** Constructor with arguments for pbuffer dimension.
      \param _w specifies the pbuffer's width in pixel.
      \param _h specifies the pbuffer's height in pixel.
  */
  PBuffer(int _w, int _h, int _bits);

  /// Destructor
  ~PBuffer();

  /** Change size of pbuffer by destroying it and recreating it with
      the desired width and height.
      \param _w specifies the pbuffer's width in pixel.
      \param _h specifies the pbuffer's height in pixel.
  */
  void resize(int _w, int _h);

  /** Activate the pbuffer. This means the pbuffer is the current rendering
      context and all subsequent OpenGL-commands are executed in this context.
  */
  void activate();

  /** Switch back to the original rendering context. No further OpenGL-commands
      are directed to the pbuffer.
  */
  void deactivate();

  /// Get current width of pbuffer.
  const int width() const;

  /// Get current height of pbuffer.
  const int height() const;

  int bits();


private:

  Display           *dpy_;
  GLXFBConfig       *fbc_;
  GLXPbuffer        pbuf_;
  GLXDrawable       currdraw_;
  GLXContext        currctx_, pbufctx_;
  std::vector<int>  sbAttrib_, pbAttrib_;
};


//=============================================================================
#endif // ACG_PBUFFER_HH defined
//=============================================================================
#endif // Linux only
//=============================================================================


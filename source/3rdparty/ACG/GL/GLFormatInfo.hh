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
//  OpenGL texture format info
//
//=============================================================================


#ifndef GL_FORMATINFO_HH
#define GL_FORMATINFO_HH


//== INCLUDES =================================================================

// GL
#include <ACG/GL/acg_glew.hh>
#include <ACG/GL/gl.hh>

// C++
#include <map>


//== NAMESPACES ===============================================================

namespace ACG {

//== CLASS DEFINITION =========================================================

class ACGDLLEXPORT GLFormatInfo
{
public:
  explicit GLFormatInfo(GLenum _internalFormat);

  GLFormatInfo();
  ~GLFormatInfo();

  // size in bytes of the i'th channel
  int channelSize(int i = 0) const {assert(i >= 0 && i < channelCount_); return channelBits_[i] >> 3;}

  // size in bits of the i'th channel
  int channelBits(int i) const {assert(i >= 0 && i < channelCount_); return channelBits_[i];}

  // number of channels
  int channelCount() const {return channelCount_;}

  // size in bytes of one element = num channels * channel size
  int elemSize() const {return bpp_ >> 3;}

  // bits per pixel
  int bpp() const {return bpp_;}

  // get internal format
  GLenum internalFormat() const {return internalFormat_;}

  // get a fitting (external) format for the internalfmt, ie. GL_RED, GL_RGBA, GL_RGBA_INTEGER etc.
  GLenum format() const {return format_;}

  // get a fitting data-type for the internalfmt, ie GL_UNSIGNED_BYTE, GL_FLOAT etc.
  GLenum type() const {return type_;}

  // some formats such as GL_RGBA, GL_R8 etc. are normalized to [0,1] when sampled
  bool isNormalized() const {return normalized_;}

  // data can be one of 3 unsized base types: floating pt, signed integer or unsigned integer
  enum BaseType
  {
    FloatingPt,
    SignedInt,
    UnsignedInt
  };

  bool isFloat() const {return baseType_ == FloatingPt;}
  bool isUint() const {return baseType_ == UnsignedInt;}
  bool isInt() const {return baseType_ == SignedInt;}

  bool isValid() const {return bpp_ != 0;}

  BaseType baseType() const {return baseType_;}

  // return string of the sized internalFormat, ie. GL_RGBA8, GL_RG32F etc.
  const char* sizedFormatString() const {return sizedName_;}

private:
  GLFormatInfo( GLenum _intfmt, GLenum _fmt, GLenum _type, int _r, int _g, int _b, int _a, BaseType _bt, bool _nrm);
  static void registerFmt(GLenum _intfmt, GLenum _fmt, GLenum _type, int _r, int _g, int _b, int _a, BaseType _bt, bool _nrm);

  GLenum internalFormat_,
    format_,
    type_;

  int channelBits_[4];

  int channelCount_,
    bpp_;

  // 0 -> floating point,  1 -> unsigned integer,  2 -> signed integer
  BaseType baseType_;

  bool normalized_;

  char sizedName_[32];

  // map from internalfmt to info
  static std::map<GLenum, GLFormatInfo> formatMap_;
};


//=============================================================================
} // namespace ACG
//=============================================================================
#endif // GL_FORMATINFO_HH defined
//=============================================================================

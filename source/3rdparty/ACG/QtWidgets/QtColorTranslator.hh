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
//  CLASS QtColorTranslator
//
//=============================================================================


#ifndef ACG_QTCOLORTRANSLATOR_HH
#define ACG_QTCOLORTRANSLATOR_HH


//== INCLUDES =================================================================

#include <QOpenGLContext>
#include <QColor>
#include "../Config/ACGDefines.hh"

//== NAMESPACES ===============================================================

namespace ACG {


//== CLASS DEFINITION =========================================================


/** \class QtColorTranslator QtColorTranslator.hh <ACG/QtWidgets/QtColorTranslator.hh>

    This class translates index <-> RGB color.

    The QtColorTranslator is attached to a QGLContext and translates
    index to color and vice versa. If the RGB bits of one buffer are
    not sufficient, both front and back buffer can be used.
**/

class ACGDLLEXPORT QtColorTranslator
{
public:

  /// Default constructor.
  QtColorTranslator() : initialized_(false),
                        redBits_(0), greenBits_(0), blueBits_(0),
                        redMask_(0), greenMask_(0), blueMask_(0),
                        redShift_(0), greenShift_(0), blueShift_(0),
                        redRound_(0), greenRound_(0), blueRound_(0)
  {};

  /// construct with QGLcontext
  explicit QtColorTranslator(QOpenGLContext& _context, QSurface* _surface)
  { initialize(_context, _surface); }

  /// Destructor.
  ~QtColorTranslator(){};


  /// init (takes current QGLcontext)
  void initialize();

  /// init with given QGLcontext
  void initialize(QOpenGLContext& _context, QSurface* _surface)
  { _context.makeCurrent(_surface);  initialize(); }

  /// has it been initialized?
  bool initialized() const { return initialized_; }


  /// index -> color (one buffer)
  bool index2color( unsigned int _idx, QRgb& _col ) const;
  /// index -> color (two buffers)
  bool index2color( unsigned int _idx,
		    QRgb& _frontColor,
		    QRgb& _backColor ) const;


  /// color -> index (one buffer)
  int color2index( QRgb _c ) const;
  /// color -> index (two buffers)
  int color2index( QRgb _frontColor, QRgb _backColor ) const;


  /// returns max convertable index (using ONE buffer)
  unsigned int maxIndex() const;


private:

  bool     initialized_;
  GLint    redBits_, greenBits_, blueBits_;
  GLuint   redMask_, greenMask_, blueMask_,
           redShift_, greenShift_, blueShift_,
           redRound_, greenRound_, blueRound_;
};


//=============================================================================
} // namespace ACG
//=============================================================================
#endif // ACG_QTCOLORTRANSLATOR_HH defined
//=============================================================================


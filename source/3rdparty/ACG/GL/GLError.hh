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
// GLError
//
//=============================================================================


#ifndef ACGGLERROR_H
#define ACGGLERROR_H

//=============================================================================
//
// Includes
//
//=============================================================================

#include <ACG/GL/gl.hh>
#include <ACG/Config/ACGDefines.hh>
#include <iostream>
#include <string>

//=============================================================================
//
// Macros
//
//=============================================================================

#ifndef NDEBUG
#define checkGLError() \
    { int error; if ( (error = glGetError()) != GL_NO_ERROR ) std::cout << "GLError " << __FILE__ << ":" << __LINE__ << " - " << error << std::endl; }

#define checkGLError2(str) \
    { int error; if ( (error = glGetError()) != GL_NO_ERROR ) std::cout << "GLError " << __FILE__ << ":" << __LINE__ << " - " << error << " - " << str <<std::endl; }

#else

#define checkGLError()
#define checkGLError2(str)

#endif

//=============================================================================
namespace ACG {
//=============================================================================

/** \brief replaces the gluErrorToString function at least for the standard OpenGL Headers
 *
 */
std::string ACGDLLEXPORT glErrorToString( GLenum _error );


/** Nice wrapper that outputs all current OpenGL errors to std::cerr.
    If no error is present nothing is printed.
**/
inline void glCheckErrors()
{
  GLenum error;
  while ((error = glGetError()) != GL_NO_ERROR)
  {
    std::cerr << "GL error: " << glErrorToString(error) << std::endl;
  }
}

//=============================================================================
}  // namespace ACG
//=============================================================================
#endif //ACGGLERROR_H defined
//=============================================================================

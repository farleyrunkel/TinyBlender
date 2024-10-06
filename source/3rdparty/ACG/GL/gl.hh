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
//  overload some GL functions
//=============================================================================


#ifndef ACG_GLTEXT_HH
#define ACG_GLTEXT_HH


//== INCLUDES =================================================================

#include <ACG/Config/ACGDefines.hh>


#if defined(ARCH_DARWIN)

  #include <OpenGL/gl.h>

#elif defined(WIN32)

  #include <windows.h>
  // Dont do this anymore! Use dll version. No problems with plugins and dll
  // but a lot with static linking
  // #  define GLEW_STATIC 1
  #include <gl/glew.h>

#else

  #include <GL/gl.h>

#endif


// qt refers to outdated include guard of glext.h, set the old guard if the
// new one is defined:
#ifdef __gl_glext_h_
#  ifndef __glext_h_
#    define __glext_h_ 1
#  endif
#endif


#include "../Math/VectorT.hh"


//=============================================================================
namespace ACG {
//=============================================================================


//-------------------------------------------------------------------- glVertex

/// Wrapper: glVertex for Vec2i
inline void glVertex(const Vec2i& _v)  { glVertex2i(_v[0], _v[1]); }
/// Wrapper: glVertex for Vec2f
inline void glVertex(const Vec2f& _v)  { glVertex2fv(_v.data()); }
/// Wrapper: glVertex for Vec2d
inline void glVertex(const Vec2d& _v)  { glVertex2dv(_v.data()); }

/// Wrapper: glVertex for Vec3f
inline void glVertex(const Vec3f& _v)  { glVertex3fv(_v.data()); }
/// Wrapper: glVertex for Vec3d
inline void glVertex(const Vec3d& _v)  { glVertex3dv(_v.data()); }

/// Wrapper: glVertex for Vec4f
inline void glVertex(const Vec4f& _v)  { glVertex4fv(_v.data()); }
/// Wrapper: glVertex for Vec4d
inline void glVertex(const Vec4d& _v)  { glVertex4dv(_v.data()); }



//------------------------------------------------------------------- glTexCoord

/// Wrapper: glTexCoord for Vec2f
inline void glTexCoord(const Vec2f& _t) { glTexCoord2fv(_t.data()); }
/// Wrapper: glTexCoord for Vec2d
inline void glTexCoord(const Vec2d& _t) { glTexCoord2dv(_t.data()); }

/// Wrapper: glTexCoord for Vec3f
inline void glTexCoord(const Vec3f& _t) { glTexCoord3fv(_t.data()); }
/// Wrapper: glTexCoord for Vec3d
inline void glTexCoord(const Vec3d& _t) { glTexCoord3dv(_t.data()); }

/// Wrapper: glTexCoord for Vec4f
inline void glTexCoord(const Vec4f& _t) { glTexCoord4fv(_t.data()); }
/// Wrapper: glTexCoord for Vec4d
inline void glTexCoord(const Vec4d& _t) { glTexCoord4dv(_t.data()); }



//--------------------------------------------------------------------- glNormal

/// Wrapper: glNormal for Vec3f
inline void glNormal(const Vec3f& _n)  { glNormal3fv(_n.data()); }
/// Wrapper: glNormal for Vec3d
inline void glNormal(const Vec3d& _n)  { glNormal3dv(_n.data()); }



//---------------------------------------------------------------------- glColor

/// Wrapper: glColor for Vec3f
inline void glColor(const Vec3f&  _v)  { glColor3fv(_v.data()); }
/// Wrapper: glColor for Vec3uc
inline void glColor(const Vec3uc& _v)  { glColor3ubv(_v.data()); }

/// Wrapper: glColor for Vec4f
inline void glColor(const Vec4f&  _v)  { glColor4fv(_v.data()); }
/// Wrapper: glColor for Vec4uc
inline void glColor(const Vec4uc&  _v) { glColor4ubv(_v.data()); }



//-------------------------------------------------------------- ACG::GLState::vertexPointer
/*
/// Wrapper: ACG::GLState::vertexPointer for Vec2f
inline void glVertexPointer(const Vec2f* _p)
{ ::ACG::GLState::vertexPointer(2, GL_FLOAT, 0, _p); }
/// Wrapper: ACG::GLState::vertexPointer for Vec2d
inline void ACG::GLState::vertexPointer(const Vec2d* _p)
{ ::ACG::GLState::vertexPointer(2, GL_DOUBLE, 0, _p); }

/// Wrapper: ACG::GLState::vertexPointer for Vec3f
inline void ACG::GLState::vertexPointer(const Vec3f* _p)
{ ::ACG::GLState::vertexPointer(3, GL_FLOAT, 0, _p); }
/// Wrapper: ACG::GLState::vertexPointer for Vec3d
inline void ACG::GLState::vertexPointer(const Vec3d* _p)
{ ::ACG::GLState::vertexPointer(3, GL_DOUBLE, 0, _p); }

/// Wrapper: ACG::GLState::vertexPointer for Vec4f
inline void ACG::GLState::vertexPointer(const Vec4f* _p)
{ ::ACG::GLState::vertexPointer(4, GL_FLOAT, 0, _p); }
/// Wrapper: ACG::GLState::vertexPointer for Vec4d
inline void ACG::GLState::vertexPointer(const Vec4d* _p)
{ ::ACG::GLState::vertexPointer(4, GL_DOUBLE, 0, _p); }

/// original method
inline void ACG::GLState::vertexPointer(GLint n, GLenum t, GLsizei s, const GLvoid *p)
{ ::ACG::GLState::vertexPointer(n, t, s, p); }



//-------------------------------------------------------------- ACG::GLState::normalPointer

/// Wrapper: ACG::GLState::normalPointer for Vec3f
inline void ACG::GLState::normalPointer(const Vec3f* _p)
{ ::ACG::GLState::normalPointer(GL_FLOAT, 0, _p); }
/// Wrapper: ACG::GLState::normalPointer for Vec3d
inline void ACG::GLState::normalPointer(const Vec3d* _p)
{ ::ACG::GLState::normalPointer(GL_DOUBLE, 0, _p); }

/// original method
inline void ACG::GLState::normalPointer(GLenum t, GLsizei s, const GLvoid *p)
{ ::ACG::GLState::normalPointer(t, s, p); }



//--------------------------------------------------------------- ACG::GLState::colorPointer

/// Wrapper: ACG::GLState::colorPointer for Vec3uc
inline void ACG::GLState::colorPointer(const Vec3uc* _p)
{ ::ACG::GLState::colorPointer(3, GL_UNSIGNED_BYTE, 0, _p); }
/// Wrapper: ACG::GLState::colorPointer for Vec3f
inline void ACG::GLState::colorPointer(const Vec3f* _p)
{ ::ACG::GLState::colorPointer(3, GL_FLOAT, 0, _p); }

/// Wrapper: ACG::GLState::colorPointer for Vec4uc
inline void ACG::GLState::colorPointer(const Vec4uc* _p)
{ ::ACG::GLState::colorPointer(4, GL_UNSIGNED_BYTE, 0, _p); }
/// Wrapper: ACG::GLState::colorPointer for Vec4f
inline void ACG::GLState::colorPointer(const Vec4f* _p)
{ ::ACG::GLState::colorPointer(4, GL_FLOAT, 0, _p); }

/// original method
inline void ACG::GLState::colorPointer(GLint n, GLenum t, GLsizei s, const GLvoid *p)
{ ::ACG::GLState::colorPointer(n, t, s, p); }



//------------------------------------------------------------ ACG::GLState::texcoordPointer

/// Wrapper: ACG::GLState::texcoordPointer for float
inline void ACG::GLState::texcoordPointer(const float* _p)
{ ::ACG::GLState::texcoordPointer(1, GL_FLOAT, 0, _p); }
/// Wrapper: ACG::GLState::texcoordPointer for Vec2d
inline void ACG::GLState::texcoordPointer(const double* _p)
{ ::ACG::GLState::texcoordPointer(1, GL_DOUBLE, 0, _p); }

/// Wrapper: ACG::GLState::texcoordPointer for Vec2f
inline void ACG::GLState::texcoordPointer(const Vec2f* _p)
{ ::ACG::GLState::texcoordPointer(2, GL_FLOAT, 0, _p); }
/// Wrapper: ACG::GLState::texcoordPointer for Vec2d
inline void ACG::GLState::texcoordPointer(const Vec2d* _p)
{ ::ACG::GLState::texcoordPointer(2, GL_DOUBLE, 0, _p); }

/// Wrapper: ACG::GLState::texcoordPointer for Vec3f
inline void ACG::GLState::texcoordPointer(const Vec3f* _p)
{ ::ACG::GLState::texcoordPointer(3, GL_FLOAT, 0, _p); }
/// Wrapper: ACG::GLState::texcoordPointer for Vec3d
inline void ACG::GLState::texcoordPointer(const Vec3d* _p)
{ ::ACG::GLState::texcoordPointer(3, GL_DOUBLE, 0, _p); }

/// Wrapper: ACG::GLState::texcoordPointer for Vec4f
inline void ACG::GLState::texcoordPointer(const Vec4f* _p)
{ ::ACG::GLState::texcoordPointer(4, GL_FLOAT, 0, _p); }
/// Wrapper: ACG::GLState::texcoordPointer for Vec4d
inline void ACG::GLState::texcoordPointer(const Vec4d* _p)
{ ::ACG::GLState::texcoordPointer(4, GL_DOUBLE, 0, _p); }

/// original method
inline void ACG::GLState::texcoordPointer(GLint n, GLenum t, GLsizei s, const GLvoid *p)
{ ::ACG::GLState::texcoordPointer(n, t, s, p); }
*/

/**
 * @brief getExtensionString returns a string containing all supported OpenGL extensions
 * this function uses the new style to query the extensions
 *
 * @return std::string with all supported extensions
 */
std::string ACGDLLEXPORT getExtensionString(  ) ;

/** Check if the extension given by a std::string is supported by the current OpenGL extension
*/
bool ACGDLLEXPORT checkExtensionSupported( const std::string& _extension ) ;

/** Check if OpenGL Version is greater or equal than the given values
*/
bool ACGDLLEXPORT openGLVersion( const int _major, const int _minor, bool verbose = true );

/** Test if OpenGL Version is greater or equal than the given values
 *  No output is generated, if the OpenGL version is less than the tested value
*/
inline bool ACGDLLEXPORT openGLVersionTest( const int _major, const int _minor  ){return openGLVersion(_major, _minor, false);}

/// Store opengl core profile setting
void ACGDLLEXPORT compatibilityProfile(bool _compatProfile);

/// get opengl core profile setting
bool ACGDLLEXPORT compatibilityProfile( );

//=============================================================================
}  // namespace ACG
//=============================================================================
#endif // ACG_GLTEXT_HH defined
//=============================================================================

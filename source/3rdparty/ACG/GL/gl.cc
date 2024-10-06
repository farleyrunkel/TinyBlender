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

#include <ACG/GL/acg_glew.hh>
#include "gl.hh"

#include <cstdlib>
#include <sstream>
#include <iostream>



//=============================================================================
namespace ACG {
//=============================================================================
namespace {
/**
 * Helper function for checkExtensionSupported(). Only for internal use.
 *
 * @return The string returned by glGetString(GL_EXTENSIONS) or an empty string
 * if glGetString() returned a null pointer.
 */
inline const char *_getExtensionString() {

  // glGetString(GL_EXTENSIONS) is deprecated and not available in core profile
    if(!openGLVersion(3,0))
    {
        return (const char*) glGetString(GL_EXTENSIONS);
    }

  // enumerate extensions with glGetStringi(GL_EXTENSIONS, id) instead
#ifndef __APPLE__
  if (!glGetStringi)
    glewInit();
#endif
  static std::string supported_str;

  if (supported_str.empty()) {
    GLint extensions = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &extensions);

    for (int i = 0; i < extensions; ++i) {

      const char *supported_cstr = reinterpret_cast<const char*>(glGetStringi(GL_EXTENSIONS, i));
    
      if (supported_cstr == 0) {
        std::cerr << "\x1b[1;31mACG::checkExtensionsSupported: "
          "glGetStringi(GL_EXTENSIONS, "<<i<<") call failed.\x1b[0m\n";
        return "";
      }

      supported_str += supported_cstr;
      if (i + 1 < extensions)
        supported_str += " ";
    }
  }

  return supported_str.c_str();
}
}

std::string getExtensionString()
{
    return _getExtensionString();
}

/** Check if the extension given by a std::string is supported by the current OpenGL extension
*/
bool checkExtensionSupported( const std::string& _extension )  {
    /**
     * Cache glGetString(...) output because depending on the graphics driver
     * it might return 0 the second time we call it.
     */
    static const std::string supported_str(_getExtensionString());

    /*
     * supported_str is a space delimited list. Tokenize it. Simply searching
     * for _extension within supported_str yields false positives if the
     * requested extension is a substring of a supported one.
     */
    for (std::istringstream supported(supported_str); !supported.eof(); ) {
        std::string feature;
        supported >> feature;
        if (feature == _extension) return true;
    }
    return false;
}

/** Check if OpenGL Version is greater or equal than the given values
*/
bool openGLVersion( const int _major, const int _minor, bool _verbose  )  {

  // Read OpenGL Version string
  std::string glVersionString = (const char*)glGetString(GL_VERSION);

  // Use string stream to parse
  std::istringstream stream;
  stream.str(glVersionString);

  // Buffer for the dot between major and minor
  char dot;

  // Read Major version number
  int major ;
  stream >> major;
  stream >> dot;

  // Read minor version number
  int minor;
  stream >> minor;

  if ( (_major > major) || ( (_major == major) && (_minor > minor))  ) {
    if(_verbose)
    {
      std::cerr << "OpenGL Version check failed. Required  : " << _major << "." << _minor << std::endl;
      std::cerr << "OpenGL Version check failed. Available : " << major << "." << minor << std::endl;
    }
    return false;
  }

  return true;
}

/// Use opengl compatibility profile mode?
static bool compatibilityProfile_ = false;

/// Store opengl core profile setting
void compatibilityProfile( bool _enableCoreProfile ) {
  compatibilityProfile_ = _enableCoreProfile;
}

/// get opengl core profile setting
bool compatibilityProfile( ) {
  return compatibilityProfile_;
}

//=============================================================================
}  // namespace ACG
//=============================================================================

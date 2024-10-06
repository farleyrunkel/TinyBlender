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
//  CLASS ScreenQuad
//
//=============================================================================

#ifndef ACG_SCREENQUAD_HH
#define ACG_SCREENQUAD_HH


//== INCLUDES =================================================================

#include <vector>

#include <ACG/GL/gl.hh>
#include <ACG/GL/VertexDeclaration.hh>


//== NAMESPACES ===============================================================


namespace ACG {


//== CLASS DEFINITION =========================================================


/** This class provides a simple method to draw a screen-aligned quad.
*/
class ACGDLLEXPORT ScreenQuad
{
public:
   /// Destructor.
  ~ScreenQuad();


  /** \brief Draw the screen quad
  *
  * The quad is in projected space with coordinates in [-1, 1].
  * @param _prog GLSL shader to bind attribute id's if needed. Pass null-pointer for fixed function rendering
  */ 
  static void draw(GLSL::Program* _prog = 0);

  /** \brief Draw the screen quad with instancing
  *
  * Can be used for render to 2D-array / 3D volume
  * The quad is in projected space with coordinates in [-1, 1].
  * @param _count number of instances
  * @param _prog GLSL shader to bind attribute id's if needed. Pass null-pointer for fixed function rendering
  */ 
  static void drawInstanced(int _count, GLSL::Program* _prog = 0);


  /** \brief Draw a 2D texture to screen
  *
  * Useful for debugging, copying a texture..
  *
  * @param _texture Texture that should be bound when drawing the quad
  * @param _offset  Offset passed to the uniform called offset in the shader
  * @param _size    Size passed to the uniform called size in the shader
  */ 
  static void drawTexture2D(GLuint _texture, const Vec2f& _offset = Vec2f(0.0f, 0.0f),
                                             const Vec2f& _size   = Vec2f(1.0f, 1.0f));

private:

  /// Default constructor.
  ScreenQuad();


  /// Get singleton instance
  static ScreenQuad& instance();

  /// Initialize vbo and format
  void init();

  /// Internal draw function
  void intDraw(GLSL::Program* _prog, int _numInstances = 0);



  /// vbo containing the quad in projected coordinates
  GLuint vbo_;

  /// vertex format of screen quad (float2 pos)
  VertexDeclaration* decl_;


  /// Simple texture drawing shader
  GLSL::Program* texDrawProg_;
};


//=============================================================================
} // namespace ACG
//=============================================================================
#endif // ACG_SCREENQUAD_HH defined
//=============================================================================


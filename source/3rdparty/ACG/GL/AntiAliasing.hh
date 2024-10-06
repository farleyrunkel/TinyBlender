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
//  Helper classes for Anti-Aliasing of render targets
//
//=============================================================================


#ifndef ANTI_ALIASING_HH
#define ANTI_ALIASING_HH


//== INCLUDES =================================================================

// GL
#include <ACG/GL/acg_glew.hh>
#include <ACG/GL/globjects.hh>
#include <ACG/GL/gl.hh>

// C++
#include <vector>


//== FORWARD DECLARATIONS =====================================================

namespace GLSL{
  class Program;
}


//== NAMESPACES ===============================================================

namespace ACG {


//== CLASS DEFINITION =========================================================


/*
OpenGL MSAA:

A post-processing fragment shader has to compute colors per sample rather than per pixel for full MSAA.
Sample values are then weighted by filter weights to obtain a final pixel color.

Filter weights can be computed with several methods, for example:
 - distance based filter weights (computed with MSFilterWeights)
 - constant filter weights: 1/numSamples

However, a faster approach can be used where the MSAA texture is filtered onto a regular texture first, 
followed by executing post processing shaders on that regular texture.
While this is not a correct MSAA implementation, it can give something close at better performance.
-> MSTextureSampler can be used to filter a MSAA texture onto a regular texture

Recommended resources on implementing MSAA with render to texture:
 http://diaryofagraphicsprogrammer.blogspot.com/2009/06/multisample-anti-aliasing.html
 http://www.humus.name/index.php?page=3D&ID=81
*/

#ifdef GL_ARB_texture_multisample
#define MSFILTERWEIGHTS

class ACGDLLEXPORT MSFilterWeights
{
public:

  MSFilterWeights(int _numSamples);

  virtual ~MSFilterWeights() { }


  // Initializes a texture buffer with filter weights in format GL_R32F.
  // This can be used for reading from multisampled textures in a shader.
  void asTextureBuffer(TextureBuffer& out);


  // get ptr to weights as array, can be used for setUniform() for example
  const float* asDataPtr() const {return &weights_[0];}


  float operator [] (int i) const {return weights_[i];}

  float getWeight(int i) const {return weights_[i];}

  int getNumSamples() const {return numSamples_;}

private:

  int numSamples_;
  std::vector<float> weights_;
};

#endif // GL_ARB_texture_multisample




//== CLASS DEFINITION =========================================================


// This class performs multisampling on MSAA textures and writes the result to the currently bound FBO.
#ifdef GL_ARB_texture_multisample
#define MSTEXTURESAMPLER

class ACGDLLEXPORT MSTextureSampler
{
public:

  ~MSTextureSampler();


  // nearest point filtering of a MSAA texture, recommended when the target FBO has the same size as the input texture
  static void filterMSAATexture_Nearest(GLuint _texture, int _samples, const float* _weights = 0);

  // nearest point filtering of a MSAA texture, recommended when the target FBO has the same size as the input texture
  // also resolves a multisampled depth texture into the depth target
  static void filterMSAATexture_Nearest(GLuint _texture, GLuint _depthTexture, int _samples, const float* _weights = 0);

  // bilinear filtering of a MSAA texture
  static void filterMSAATexture_Linear(GLuint _texture, int _samples, const float* _weights = 0);

private:

  MSTextureSampler();


  void init();

  // singleton instance
  static MSTextureSampler& instance();


  TextureBuffer filterWeights_;

  // sampling shaders for nearest and bilinear texture filtering
  GLSL::Program* shaderNearest_;
  GLSL::Program* shaderNearestDepth_;
  GLSL::Program* shaderLinear_;
};

#endif // GL_ARB_texture_multisample



// maybe add:
// - blitting of multisampled render buffers
// - automatic generation of MSAA variants of a post processing fragment shader
// - downsampling of supersampled FBOs

//=============================================================================
} // namespace ACG
//=============================================================================
#endif // ANTI_ALIASING_HH defined
//=============================================================================

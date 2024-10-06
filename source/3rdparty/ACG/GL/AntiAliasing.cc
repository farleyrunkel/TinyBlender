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
//  OpenGL AntiAliasing methods
//
//=============================================================================


//== INCLUDES =================================================================

#include <ACG/GL/AntiAliasing.hh>
#include <ACG/GL/acg_glew.hh>
#include <ACG/ShaderUtils/GLSLShader.hh>
#include <ACG/GL/ScreenQuad.hh>

#include <ACG/GL/ShaderCache.hh>

//== DEFINES ==================================================================

// shader files
#define MSAA_SCREENQUAD_SHADER "ScreenQuad/screenquad.glsl"
#define MSAA_NEAREST_SHADER "MSAA/sample_nearest.glsl"
#define MSAA_NEAREST_DEPTH_SHADER "MSAA/sample_nearest_and_depth.glsl"
#define MSAA_LINEAR_SHADER "MSAA/sample_linear.glsl"



//== NAMESPACES ===============================================================

namespace ACG {


//== CLASS IMPLEMENTATION =====================================================

#ifdef MSFILTERWEIGHTS


MSFilterWeights::MSFilterWeights(int _numSamples) : numSamples_(_numSamples) {

  weights_.resize(_numSamples);
  float sumWeights = 0.0f;

  // texel center is at (0.5, 0.5) as specified in 
  //  http://www.opengl.org/sdk/docs/man3/xhtml/glGetMultisample.xml
  Vec2f texelCenter(0.5f, 0.5f);

  // query sample count of currently bound fbo to avoid error on calling glGetMultisample
  int fboSamples = 0;
  glGetIntegerv(GL_SAMPLES, &fboSamples);


  for (int i = 0; i < _numSamples; ++i) {
    GLfloat val[2];

    if (i < fboSamples)
      glGetMultisamplefv(GL_SAMPLE_POSITION, i, val);
    else
      val[0] = val[1] = 0.5f; // maybe output warning here
    
    Vec2f samplePosition(val[0], val[1]);

    // weighting based on distance to texel center
    float sampleDist = (samplePosition - texelCenter).norm();

    // samples close to the center are weighted higher than samples farther away
    weights_[i] = 1.0f - sampleDist;

    sumWeights += weights_[i];
  }

  // normalize weights

  for (int i = 0; i < _numSamples; ++i)
    weights_[i] /= sumWeights;
}

//=============================================================================
void MSFilterWeights::asTextureBuffer( TextureBuffer& out ) {
  if (numSamples_)
    out.setBufferData(numSamples_ * 4, &weights_[0], GL_R32F, GL_STATIC_DRAW);
}
//=============================================================================

#endif //MSFILTERWEIGHTS


//=============================================================================

#ifdef MSTEXTURESAMPLER
MSTextureSampler::MSTextureSampler() : shaderNearest_(0), shaderNearestDepth_(0), shaderLinear_(0) {
}


MSTextureSampler::~MSTextureSampler() {
  delete shaderNearest_;
  delete shaderLinear_;
  delete shaderNearestDepth_;
}

MSTextureSampler& MSTextureSampler::instance() {
  static MSTextureSampler singleton;
  return singleton;
}

//=============================================================================

void MSTextureSampler::init() {
  if (!shaderNearest_)
    shaderNearest_ = GLSL::loadProgram(MSAA_SCREENQUAD_SHADER, MSAA_NEAREST_SHADER);

  if (!shaderNearestDepth_)
    shaderNearestDepth_ = GLSL::loadProgram(MSAA_SCREENQUAD_SHADER, MSAA_NEAREST_DEPTH_SHADER);

  if (!shaderLinear_)
    shaderLinear_ = GLSL::loadProgram(MSAA_SCREENQUAD_SHADER, MSAA_LINEAR_SHADER);
}

//=============================================================================

void MSTextureSampler::filterMSAATexture_Nearest( GLuint _texture, int _samples, const float* _weights /*= 0*/ ) {

  MSTextureSampler& sampler = instance();

  // load shader
  if (!sampler.shaderNearest_)
    sampler.init();

  GLSL::Program* shader = sampler.shaderNearest_;

  if (!shader)
    return;
  

  shader->use();

  // offset and scale of screenquad
  shader->setUniform("offset", Vec2f(0.0f, 0.0f));
  shader->setUniform("size", Vec2f(1.0f, 1.0f));
  
  // sample count and filter weights
  shader->setUniform("numSamples", _samples);

  // bind multisampled texture to slot 0
  shader->setUniform("inputTex", 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, _texture);

  // run texture filter
  ScreenQuad::draw(shader);

  shader->disable();
}

//=============================================================================

void MSTextureSampler::filterMSAATexture_Nearest( GLuint _texture, GLuint _depthTexture, int _samples, const float* _weights /*= 0*/ ) {

  MSTextureSampler& sampler = instance();

  // load shader
  if (!sampler.shaderNearestDepth_)
    sampler.init();

//  GLSL::Program* shader = sampler.shaderNearestDepth_;
  GLSL::Program* shader = ACG::ShaderCache::getInstance()->getProgram(MSAA_SCREENQUAD_SHADER, MSAA_NEAREST_DEPTH_SHADER);

  if (!shader)
    return;


  shader->use();

  // offset and scale of screenquad
  shader->setUniform("offset", Vec2f(0.0f, 0.0f));
  shader->setUniform("size", Vec2f(1.0f, 1.0f));

  // sample count and filter weights
  shader->setUniform("numSamples", _samples);

  // bind multisampled depth texture to slot 1
  shader->setUniform("inputDepthTex", 1);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, _depthTexture);

  // bind multisampled texture to slot 0
  shader->setUniform("inputTex", 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, _texture);

  // run texture filter
  ScreenQuad::draw(shader);

  shader->disable();
}

//=============================================================================

void MSTextureSampler::filterMSAATexture_Linear( GLuint _texture, int _samples, const float* _weights /*= 0*/ ) {

  MSTextureSampler& sampler = instance();

  // load shader
  if (!sampler.shaderLinear_)
    sampler.init();

  GLSL::Program* shader = sampler.shaderLinear_;

  if (!shader)
    return;


  shader->use();

  // offset and scale of screenquad
  shader->setUniform("offset", Vec2f(0.0f, 0.0f));
  shader->setUniform("size", Vec2f(1.0f, 1.0f));

  // sample count and filter weights
  shader->setUniform("numSamples", _samples);

  // bind multisampled texture to slot 0
  shader->setUniform("inputTex", 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, _texture);

  // run texture filter
  ScreenQuad::draw(shader);

  shader->disable();
}


//=============================================================================



#endif // MSTEXTURESAMPLER


} // namespace ACG
//=============================================================================

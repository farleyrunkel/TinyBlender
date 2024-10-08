/*===========================================================================*\
*                                                                            *
*                              OpenFlipper                                   *
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
*                                                                            *
\*===========================================================================*/



#pragma once


#include <ACG/Config/ACGDefines.hh>
#include <ACG/Math/GLMatrixT.hh>
#include <ACG/GL/gl.hh>

#include <QStringList>
#include <QImage>
#include <vector>



// Forward Declaration
namespace GLSL
{
  class Program;
}

namespace ACG
{

// Forward Declaration
class FBO;



class ACGDLLEXPORT BaseSeparableFilterKernel
{
public:

  /* \brief Create separable filter
   *
   * @param _texWidth width of input texture
   * @param _texHeight height of input texture
   * @param _internalfmt internal gl format of the texture
  */
  BaseSeparableFilterKernel(int _texWidth, int _texHeight, GLenum _internalfmt = GL_RGBA);

  /// Class destructor
  virtual ~BaseSeparableFilterKernel();


  /* \brief Execute filter
   *
   * Perform the two filter passes.
   * If a custom fbo and a _tempColorAttachment is provided, the first pass is 
   * rendered into that temporary render texture.
   * If a custom fbo and a _dstColorAttachment is provided, the second pass is 
   * rendered into that destination render texture.
   * 
   * If _dstFBO or _tempColorAttachment is invalid, an internal fbo is created as target for the first pass.
   * If _dstFBO or _dstColorAttachment is invalid, the second pass renders into whatever is currently bound as render target.
   *
   * @param _srcTexture 2d input texture
   * @param _dstFBO custom target fbo (optional)
   * @param _dstColorAttachment target attachment of the custom target fbo (optional)
   * @param _tempColorAttachment temporary attachment of the custom fbo, that is different from _dstColorAttachment (optional)
   * @return true on success, false otherwise
  */
  bool execute(GLuint _srcTexture, ACG::FBO* _dstFBO = 0, GLuint _dstColorAttachment = GL_COLOR_ATTACHMENT0, GLuint _tempColorAttachment = 0);


  /* \brief Resize input texture
   *
   * @param _texWidth new input texture width
   * @param _texHeight new input texture height
  */
  void resizeInput(int _texWidth, int _texHeight);

  /// input texture width
  int texWidth() const {return texWidth_;}

  /// input texture height
  int texHeight() const {return texHeight_;}

  /// texel size in uv space
  const ACG::Vec2f& texelSize() const {return texelSize_;}

  /// internal format of the input texture
  GLenum internalFormat() const {return internalfmt_;}

protected:

  /* \brief Setup shader with uniforms
   *
   * @param _pass pass number: 0 or 1
   * @param _scrTex source texture for pass
   * @return shader program for screen quad
  */
  virtual GLSL::Program* setupPass(int _pass, GLuint _srcTex) = 0;


  /* \brief Update kernel after resizing
   *
  */
  virtual void updateKernel() = 0;

private:

  int texWidth_,
    texHeight_;

  GLenum internalfmt_,
    externalfmt_;

  // size of a texel in uv space
  ACG::Vec2f texelSize_;

  // temp render targets if none supplied by user
  //  attachment0 : target for first axis pass
  ACG::FBO* tempRT_;
};


// separable 2d gaussian blur
class ACGDLLEXPORT GaussianBlurFilter : public BaseSeparableFilterKernel
{
public:

  /* \brief Create separable gauss filter
   *
   * @param _texWidth width of input texture
   * @param _texHeight height of input texture
   * @param _blurRadius radius in pixel coords of the blur kernel
   * @param _blurSigma blur smoothness, standard deviation of the gaussian function
   * @param _internalfmt internal gl format of the texture
  */
  GaussianBlurFilter(int _texWidth, int _texHeight, int _blurRadius, float _blurSigma = 1.0f, GLenum _internalfmt = GL_RGBA);

  /// Class destructor
  virtual ~GaussianBlurFilter();

  /* \brief Change kernel settings
   *
   * @param _blurRadius new radius
   * @param _blurSigma new sigma
  */
  void setKernel(int _blurRadius, float _blurSigma);

  /// radius
  int radius() const {return radius_;}

  /// number of samples
  int samples() const {return samples_;}

  /// blur sigma
  int sigma() const {return sigma_;}

  /// sample offsets along x direction
  const std::vector<ACG::Vec2f>& offsetsX() const {return offsetsX_;}

  /// sample offsets along y direction
  const std::vector<ACG::Vec2f>& offsetsY() const {return offsetsY_;}

  /// sample weights
  const std::vector<float>& weights() const {return weights_;}

protected:
  
  virtual GLSL::Program* setupPass(int _pass, GLuint _srcTex) override;

  void updateKernel() override;

private:

  int radius_, samples_;

  /// blur std
  float sigma_;

  /// shader macros
  QStringList macros_;

  /// filter taps
  std::vector<ACG::Vec2f> offsetsX_;
  std::vector<ACG::Vec2f> offsetsY_;

  /// kernel weights
  std::vector<float> weights_;
};


// bilateral blur: gaussian blur with silhouette preservation
class ACGDLLEXPORT BilateralBlurFilter : public BaseSeparableFilterKernel
{
public:

  /* \brief Create bilateral filter
   *
   * @param _texWidth width of input texture
   * @param _texHeight height of input texture
   * @param _blurRadius radius in pixel coords of the blur kernel
   * @param _blurSigmaS blur smoothness in spatial distance (here distance in pixel coords)
   * @param _blurSigmaR blur smoothness in range distance (here linear view space difference)
   * @param _internalfmt internal gl format of the texture
  */
  BilateralBlurFilter(int _texWidth, int _texHeight, int _blurRadius, float _blurSigmaS = 1.0f, float blurSigmaR = 1.0f, GLenum _internalfmt = GL_RGBA);

  /// Class destructor
  virtual ~BilateralBlurFilter();


  /* \brief Set dynamic params before calling execute()
   *
   * @param _proj projection matrix
   * @param _depthTex depthbuffer texture (nonlinear depths)
  */
  void setParams(const ACG::GLMatrixf& _proj, GLuint _depthTex)
  {
    proj_ = _proj;
    depthTex_ = _depthTex;
  }


  /* \brief Change kernel settings
   *
   * @param _blurRadius new radius
   * @param _blurSigmaS new spatial smoothness
   * @param _blurSigmaR new range smoothness
  */
  void setKernel(int _blurRadius, float _blurSigmaS, float _blurSigmaR);


  /// radius
  int radius() const {return radius_;}

  /// number of samples
  int samples() const {return samples_;}

  /// blur (sigmaS, sigmaR)
  const ACG::Vec2f& sigma() const {return sigma_;}

  /// sample offsets along x direction
  const std::vector<ACG::Vec2f>& offsetsX() const {return offsetsX_;}

  /// sample offsets along y direction
  const std::vector<ACG::Vec2f>& offsetsY() const {return offsetsY_;}

protected:
  virtual GLSL::Program* setupPass(int _pass, GLuint _srcTex) override;

  void updateKernel() override;

private:

  int radius_,
    samples_;

  /// (sigmaS, sigmaR)
  ACG::Vec2f sigma_;

  /// -1 / (2 * sigma^2)
  ACG::Vec2f sigma2Rcp_;

  /// filter taps
  std::vector<ACG::Vec2f> offsetsX_;
  std::vector<ACG::Vec2f> offsetsY_;

  /// precomputed sample -r^2 / (2 * sigma_s^2)
  std::vector<float> spatialKernel_;

  /// shader macros
  QStringList macros_;

  ACG::GLMatrixf proj_;
  GLuint depthTex_;
};


class ACGDLLEXPORT RadialBlurFilter
{
public:

  /* \brief Create radial blur filter
   *
   * @param _numSamples number of kernel samples
  */
  explicit RadialBlurFilter(int _numSamples);

  /// Class destructor
  virtual ~RadialBlurFilter() {}

  /* \brief Perform radial blur
   *
   * Writes to currently bound render target.
   * A good intensity range for light to very strong blur is [0.0025, 0.01].
   *
   * @param _srcTexture input 2d texture
   * @param _blurRadius blur radius in uv space
   * @param _blurIntensity intensity, quadratic distance factor
   * @param _blurCenter center in uv space
  */
  bool execute(GLuint _srcTexture, float _blurRadius = 1.0f, float _blurIntensity = 0.0025f, const ACG::Vec2f& _blurCenter = ACG::Vec2f(0.5f, 0.5f));


  /* \brief Change kernel settings
   *
   * @param _numSamples new sample count
  */
  void setKernel(int _numSamples);

  /// number of samples
  int samples() const {return samples_;}

private:

  int samples_;

  QStringList macros_;
};


class ACGDLLEXPORT PoissonBlurFilter
{
public:

  /* \brief Create poisson blur filter
   *
   * @param _radius radius of poisson disk
   * @param _sampleDistance min distance between two samples
   * @param _numTries number of tries per sample to find the next sample
   * @param _disk should samples lie inside a disk of radius r or inside square [-r/2, -r/2] x [r/2, r/2]
   * @param _tilingCheck check min distance also across borders in a repeat-tiled kernel grid
  */
  PoissonBlurFilter(float _radius, float _sampleDistance, int _numTries = 30, bool _disk = true, bool _tilingCheck = false);

  /// Class destructor
  virtual ~PoissonBlurFilter();

  /* \brief Perform poisson blur
   *
   * Writes to currently bound render target.
   * 
   * @param _srcTex input 2d texture
   * @param _kernelScale kernel radius scaling factor
  */
  bool execute(GLuint _srcTex, float _kernelScale = 1.0f);


  /// radius
  float radius() const {return radius_;}

  /// number of samples
  int numSamples() const {return int(samples_.size());}

  /// min distance between two samples
  float sampleDistance() const {return sampleDistance_;}

  /// number of iterations per sample
  int numTries() const {return numTries_;}

  /// samples inside disk or square area
  bool disk() const { return disk_; }

  /// disk sample offsets
  const std::vector<ACG::Vec2f>& samples() const {return samples_;}

  /// dump samples as point cloud in obj format
  void dumpSamples(const char* _filename);

  /// plot samples on qt image
  void plotSamples(QImage* _image);

private:

  // sampling settings
  float radius_;
  float sampleDistance_;
  int numTries_;
  bool disk_;

  // poisson disk
  std::vector<ACG::Vec2f> samples_;

  // scaled samples
  std::vector<ACG::Vec2f> samplesScaled_;

  // shader macros
  QStringList macros_;
};



}

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
#include <ACG/GL/globjects.hh>
#include <ACG/GL/GLFormatInfo.hh>
#include <ACG/ShaderUtils/GLSLShader.hh>
#include <ACG/Utils/ImageConversion.hh>

#include <QImage>
#if QT_VERSION_MAJOR < 6
    #include <QGLWidget>
#else
    #include <QtOpenGLWidgets/QOpenGLWidget>
#endif

namespace ACG {


//-----------------------------------------------------------------------------

Texture::Texture( GLenum tgt, GLenum _unit )
  : target(tgt), unit(_unit), valid(false), texture(0u), internalFormat_(0)
{
}

void Texture::bindAsImage(GLuint _index, GLenum _access)
{
#if defined(GL_ARB_shader_image_load_store)
  if (is_valid())
    glBindImageTexture(_index, id(), 0, GL_FALSE, 0, _access, getInternalFormat());
  else
    std::cerr << "Texture::bindAsImage - error: texture not initialized!" << std::endl;
#else
  std::cerr << "Texture::bindAsImage - glBindImageTexture symbol not loaded!" << std::endl;
#endif
}


GLint Texture::getInternalFormat()
{
  if (!internalFormat_)
  {
    bind();
    glGetTexLevelParameteriv(target, 0, GL_TEXTURE_INTERNAL_FORMAT, &internalFormat_);
  }

  return internalFormat_;
}
// 
// bool Texture::clear( float _color )
// {
// #ifdef GL_ARB_clear_texture
//   if (supportsClearTexture() && texture)
//   {
//     glClearTexImage(texture, 0, GL_R32F, GL_FLOAT, &_color);
//     return true;
//   }
// #endif
//   return false;
// }
// 
// bool Texture::clear( const ACG::Vec2f& _color )
// {
// #ifdef GL_ARB_clear_texture
//   if (supportsClearTexture() && texture)
//   {
//     glClearTexImage(texture, 0, GL_RG32F, GL_FLOAT, _color.data());
//     return true;
//   }
// #endif
//   return false;
// }
// 
// bool Texture::clear( const ACG::Vec3f& _color )
// {
// #ifdef GL_ARB_clear_texture
//   if (supportsClearTexture() && texture)
//   {
//     glClearTexImage(texture, 0, GL_RGB32F, GL_FLOAT, _color.data());
//     return true;
//   }
// #endif
//   return false;
// }

bool Texture::clear( const ACG::Vec4f& _color )
{
#ifdef GL_ARB_clear_texture
  if (supportsClearTexture() && texture)
  {
    glClearTexImage(texture, 0, GL_RGBA, GL_FLOAT, _color.data());
    return true;
  }
#endif
  return false;
}

bool Texture::clear( const ACG::Vec4ui& _color )
{
#ifdef GL_ARB_clear_texture
  if (supportsClearTexture() && texture)
  {
    glClearTexImage(texture, 0, GL_RGBA_INTEGER, GL_UNSIGNED_INT, _color.data());
    return true;
  }
#endif
  return false;
}

bool Texture::clear( const ACG::Vec4i& _color )
{
#ifdef GL_ARB_clear_texture
  if (supportsClearTexture() && texture)
  {
    glClearTexImage(texture, 0, GL_RGBA_INTEGER, GL_INT, _color.data());
    return true;
  }
#endif
  return false;
}

bool Texture::supportsImageLoadStore()
{
  static int status = -1;

  if (status < 0)
  {
#if defined(GL_ARB_shader_image_load_store)
    // core in version 4.2
    status = checkExtensionSupported("ARB_shader_image_load_store") || openGLVersion(4,2);
#else
    // symbol missing, install latest glew version
    status = 0;
#endif
  }

  return status > 0;
}


bool Texture::supportsTextureBuffer()
{
  static int status = -1;

  if (status < 0)
  {
    // core in version 3.0
    status = checkExtensionSupported("EXT_texture_buffer") || openGLVersion(3,1);
  }

  return status > 0;
}


bool Texture::supportsClearTexture()
{
  static int status = -1;

  if (status < 0)
  {
#if defined(GL_ARB_clear_texture)
    status = checkExtensionSupported("ARB_clear_texture");
#else
    // symbol missing, install latest glew version
    status = 0;
#endif
  }

  return status > 0;
}

bool Texture::supportsGenerateMipmap()
{
  static int status = -1;

  if (status < 0)
  {
#if defined(GL_SGIS_generate_mipmap)
    status = checkExtensionSupported("GL_SGIS_generate_mipmap");
#else
    // symbol missing, install latest glew version
    status = 0;
#endif
  }

  return status > 0;
}

//-----------------------------------------------------------------------------


Texture1D::Texture1D( GLenum unit ) : Texture(GL_TEXTURE_1D, unit),
  width_(0),
  format_(0), type_(0)
{}

void Texture1D::setData(GLint _level, 
  GLint _internalFormat, 
  GLsizei _width, 
  GLenum _format,
  GLenum _type,
  const GLvoid* _data) {

    bind();

    glTexImage1D(GL_TEXTURE_1D, _level, _internalFormat, _width, 0, _format, _type, _data);

    width_ = _width;
    internalFormat_ = _internalFormat;
    format_ = _format;
    type_ = _type;
}


void Texture1D::setStorage( GLsizei _levels, GLenum _internalFormat, GLsizei _width ) {
#ifdef GL_ARB_texture_storage
  bind();
  glTexStorage1D(GL_TEXTURE_1D, _levels, _internalFormat, _width);

  width_ = _width;
  internalFormat_ = _internalFormat;

  GLFormatInfo finfo(_internalFormat);
  format_ = finfo.format();
  type_ = finfo.type();
#endif // GL_ARB_texture_storage
}


bool Texture1D::getData( GLint _level, void* _dst ) {
  if (is_valid()) {
    GLint curTex = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_1D, &curTex);

    bind();
    glGetTexImage(GL_TEXTURE_1D, _level, format_, type_, _dst);

    glBindTexture(GL_TEXTURE_1D, curTex);

    return true;
  }
  return false;
}

bool Texture1D::getData( GLint _level, std::vector<char>& _dst ) {
  if (is_valid()) {

    GLFormatInfo finfo(internalFormat_);

    if (finfo.isValid()) {
      size_t bufSize = finfo.elemSize() * width_;

      if (_dst.size() < bufSize)
        _dst.resize(bufSize);

      if (!_dst.empty())
        return getData(_level, &_dst[0]);
    }
  }
  return false;
}





//-----------------------------------------------------------------------------

Texture2D::Texture2D(GLenum unit)
  : Texture(GL_TEXTURE_2D, unit),
  width_(0), height_(0),
  format_(0), type_(0),
  buildMipsCPU_(false)
{}

//-----------------------------------------------------------------------------

bool Texture2D::autogenerateMipMaps()
{
    if(openGLVersion(3,0))
    {
        // From OpenGL 3.0, glGenerateMipmap is supported and we should use that
        // but glGenerateMipmap must be called AFTER the data was uploaded.
        return false;
    }

#ifdef GL_SGIS_generate_mipmap
  if (supportsGenerateMipmap())
  {
      parameter(GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
    return true;
  }
#endif
  // hardware accelerated generation is not available, fall back to software implementation
  buildMipsCPU_ = true;
  return false;
}

//-----------------------------------------------------------------------------

void Texture2D::disableAutogenerateMipMaps()
{
#ifdef GL_SGIS_generate_mipmap
  if (supportsGenerateMipmap())
    parameter(GL_GENERATE_MIPMAP_SGIS, GL_FALSE);
#endif
  buildMipsCPU_ = false;
}

//-----------------------------------------------------------------------------

void Texture2D::setData(GLint _level,
  GLint _internalFormat,
  GLsizei _width,
  GLsizei _height,
  GLenum _format,
  GLenum _type,
  const GLvoid* _data,
  bool _mipmaps) {

  if (getUnit() == GL_NONE)
    setUnit(GL_TEXTURE0);

  bind();

  if (buildMipsCPU_ && _level == 0)
    buildMipMaps(_internalFormat, _width, _height, _format, _type, _data);
  else
    glTexImage2D(GL_TEXTURE_2D, _level, _internalFormat, _width, _height, 0, _format, _type, _data);

  // If mip maps should not be built on cpu and the OpenGL version is high enough, we go the default way
  if(_mipmaps && !buildMipsCPU_ && openGLVersion(3,0))
      glGenerateMipmap(GL_TEXTURE_2D);

  width_ = _width;
  height_ = _height;
  internalFormat_ = _internalFormat;
  format_ = _format;
  type_ = _type;
}


void Texture2D::setStorage( GLsizei _levels, GLenum _internalFormat, GLsizei _width, GLsizei _height ) {
#ifdef GL_ARB_texture_storage
  bind();
  glTexStorage2D(GL_TEXTURE_2D, _levels, _internalFormat, _width, _height);

  width_ = _width;
  height_ = _height;
  internalFormat_ = _internalFormat;

  GLFormatInfo finfo(_internalFormat);
  format_ = finfo.format();
  type_ = finfo.type();
#endif // GL_ARB_texture_storage
}


bool Texture2D::getData( GLint _level, void* _dst ) {
  if (is_valid()) {
    GLint curTex = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &curTex);

    bind();
    glGetTexImage(GL_TEXTURE_2D, _level, format_, type_, _dst);

    glBindTexture(GL_TEXTURE_2D, curTex);

    return true;
  }
  return false;
}

bool Texture2D::getData( GLint _level, std::vector<char>& _dst ) {
  if (is_valid()) {

    GLFormatInfo finfo(internalFormat_);

    if (finfo.isValid()) {
      size_t bufSize = finfo.elemSize() * width_ * height_;

      if (_dst.size() < bufSize)
        _dst.resize(bufSize);

      if (!_dst.empty())
        return getData(_level, &_dst[0]);
    }
  }
  return false;
}

template<class T>
void Texture2D_buildMipMaps_DataInterpreter(Vec4f* _dst, int _numChannels, int _srcOffset, const void* _src)
{
  const T* dataT = static_cast<const T*>(_src);

  for (int i = 0; i < _numChannels; ++i)
    (*_dst)[i] = float(dataT[_srcOffset + i]);
}

void Texture2D::buildMipMaps( GLenum _internalfmt, 
  GLint _width, 
  GLint _height,
  GLenum _format,
  GLenum _type, 
  const void* _data )
{
//   gluBuild2DMipmaps(_target, _internalfmt, _width, _height, _format, _type, _data);
//   return;
// 
  if (_data)
  {
    GLFormatInfo finfo(_internalfmt);

    if (finfo.isValid() && (finfo.isFloat() || finfo.isNormalized()))
    {
      int numChannels = finfo.channelCount();


      // avoid quantization error for smaller mipmaps
      // -> treat image data as floats instead of normalized ubytes


      // compute number of mipmaps

      Vec2i curSize = Vec2i(_width, _height);

      std::vector<int> mipMemsize(1, 0);
      std::vector<Vec2i> mipSize(1, curSize);
      // mipmap count is usually a small number, so push_back() shouldn't be problematic
      mipMemsize.reserve(16);
      mipSize.reserve(16);

      int numMips = 1; // first level

      // downscale width and height by 2 until 1x1 texture
      while (curSize[0] > 1 || curSize[1] > 1)
      {
        for (int k = 0; k < 2; ++k)
          curSize[k] = std::max(1, curSize[k] >> 1);

        // tex dimension
        mipSize.push_back(curSize);

        // size in bytes
        int numPixels = curSize[0] * curSize[1];
        mipMemsize.push_back(numPixels * numChannels * 4);

        ++numMips;
      }

      // compute size in bytes required for the complete mipmap chain starting at level 1
      std::vector<int> mipOffset; // offset in bytes
      mipOffset.reserve(16);
      int totalMemSize = 0;
      for (int mipID = 0; mipID < numMips; ++mipID)
      {
        mipOffset.push_back(totalMemSize);
        totalMemSize += mipMemsize[mipID];
      }


      // alloc memory block for the mipmaps
      std::vector<float> mipData(totalMemSize / 4);

      // downsample
      for (int mipID = 1; mipID < numMips; ++mipID)
      {
        Vec2i srcSize = mipSize[mipID-1];
        Vec2i dstSize = mipSize[mipID];

        int srcOffset = mipOffset[mipID-1];
        int dstOffset = mipOffset[mipID];

        int dstNumPixels = dstSize[0] * dstSize[1];

        // loop is parallelizable, but synchronization overhead is too high
// #ifdef USE_OPENMP
// #pragma omp parallel for
// #endif // USE_OPENMP
        for (int dstPixel = 0; dstPixel < dstNumPixels; ++dstPixel)
        {
          int x = dstPixel % dstSize[0];
          int y = dstPixel / dstSize[0];

          Vec4f pixelData[4];

          Vec2i srcPixelPos[4] =
          {
            Vec2i(x * 2, y * 2), Vec2i(x * 2 + 1, y * 2),
            Vec2i(x * 2, y * 2 + 1), Vec2i(x * 2 + 1, y * 2 + 1)
          };

          Vec4f avgColor = Vec4f(0.0f, 0.0f, 0.0f, 0.0f);

          // load the four source pixels
          for (int srcPixel = 0; srcPixel < 4; ++srcPixel)
          {
            // init with black
            pixelData[srcPixel] = Vec4f(0.0f, 0.0f, 0.0f, 1.0f);

            // clamp pixel position 
            srcPixelPos[srcPixel][0] = std::min(srcPixelPos[srcPixel][0], srcSize[0] - 1);
            srcPixelPos[srcPixel][1] = std::min(srcPixelPos[srcPixel][1], srcSize[1] - 1);

            // linear position of 2d pixel pos, row-major
            int srcPixelPosLinear = srcSize[0] * srcPixelPos[srcPixel][1] + srcPixelPos[srcPixel][0];

            // interpret pixel of the input image based on type
            if (mipID == 1)
            {
              switch ( _type )
              {
              case GL_DOUBLE: Texture2D_buildMipMaps_DataInterpreter<double>(pixelData + srcPixel, numChannels, srcPixelPosLinear * numChannels, _data); break;
              case GL_FLOAT: Texture2D_buildMipMaps_DataInterpreter<float>(pixelData + srcPixel, numChannels, srcPixelPosLinear * numChannels, _data); break;
              case GL_INT: Texture2D_buildMipMaps_DataInterpreter<int>(pixelData + srcPixel, numChannels, srcPixelPosLinear * numChannels, _data); break;
              case GL_UNSIGNED_INT: Texture2D_buildMipMaps_DataInterpreter<unsigned int>(pixelData + srcPixel, numChannels, srcPixelPosLinear * numChannels, _data); break;
              case GL_SHORT: Texture2D_buildMipMaps_DataInterpreter<short>(pixelData + srcPixel, numChannels, srcPixelPosLinear * numChannels, _data); break;
              case GL_UNSIGNED_SHORT: Texture2D_buildMipMaps_DataInterpreter<unsigned short>(pixelData + srcPixel, numChannels, srcPixelPosLinear * numChannels, _data); break;
              case GL_BYTE:
                {
                  Texture2D_buildMipMaps_DataInterpreter<char>(pixelData + srcPixel, numChannels, srcPixelPosLinear * numChannels, _data);

                  if (finfo.isNormalized())
                    pixelData[srcPixel] /= 127.0f;
                } break;
              case GL_UNSIGNED_BYTE:
                {
                  Texture2D_buildMipMaps_DataInterpreter<unsigned char>(pixelData + srcPixel, numChannels, srcPixelPosLinear * numChannels, _data);

                  if (finfo.isNormalized())
                    pixelData[srcPixel] /= 255.0f;
                } break;

              default: std::cerr << "MipMaps: unknown data type: " << _type << std::endl;
              }
            }
            else
            {
              // load from previously computed mipmap

              for (int c = 0; c < numChannels; ++c)
                pixelData[srcPixel][c] = mipData[srcOffset/4 + srcPixelPosLinear * numChannels + c];
            }

            avgColor += pixelData[srcPixel];
          }

          avgColor *= 0.25f;

          // store average of the source pixels
          int dstPixelPosLinear = y * dstSize[0] + x;
          for (int c = 0; c < numChannels; ++c)
            mipData[dstOffset / 4 + dstPixelPosLinear * numChannels + c] = avgColor[c];
        }

      }


      // upload mipmaps to gpu

      for (int mipID = 0; mipID < numMips; ++mipID)
      {
        // inpute image at level 0
        const void* mipDataPtr = _data;
        GLenum mipDataType = _type;

        if (mipID > 0)
        {
          // downsampled image at lower levels
          // these are stored as float textures in memory
          // glTexImage2D converts float data to the requested internal format
          mipDataPtr = &mipData[mipOffset[mipID] / 4];
          mipDataType = GL_FLOAT;
        }

        glTexImage2D(getTarget(), mipID, _internalfmt, mipSize[mipID][0], mipSize[mipID][1], 0, _format, mipDataType, mipDataPtr);
      }
    }
  }
}


bool Texture2D::loadFromFile( const std::string& _filename, GLenum _minFilter, GLenum _magFilter )
{
  bool success = false;

  const int numMipmapEnums = 4;
  GLenum mipmapEnums[numMipmapEnums] = {GL_NEAREST_MIPMAP_NEAREST, GL_NEAREST_MIPMAP_LINEAR, GL_LINEAR_MIPMAP_NEAREST, GL_LINEAR_MIPMAP_LINEAR};
  bool mipmaps = false;

  for (int i = 0; i < numMipmapEnums; ++i)
    mipmaps = mipmaps || _minFilter == mipmapEnums[i];

  if (!_filename.empty())
  {
    bind();

    QImage qtex;

    if (qtex.load(_filename.c_str()))
    {
      success = true;

      if (mipmaps)
        autogenerateMipMaps();

      QImage gltex = ACG::Util::convertToGLFormat(qtex);

      setData(0, GL_RGBA, gltex.width(), gltex.height(), GL_RGBA, GL_UNSIGNED_BYTE, gltex.bits(), mipmaps);
    }

  }

  if (success)
  {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, _minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, _magFilter);
  }

  return success;
}


void Texture2D::loadRandom( GLint _internalFormat, GLsizei _width, GLsizei _height )
{
  ACG::GLFormatInfo finfo(_internalFormat);

  if (finfo.isValid() && _width && _height)
  {
    int n = _width * _height * finfo.channelCount();

    GLvoid* dataPtr = 0;

    std::vector<float> randF;
    std::vector<int> randI;

    GLenum gltype = 0;

    if (finfo.isFloat() || finfo.isNormalized())
    {
      randF.resize(n);

      bool isSigned = finfo.isInt();

      for (int i = 0; i < n; ++i)
      {
        float r = float(rand()) / float(RAND_MAX);

        if (isSigned)
          r = r * 2.0f - 1.0f;

        randF[i] = r;
      }

      dataPtr = &randF[0];
      gltype = GL_FLOAT;
    }
    else
    {
      randI.resize(n);

      for (int i = 0; i < n; ++i)
        randI[i] = rand();

      dataPtr = &randI[0];
      gltype = GL_INT;
    }
    
    bind();
    setData(0, _internalFormat, _width, _height, finfo.format(), gltype, dataPtr);
  }
}

bool Texture2D::checkTextureMem( GLenum _internalFormat, GLsizei _width, GLsizei _height, GLenum _format)
{
  GLuint t = 0;
  glGenTextures(1, &t);

  bool res = false;

  if (t)
  {
    GLint savedTex = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &savedTex);


    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_PROXY_TEXTURE_2D, 0, _internalFormat, _width, _height, 0, _format, GL_FLOAT, 0);

    GLint w = 0;
    glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &w);
    if (w) 
      res = true;

    glBindTexture(GL_TEXTURE_2D, savedTex);
    glDeleteTextures(1, &t);
  }

  return res;
}

//-----------------------------------------------------------------------------

#if defined(GL_VERSION_1_5)

void VertexBufferObject::del() {
    if (valid)
        glDeleteBuffers(1, &vbo);
    valid = false;
}

void VertexBufferObject::upload(
        GLsizeiptr size, const GLvoid* data, GLenum usage) {

  if (!valid)
    gen();
  
  bind();

  glBufferData(target, size, data, usage);
}

void VertexBufferObject::uploadSubData(
        GLuint _offset, GLuint _size, const GLvoid* _data ) {

  glBufferSubData(target, _offset, _size, _data);
}

void VertexBufferObject::gen() {
    glGenBuffers(1, &vbo);
    if(vbo > 0u)
        valid = true;
}

int VertexBufferObject::size() {
  bind();
  int bufsize = 0;
  glGetBufferParameteriv(target, GL_BUFFER_SIZE, &bufsize);
  return bufsize;
}

#endif


//-----------------------------------------------------------------------------

TextureBuffer::TextureBuffer(GLenum u)
  : 
Texture(GL_TEXTURE_BUFFER, u), 
  bufferSize_(0), buffer_(0), usage_(0), fmt_(0) {
}


TextureBuffer::~TextureBuffer() {
  if (buffer_)
    glDeleteBuffers(1, &buffer_);
}

void TextureBuffer::setBufferData(
        size_t _size, const void* _data, GLenum _internalFormat, GLenum _usage) {
  if (supportsTextureBuffer()) {
    // setup buffer object
    if (!buffer_)
        glGenBuffers(1, &buffer_);

    glBindBuffer(GL_TEXTURE_BUFFER, buffer_);
    glBufferData(GL_TEXTURE_BUFFER, static_cast<GLsizei>(_size), _data, _usage);

    usage_ = _usage;
    fmt_ = _internalFormat;

    // bind buffer to texture
    if (getUnit() == GL_NONE)
        setUnit(GL_TEXTURE0);

    bind();

    glTexBuffer(GL_TEXTURE_BUFFER, _internalFormat, buffer_);

    bufferSize_ = _size;
  }
  else
    std::cerr << "TextureBuffer::setData - gpu does not support buffer textures!" << std::endl;
}

bool TextureBuffer::getBufferData(void* _dst) {
if(!ACG::compatibilityProfile())
{
  if (buffer_) {
    glBindBuffer(GL_TEXTURE_BUFFER, buffer_);
    glGetBufferSubData(GL_TEXTURE_BUFFER, 0, bufferSize_, _dst);
    return true;
  }
  else
    std::cerr << "TextureBuffer::getBufferData - gpu does not support buffer textures!" << std::endl;
}else{
    std::cerr << "TextureBuffer::getBufferData - currently only in core profile available!" << std::endl;
}
  return false;
}

bool TextureBuffer::getBufferData(std::vector<char>& _dst) {
  if (_dst.size() < size_t(bufferSize_))
    _dst.resize(bufferSize_);

  if (!_dst.empty())
    return getBufferData(&_dst[0]);

  return false;
}



//-----------------------------------------------------------------------------


#if defined(GL_NV_vertex_program) || defined(GL_NV_fragment_program)

void ProgramBaseNV::bind() {
    if (!valid)
        gen();
    glBindProgramARB(target, program);
}

void ProgramBaseNV::unbind() {
    glBindProgramARB(target, 0);
}

bool ProgramBaseNV::load(const char* prog_text) {
    int size = int(strlen(prog_text));
    if (!valid)
        gen();
    glLoadProgramNV(target, program, size, (const GLubyte *) prog_text);
    GLint errpos;
    glGetIntegerv(GL_PROGRAM_ERROR_POSITION_NV, &errpos);
    if (errpos != -1) {
        fprintf(stderr, "\nprogram error:\n");
        int bgn = std::max(0, errpos - 10), end = std::min(size, bgn + 30);
        for (int i = bgn; i < end; ++i)
            fputc(prog_text[i], stderr);
        fputc('\n', stderr);
        return false;
    }
    return true;
}


void ProgramBaseNV::gen() {
    glGenProgramsARB(1, &program);
    valid = true;
}

void ProgramBaseNV::del() {
    if (valid)
        glDeleteProgramsARB(1, &program);
    valid = false;
}

#endif

#if defined(GL_ARB_vertex_program) || defined(GL_ARB_fragment_program)

void ProgramBaseARB::bind() {
    if (!valid)
        gen();
    glBindProgramARB(target, program);
}
void ProgramBaseARB::unbind() {
    glBindProgramARB(target, 0);
}

bool ProgramBaseARB::load(const char* prog_text) {
    int size = int(strlen(prog_text));
    if (!valid)
        gen();
    bind();
    glProgramStringARB(target, GL_PROGRAM_FORMAT_ASCII_ARB, size, prog_text);
    GLint errpos;
    glGetIntegerv(GL_PROGRAM_ERROR_POSITION_ARB, &errpos);
    if (errpos != -1) {
        fprintf(stderr, "\nprogram error:\n");
        int bgn = std::max(0, errpos - 10), end = std::min(size, bgn + 30);
        for (int i = bgn; i < end; ++i)
            fputc(prog_text[i], stderr);
        fputc('\n', stderr);
        return false;
    }
    return true;
}

void ProgramBaseARB::gen() {
    glGenProgramsARB(1, &program);
    valid = true;
}
void ProgramBaseARB::del() {
    if (valid)
        glDeleteProgramsARB(1, &program);
    valid = false;
}

#endif // GL_ARB_vertex_program


//-----------------------------------------------------------------------------

// support state unknown : -1
int VertexArrayObject::supportStatus_ = -1;

VertexArrayObject::VertexArrayObject() 
  : id_(0)
{
}

VertexArrayObject::~VertexArrayObject()
{
#ifdef GL_ARB_vertex_array_object
  if (id_)
    glDeleteVertexArrays(1, &id_);
#endif
}


void VertexArrayObject::bind()
{
#ifdef GL_ARB_vertex_array_object
  if (!id_)
    init();

  if (id_)
    glBindVertexArray(id_);
#endif
}

void VertexArrayObject::unbind()
{
#ifdef GL_ARB_vertex_array_object
  glBindVertexArray(0);
#endif
}

void VertexArrayObject::init()
{
#ifdef GL_ARB_vertex_array_object
  if (id_)
    glDeleteVertexArrays(1, &id_);

  glGenVertexArrays(1, &id_);
#endif
}

bool VertexArrayObject::isSupported()
{
#ifndef GL_ARB_vertex_array_object
  // missing definition in gl header!
  supportStatus_ = 0;
#else

  if (supportStatus_ < 0)
    supportStatus_ = checkExtensionSupported("GL_ARB_vertex_array_object") ? 1 : 0;
  if( openGLVersion(3,2) )
      return true;
#endif

  return supportStatus_ > 0;
}



//-----------------------------------------------------------------------------



// support state unknown : -1
int AtomicCounter::supportStatus_ = -1;

AtomicCounter::AtomicCounter(int _numCounters)
  : numCounters_(_numCounters), buffer_(0)
{
}

AtomicCounter::~AtomicCounter()
{
  if (buffer_)
    glDeleteBuffers(1, &buffer_);
}

void AtomicCounter::init()
{
  // check support and initialize
#ifdef GL_ARB_shader_atomic_counters
  if (isSupported() && numCounters_ > 0)
  {
    glGenBuffers(1, &buffer_);
    bind();
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, numCounters_ * sizeof(unsigned int), 0, GL_DYNAMIC_COPY);
    unbind();
  }
#endif

  if (!isValid())
    std::cerr << "atomic counter failed to initialize!" << std::endl;
}

void AtomicCounter::bind()
{
#ifdef GL_ARB_shader_atomic_counters
  // implicit initialization
  if (!isValid())
    init();

  if (isValid())
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, buffer_);
#endif
}

void AtomicCounter::bind(GLuint _index)
{
#ifdef GL_ARB_shader_atomic_counters
  // implicit initialization
  if (!isValid())
    init();

  if (isValid())
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, _index, buffer_);
#endif
}

void AtomicCounter::unbind()
{
#ifdef GL_ARB_shader_atomic_counters
  glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
#endif
}

void AtomicCounter::set(unsigned int _value)
{
#ifdef GL_ARB_shader_atomic_counters
  // implicit initialization
  bind();

  if (isValid())
  {
    const size_t bufSize = numCounters_ * sizeof(unsigned int);
    //     unsigned int* bufData = new unsigned int[numCounters_];
    //     memset(bufData, int(_value), bufSize);
    // 
    //     glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, bufSize, bufData);
    //     delete [] bufData;

    void* bufPtr = glMapBufferRange(GL_ATOMIC_COUNTER_BUFFER, 0, bufSize, GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    memset(bufPtr, int(_value), bufSize);
    glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

    unbind();
  }
#endif
}

void AtomicCounter::get(unsigned int* _out)
{
#ifdef GL_ARB_shader_atomic_counters
  if (isValid())
  {
    bind();

    const size_t bufSize = numCounters_ * sizeof(unsigned int);

    // doesnt work, driver crash on ati:
    //    glGetBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, bufSize, _out);

    void* bufPtr = glMapBufferRange(GL_ATOMIC_COUNTER_BUFFER, 0, bufSize, GL_MAP_READ_BIT);
    memcpy(_out, bufPtr, bufSize);
    glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

    unbind();
  }
#endif
}

bool AtomicCounter::isSupported()
{
#ifndef GL_ARB_shader_atomic_counters
  // missing definition in gl header!
  supportStatus_ = 0;
#else

  if (supportStatus_ < 0)
    supportStatus_ = checkExtensionSupported("GL_ARB_shader_atomic_counters") ? 1 : 0;
#endif

  return supportStatus_ > 0;
}

bool AtomicCounter::isValid() const
{
  return buffer_ && numCounters_ > 0;
}

//-----------------------------------------------------------------------------

QueryObject::QueryObject(GLenum _type) 
  : id_(0), state_(-1), type_(_type)
{

}

QueryObject::~QueryObject()
{
  if (id_)
    glDeleteQueries(1, &id_);
}

void QueryObject::begin()
{
  if (!id_)
    glGenQueries(1, &id_);

  glBeginQuery(type_, id_);
  state_ = 0;
}

void QueryObject::end()
{
  if (!state_)
  {
    glEndQuery(type_);
    state_ = 1;
  }
}

bool QueryObject::available() const
{
  GLint r = GL_FALSE;
  if (state_ > 0)
    glGetQueryObjectiv(id_, GL_QUERY_RESULT_AVAILABLE, &r);
  return r != GL_FALSE;
}

GLuint QueryObject::result() const
{
  GLuint r = 0xffffffff;
  if (state_ > 0)
    glGetQueryObjectuiv(id_, GL_QUERY_RESULT, &r);
  return r;
}


//-----------------------------------------------------------------------------

int QueryCounter::supportStatus_ = -1;

QueryCounter::QueryCounter()
  : state_(-1)
{
  queryObjects_[0] = queryObjects_[1] = 0;
}

QueryCounter::~QueryCounter()
{
  if (queryObjects_[0])
    glDeleteQueries(2, queryObjects_);
}


void QueryCounter::restart()
{
#ifdef GL_ARB_timer_query
  if (isSupported())
  {
    state_ = 0;

    if (!queryObjects_[0])
      glGenQueries(2, queryObjects_);

    glQueryCounter(queryObjects_[0], GL_TIMESTAMP);
  }
#endif
}

void QueryCounter::stop()
{
#ifdef GL_ARB_timer_query
  if (state_ == 0)
  {
    glQueryCounter(queryObjects_[1], GL_TIMESTAMP);
    ++state_;
  }
#endif
}

GLuint64 QueryCounter::elapsedNs()
{
  GLuint64 timing = 0;
#ifdef GL_ARB_timer_query
  stop();

  if (state_ == 1)
  {
    GLint available = 0;
    while (!available)
      glGetQueryObjectiv(queryObjects_[1], GL_QUERY_RESULT_AVAILABLE, &available);

    GLuint64 timeStart;
    glGetQueryObjectui64v(queryObjects_[0], GL_QUERY_RESULT, &timeStart);
    glGetQueryObjectui64v(queryObjects_[1], GL_QUERY_RESULT, &timing);
    timing -= timeStart;
  }
#endif
  return timing;
}

GLuint64 QueryCounter::elapsedMs()
{
  return elapsedNs() / 1000;
}

float QueryCounter::elapsedSecs()
{
  GLuint64 ms = elapsedMs();

  return float(ms) / 1000.0f;
}

bool QueryCounter::isSupported()
{
#ifndef GL_ARB_timer_query
  // missing definition in gl header!
  supportStatus_ = 0;
#else

  if (supportStatus_ < 0)
    supportStatus_ = checkExtensionSupported("GL_ARB_timer_query") || openGLVersion(3,2) ? 1 : 0;
#endif

  return supportStatus_ > 0;
}


//-----------------------------------------------------------------------------


// support state unknown : -1
int UniformBufferObject::supportStatus_ = -1;
int UniformBufferObject::maxBlockSize_ = -1;
int UniformBufferObject::maxBindings_ = -1;
int UniformBufferObject::maxCombinedShaderBlocks_ = -1;
int UniformBufferObject::offsetAlignment_ = -1;

UniformBufferObject::UniformBufferObject()
  : VertexBufferObject(
#ifndef GL_ARB_uniform_buffer_object
  GL_NONE
#else
  GL_UNIFORM_BUFFER
#endif
  ),
  data_(0)
{
}

UniformBufferObject::~UniformBufferObject()
{
}

void UniformBufferObject::bind( GLuint _index )
{
#ifdef GL_ARB_uniform_buffer_object
  glBindBufferBase(GL_UNIFORM_BUFFER, _index, id());
#endif
}


bool UniformBufferObject::isSupported()
{
#ifndef GL_ARB_uniform_buffer_object
  // missing definition in gl header!
  supportStatus_ = 0;
#else

  if (supportStatus_ < 0)
    supportStatus_ = checkExtensionSupported("GL_ARB_uniform_buffer_object") ? 1 : 0;
#endif

  return supportStatus_ > 0;
}

void UniformBufferObject::queryCaps()
{
#ifdef GL_ARB_uniform_buffer_object
  if (isSupported())
  {
    glGetIntegerv(GL_MAX_UNIFORM_BUFFER_BINDINGS, &maxBindings_);
    glGetIntegerv(GL_MAX_UNIFORM_BLOCK_SIZE, &maxBlockSize_);
    glGetIntegerv(GL_MAX_COMBINED_UNIFORM_BLOCKS, &maxCombinedShaderBlocks_);
    glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &offsetAlignment_);
  }
#endif
}

int UniformBufferObject::getMaxBindings()
{
  if (maxBindings_ < 0)
    queryCaps();

  return maxBindings_;
}

int UniformBufferObject::getMaxBlocksize()
{
  if (maxBlockSize_ < 0)
    queryCaps();

  return maxBlockSize_;
}

int UniformBufferObject::getMaxCombinedShaderBlocks()
{
  if (maxCombinedShaderBlocks_ < 0)
    queryCaps();

  return maxCombinedShaderBlocks_;
}

int UniformBufferObject::getOffsetAlignment()
{
  if (offsetAlignment_ < 0)
    queryCaps();

  return offsetAlignment_;
}

void UniformBufferObject::setUniformData( GLSL::Program* _prog, const char* _bufferName, const char* _uniformName, const void* _data, int _datasize, bool _delay )
{
  if (_prog && _bufferName && _uniformName && _data)
  {
    GLuint idx = _prog->getUniformBlockIndex(_bufferName);

    if (idx != GL_INVALID_INDEX)
    {
      size_t bufsize = size_t(_prog->getUniformBlockSize(idx));

      if (data_.size() != bufsize)
        data_.resize(bufsize, 0);

      int offset = -1;
      _prog->getUniformBlockOffsets(1, &_uniformName, &offset);

      if (offset >= 0)
      {
        memcpy(&data_[offset], _data, _datasize);

        if (!_delay)
        {
          VertexBufferObject::bind();

          if (size() != int(bufsize))
            VertexBufferObject::upload(bufsize, &data_[0], GL_DYNAMIC_DRAW);
          else
            uploadSubData(offset, _datasize, _data);
        }
      }
    }
  }
}

void UniformBufferObject::upload()
{
  if (!data_.empty())
  {
    VertexBufferObject::bind();

    VertexBufferObject::upload(data_.size(), &data_[0], GL_DYNAMIC_DRAW);
  }
}


//-----------------------------------------------------------------------------



// support state unknown : -1
int ShaderStorageBufferObject::supportStatus_ = -1;
int ShaderStorageBufferObject::maxBlockSize_ = -1;
int ShaderStorageBufferObject::maxBindings_ = -1;
int ShaderStorageBufferObject::maxCombinedShaderBlocks_ = -1;

ShaderStorageBufferObject::ShaderStorageBufferObject()
  : VertexBufferObject(
#ifndef GL_ARB_shader_storage_buffer_object
   GL_NONE
#else
   GL_SHADER_STORAGE_BUFFER
#endif
   )
{
}

ShaderStorageBufferObject::~ShaderStorageBufferObject()
{
}

void ShaderStorageBufferObject::bind( GLuint _index )
{
#ifdef GL_ARB_shader_storage_buffer_object
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _index, id());
#endif
}

bool ShaderStorageBufferObject::isSupported()
{
#ifndef GL_ARB_shader_storage_buffer_object
  // missing definition in gl header!
  supportStatus_ = 0;
#else

  if (supportStatus_ < 0)
    supportStatus_ = checkExtensionSupported("GL_ARB_shader_storage_buffer_object") ? 1 : 0;
#endif

  return supportStatus_ > 0;
}

void ShaderStorageBufferObject::queryCaps()
{
#ifdef GL_ARB_shader_storage_buffer_object
  if (isSupported())
  {
    glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &maxBindings_);
    glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &maxBlockSize_);
    glGetIntegerv(GL_MAX_COMBINED_SHADER_STORAGE_BLOCKS, &maxCombinedShaderBlocks_);
  }
#endif
}

int ShaderStorageBufferObject::getMaxBindings()
{
  if (maxBindings_ < 0)
    queryCaps();

  return maxBindings_;
}

int ShaderStorageBufferObject::getMaxBlocksize()
{
  if (maxBlockSize_ < 0)
    queryCaps();

  return maxBlockSize_;
}

int ShaderStorageBufferObject::getMaxCombinedShaderBlocks()
{
  if (maxCombinedShaderBlocks_ < 0)
    queryCaps();

  return maxCombinedShaderBlocks_;
}





} /* namespace ACG */

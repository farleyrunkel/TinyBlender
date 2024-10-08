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
//  CLASS FBO - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include <ACG/GL/acg_glew.hh>
#include "FBO.hh"
#include "GLState.hh"
#include "GLError.hh"
#include "GLFormatInfo.hh"

//== NAMESPACES ===============================================================

namespace ACG
{

//== IMPLEMENTATION ==========================================================


FBO::RenderTexture::RenderTexture()
  : id(0), target(0), internalFormat(0), format(0), gltype(0),
  dim(0,0,0), wrapMode(0), minFilter(0), magFilter(0), owner(false)
{
}


FBO::FBO()
: fbo_(0), depthbuffer_(0), stencilbuffer_(0), width_(0), height_(0), samples_(0), fixedsamplelocation_(GL_TRUE), prevFbo_(0), prevDrawBuffer_(GL_NONE)
{
}

FBO::
~FBO()
{
  del();
}

//-----------------------------------------------------------------------------

void
FBO::
init()
{
  // Create framebuffer object

    //EXT_framebuffer_object was removed in OpenGL version 3.1
    //hence no core profile has the extension explicitly, but fbos
    //were adopted in opengl v 3.0 so don use this extension on core profiles
  if(!ACG::openGLVersion(3,0))
  {
    if (!checkExtensionSupported("GL_EXT_framebuffer_object")) {
        std::cerr << "Framebuffer object not supported! " << std::endl;
        exit( 1 );
    }
  }

  // test whether fbo hasn't been created before
  if(!fbo_)
    glGenFramebuffers( 1, &fbo_ );

  // check status
  checkFramebufferStatus();
}

//-----------------------------------------------------------------------------

void FBO::del()
{
  // delete framebuffer object
  if(fbo_)
    glDeleteFramebuffers( 1, &fbo_ );

  // delete render buffer
  if(depthbuffer_)
    glDeleteRenderbuffers(1, &depthbuffer_);

  // delete stencil buffer
  if(stencilbuffer_)
    glDeleteRenderbuffers(1, &stencilbuffer_);

  for (AttachmentList::iterator it = attachments_.begin(); it != attachments_.end(); ++it)
    if (it->second.id && it->second.owner)
      glDeleteTextures(1, &it->second.id);
}


//-----------------------------------------------------------------------------

void FBO::attachTexture( GLenum _attachment, GLuint _texture, GLuint _level )
{
#ifdef GL_VERSION_3_2
  // bind fbo
  bind();

  // add texture to frame buffer object
  glFramebufferTexture( GL_FRAMEBUFFER_EXT, _attachment, _texture, _level );

//   GLint layered = 0;
//   glGetFramebufferAttachmentParameteriv( GL_FRAMEBUFFER_EXT, _attachment, GL_FRAMEBUFFER_ATTACHMENT_LAYERED, &layered);

  checkGLError();

  // check status
  checkFramebufferStatus();

  // unbind fbo
  unbind();


  // store texture id in internal array
  RenderTexture intID;
  intID.id = _texture;

  // free previously bound texture
  const RenderTexture& prevTex = attachments_[_attachment];
  if (prevTex.owner && prevTex.id)
    glDeleteTextures(1, &prevTex.id);

  // track texture id
  attachments_[_attachment] = intID;
#else
  std::cerr << "error: FBO::attachTexture unsupported - update glew headers and rebuild" << std::endl;
#endif
}

//-----------------------------------------------------------------------------

void
FBO::
attachTexture2D( GLenum _attachment, GLuint _texture, GLenum _target )
{
  // bind fbo
  bind();

  // add texture to frame buffer object
  glFramebufferTexture2D( GL_FRAMEBUFFER_EXT, _attachment, _target, _texture, 0 );

  checkGLError();

  // check status
  checkFramebufferStatus();

  // unbind fbo
  unbind();


  // store texture id in internal array
  RenderTexture intID;
  intID.id = _texture;
  intID.target = _target;

  // free previously bound texture
  const RenderTexture& prevTex = attachments_[_attachment];
  if (prevTex.owner && prevTex.id)
    glDeleteTextures(1, &prevTex.id);

  // track texture id
  attachments_[_attachment] = intID;
}

//-----------------------------------------------------------------------------

void FBO::attachTexture2D( GLenum _attachment, GLsizei _width, GLsizei _height, GLuint _internalFmt, GLenum _format, GLint _wrapMode /*= GL_CLAMP*/, GLint _minFilter /*= GL_LINEAR*/, GLint _magFilter /*= GL_LINEAR*/ )
{
  // gen texture id
  GLuint texID;
  glGenTextures(1, &texID);

#ifdef GL_ARB_texture_multisample
  GLenum target = samples_ ? GL_TEXTURE_2D_MULTISAMPLE : GL_TEXTURE_2D;
#else
  GLenum target = GL_TEXTURE_2D;
#endif // GL_ARB_texture_multisample


  // if multisampled, texfilter must be GL_NEAREST
  // texelFetch returns darker color otherwise!
  if (samples_)
  {
    if (_minFilter != GL_NEAREST || _magFilter != GL_NEAREST)
    {
      std::cerr << "ACG::FBO - Multisampled texture must be filtered with GL_NEAREST!" << std::endl;

      _minFilter = _magFilter = GL_NEAREST;
    }
  }

  // store texture id in internal array
  RenderTexture intID;
  intID.id = texID;
  intID.internalFormat = _internalFmt;
  intID.format = _format;
  intID.gltype = GLFormatInfo(_internalFmt).type();
  intID.target = target;
  intID.dim = ACG::Vec3i(_width, _height, 1);
  intID.wrapMode = _wrapMode;
  intID.minFilter = _minFilter;
  intID.magFilter = _magFilter;
  intID.owner = true;


  // specify texture
  glBindTexture(target, texID);


#ifdef GL_ARB_texture_multisample
  if (!samples_)
  {
    glTexParameteri(target, GL_TEXTURE_WRAP_S, _wrapMode);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, _wrapMode);
    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, _minFilter);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, _magFilter);
    glTexImage2D(target, 0, _internalFmt, _width, _height, 0, _format, intID.gltype, 0);
  }
  else
    glTexImage2DMultisample(target, samples_, _internalFmt, _width, _height, fixedsamplelocation_);
#else
  glTexParameteri(target, GL_TEXTURE_WRAP_S, _wrapMode);
  glTexParameteri(target, GL_TEXTURE_WRAP_T, _wrapMode);
  glTexParameteri(target, GL_TEXTURE_MIN_FILTER, _minFilter);
  glTexParameteri(target, GL_TEXTURE_MAG_FILTER, _magFilter);
  glTexImage2D(target, 0, _internalFmt, _width, _height, 0, _format, intID.gltype, 0);
#endif // GL_ARB_texture_multisample


  checkGLError();

  width_ = _width;
  height_ = _height;

  glBindTexture(target, 0);

  // attach
  attachTexture2D(_attachment, texID, target);

  // track texture id
  attachments_[_attachment] = intID;
}

//-----------------------------------------------------------------------------

void FBO::attachTexture3D( GLenum _attachment, GLsizei _width, GLsizei _height, GLsizei _depth, GLuint _internalFmt, GLenum _format, GLint _wrapMode , GLint _minFilter , GLint _magFilter  )
{
  // gen texture id
  GLuint texID;
  glGenTextures(1, &texID);

  GLenum target = GL_TEXTURE_3D;

  // store texture id in internal array
  RenderTexture intID;
  intID.id = texID;
  intID.internalFormat = _internalFmt;
  intID.format = _format;
  intID.gltype = GLFormatInfo(_internalFmt).type();
  intID.target = target;
  intID.dim = ACG::Vec3i(_width, _height, _depth);
  intID.wrapMode = _wrapMode;
  intID.minFilter = _minFilter;
  intID.magFilter = _magFilter;
  intID.owner = true;


  // specify texture
  glBindTexture(target, texID);

  glTexParameteri(target, GL_TEXTURE_WRAP_S, _wrapMode);
  glTexParameteri(target, GL_TEXTURE_WRAP_T, _wrapMode);
  glTexParameteri(target, GL_TEXTURE_WRAP_R, _wrapMode);
  glTexParameteri(target, GL_TEXTURE_MIN_FILTER, _minFilter);
  glTexParameteri(target, GL_TEXTURE_MAG_FILTER, _magFilter);
  glTexImage3D(target, 0, _internalFmt, _width, _height, _depth, 0, _format, GL_FLOAT, 0);

  checkGLError();

  width_ = _width;
  height_ = _height;

  glBindTexture(target, 0);

  // attach
  attachTexture(_attachment, texID, 0);

  // track texture id
  attachments_[_attachment] = intID;
}

//-----------------------------------------------------------------------------

void FBO::attachTexture2DDepth( GLsizei _width, GLsizei _height, GLuint _internalFmt /*= GL_DEPTH_COMPONENT32*/, GLenum _format /*= GL_DEPTH_COMPONENT */ )
{
  attachTexture2D(GL_DEPTH_ATTACHMENT, _width, _height, _internalFmt, _format, GL_CLAMP_TO_EDGE, GL_NEAREST, GL_NEAREST);
}

//-----------------------------------------------------------------------------

void FBO::attachTexture2DStencil( GLsizei _width, GLsizei _height )
{
  attachTexture2D(GL_STENCIL_ATTACHMENT_EXT, _width, _height, GL_STENCIL_INDEX8, GL_STENCIL_INDEX, GL_CLAMP_TO_EDGE, GL_NEAREST, GL_NEAREST);
}

//-----------------------------------------------------------------------------

void
FBO::
addDepthBuffer( GLuint _width, GLuint _height )
{
  if (depthbuffer_)
    glDeleteRenderbuffers(1, &depthbuffer_);

  // create renderbuffer
  glGenRenderbuffers(1, &depthbuffer_);

  // bind renderbuffer
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer_);

  // malloc
#ifdef GL_ARB_texture_multisample
  glRenderbufferStorageMultisample(GL_RENDERBUFFER_EXT, samples_, GL_DEPTH_COMPONENT, _width, _height);
#else
  glRenderbufferStorage(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, _width, _height);
#endif

  // attach to framebuffer object
  if ( bind() )
    glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthbuffer_);

  // check status
  checkFramebufferStatus();

  // normal render mode
  unbind();
}

//-----------------------------------------------------------------------------

void
FBO::
addStencilBuffer( GLuint _width, GLuint _height )
{
  if (stencilbuffer_)
    glDeleteRenderbuffers(1, &stencilbuffer_);

  // create renderbuffer
  glGenRenderbuffers(1, &stencilbuffer_);

  // bind renderbuffer
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, stencilbuffer_);

  // malloc
#ifdef GL_ARB_texture_multisample
  glRenderbufferStorageMultisample(GL_RENDERBUFFER_EXT, samples_, GL_STENCIL_INDEX, _width, _height);
#else
  glRenderbufferStorage(GL_RENDERBUFFER_EXT, GL_STENCIL_INDEX, _width, _height);
#endif

  // attach to framebuffer object
  if ( bind() )
    glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, stencilbuffer_);

  // check status
  checkFramebufferStatus();

  // normal render mode
  unbind();
}

//-----------------------------------------------------------------------------

void 
FBO::
addDepthStencilBuffer( GLuint _width, GLuint _height )
{
  if (depthbuffer_)
    glDeleteRenderbuffers(1, &depthbuffer_);

  if (stencilbuffer_)
    glDeleteRenderbuffers(1, &stencilbuffer_);

  depthbuffer_ = stencilbuffer_ = 0;

  // create renderbuffer
  glGenRenderbuffers(1, &depthbuffer_);

  // bind renderbuffer
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer_);

  // malloc
#ifdef GL_ARB_texture_multisample
  glRenderbufferStorageMultisample(GL_RENDERBUFFER_EXT, samples_, GL_DEPTH_STENCIL, _width, _height);
#else
  glRenderbufferStorage(GL_RENDERBUFFER_EXT, GL_DEPTH_STENCIL, _width, _height);
#endif

  // attach to framebuffer object
  if (bind())
  {
    glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthbuffer_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthbuffer_);
  }

  // check status
  checkFramebufferStatus();

  // normal render mode
  unbind();
}


//-----------------------------------------------------------------------------

bool
FBO::
bind()
{
  // save previous fbo id
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, (GLint*)&prevFbo_);
  glGetIntegerv(GL_DRAW_BUFFER, (GLint*)&prevDrawBuffer_);

  if ( !fbo_ )
    init();

  if ( !fbo_)
    return false;

  // bind framebuffer object
  ACG::GLState::bindFramebuffer( GL_FRAMEBUFFER_EXT, fbo_ );


  return true;
}

//-----------------------------------------------------------------------------

void
FBO::
unbind()
{
  //set to normal rendering
  ACG::GLState::bindFramebuffer( GL_FRAMEBUFFER_EXT, prevFbo_ );
  ACG::GLState::drawBuffer( prevDrawBuffer_ );
}

//-----------------------------------------------------------------------------

bool
FBO::
checkFramebufferStatus()
{
  GLenum status;
  status = ( GLenum ) glCheckFramebufferStatus( GL_FRAMEBUFFER_EXT );
  //std::cout << "Framebuffer status: " << status << std::endl;
  switch ( status )
  {
    case GL_FRAMEBUFFER_COMPLETE_EXT:
      //std::cout << "Framebuffer ok\n";
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
      std::cout << " Framebuffer incomplete attachment\n";
      break;
    case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
      std::cout << "Unsupported framebuffer format\n";
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
      std::cout << "Framebuffer incomplete, missing attachment\n";
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
      std::cout <<  "Framebuffer incomplete, attached images must have same dimensions\n";
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
      std::cout << "Framebuffer incomplete, attached images must have same format\n";
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
      std::cout << "Framebuffer incomplete, missing draw buffer\n";
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
      std::cout << "Framebuffer incomplete, missing read buffer\n";
      break;
    case 0x8D56: // GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE
      std::cout << "Framebuffer incomplete, attached images must have same multisample count\n";
      break;
    default:
      std::cout << "Unhandled case\n";
      break;
  }

  return ( status == GL_FRAMEBUFFER_COMPLETE_EXT );
}

//-----------------------------------------------------------------------------

GLuint FBO::getAttachment( GLenum _attachment )
{
  return attachments_[_attachment].id;
}

//-----------------------------------------------------------------------------

GLuint FBO::getInternalFormat( GLenum _attachment )
{
  return attachments_[_attachment].internalFormat;
}

//-----------------------------------------------------------------------------

void FBO::resize( GLsizei _width, GLsizei _height, bool _forceResize )
{
  if (_width != width_ ||_height != height_ || _forceResize)
  {
    // resizing already existing textures is highly driver dependent and does not always
    // work for all combinations of texture type (2d, 2dms, 3d) and format
    // safest way to resize is to first delete the FBO and all its internal textures, and then rebuild

    if (fbo_)
      glDeleteFramebuffers(1, &fbo_);
    glGenFramebuffers(1, &fbo_);

    // "detach" all textures
    AttachmentList temp;
    temp.swap(attachments_);

    // reattach all targets
    for (AttachmentList::iterator it = temp.begin(); it != temp.end(); ++it)
    {
      RenderTexture* rt = &it->second;

      // only resize textures that are owned by the FBO
      if (rt->owner)
      {
        glDeleteTextures(1, &rt->id);

        switch (rt->target)
        {
        case GL_TEXTURE_2D: 
#ifdef GL_ARB_texture_multisample
        case GL_TEXTURE_2D_MULTISAMPLE:
#endif
          attachTexture2D(it->first, _width, _height, rt->internalFormat, rt->format, rt->wrapMode, rt->minFilter, rt->magFilter);
          break;

        case GL_TEXTURE_3D:
          attachTexture3D(it->first, _width, _height, rt->dim[2], rt->internalFormat, rt->format, rt->wrapMode, rt->minFilter, rt->magFilter);
          break;

        default:
          std::cout << "FBO::resize - unknown resize target " << rt->target << std::endl;
        }
      }
    }

    // reattach render buffers
    if(depthbuffer_)
      addDepthBuffer(_width, _height);

    if(stencilbuffer_)
      addStencilBuffer(_width, _height);
  }
}

GLuint FBO::getFboID()
{
  return fbo_;
}

GLsizei FBO::setMultisampling( GLsizei _samples, GLboolean _fixedsamplelocations /*= GL_TRUE*/ )
{
  // recreate textures when params changed
  bool recreateTextures = fixedsamplelocation_ != _fixedsamplelocations;

  if (samples_ != _samples)
  {
    // clamp sample count to max supported
    static GLint maxSamples = -1;
    if (maxSamples < 0)
      glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);

    if (_samples >= maxSamples) _samples = maxSamples - 1;

    // gpu driver might cause crash on calling glTexImage2DMultisample if _samples is not a power of 2
    // -> avoid by seeking to next native MSAA sample-count

    if (_samples)
    {
      int safeSampleCount = 1;

      while (safeSampleCount < _samples)
        safeSampleCount <<= 1;

      while (safeSampleCount >= maxSamples)
        safeSampleCount >>= 1;

      _samples = safeSampleCount;
    }

    recreateTextures = recreateTextures || (samples_ != _samples);

    samples_ = _samples;
  }

  fixedsamplelocation_ = _fixedsamplelocations;

  // force texture reloading to apply new multisampling
  if (recreateTextures)
    resize(width_, height_, true);
  
  return samples_;
}


//=============================================================================
} // namespace ACG
//=============================================================================

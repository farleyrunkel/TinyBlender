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
//  CLASS PBuffer - IMPLEMENTATION
//
//============================================================================
#ifdef ARCH_LINUX
//=============================================================================


//== INCLUDES =================================================================

#include "PBuffer.hh"
#include <iostream>
#include <cstdlib>


//== IMPLEMENTATION ==========================================================

PBuffer::PBuffer(int _bits)
{
  int n;

  sbAttrib_.clear();
  sbAttrib_.push_back(GLX_DOUBLEBUFFER);  sbAttrib_.push_back(true);
  sbAttrib_.push_back(GLX_RED_SIZE);      sbAttrib_.push_back(_bits);
  sbAttrib_.push_back(GLX_GREEN_SIZE);    sbAttrib_.push_back(_bits);
  sbAttrib_.push_back(GLX_BLUE_SIZE);     sbAttrib_.push_back(_bits);
  sbAttrib_.push_back(GLX_ALPHA_SIZE);    sbAttrib_.push_back(_bits);
  sbAttrib_.push_back(GLX_DEPTH_SIZE);    sbAttrib_.push_back(24);
  sbAttrib_.push_back(GLX_RENDER_TYPE);   sbAttrib_.push_back(GLX_RGBA_BIT);
  sbAttrib_.push_back(GLX_DRAWABLE_TYPE); sbAttrib_.push_back(GLX_PBUFFER_BIT);
  sbAttrib_.push_back(None);


  pbAttrib_.clear();
  pbAttrib_.push_back(GLX_PBUFFER_WIDTH);       pbAttrib_.push_back(100);
  pbAttrib_.push_back(GLX_PBUFFER_HEIGHT);      pbAttrib_.push_back(100);
  pbAttrib_.push_back(GLX_PRESERVED_CONTENTS);  pbAttrib_.push_back(true);
  pbAttrib_.push_back(None);


  // Create the pbuffer
  dpy_      = glXGetCurrentDisplay();
  currctx_  = glXGetCurrentContext();
  currdraw_ = glXGetCurrentDrawable();

  fbc_ = glXChooseFBConfig(dpy_, DefaultScreen(dpy_), &sbAttrib_[0], &n);
  if (!fbc_) 
  {
    std::cerr << "glXChooseFBConfig failed.\n";
    return;
  }

  pbuf_ = glXCreatePbuffer(dpy_, fbc_[0], &pbAttrib_[0]);
  if (!pbuf_) 
  {
    std::cerr << "glXCreatePbuffer failed.\n";
    return;
  }

  pbufctx_ = glXCreateNewContext(dpy_, fbc_[0], GLX_RGBA_TYPE, currctx_, true);
  if (!pbufctx_) 
  {
    std::cerr << "glXCreateNewContextWithConfigSGIX failed.\n";
    return;
  }
}


//-----------------------------------------------------------------------------


PBuffer::PBuffer(int _w, int _h, int _bits)
{
  int n;

  sbAttrib_.clear();
  sbAttrib_.push_back(GLX_DOUBLEBUFFER);  sbAttrib_.push_back(true);
  sbAttrib_.push_back(GLX_RED_SIZE);      sbAttrib_.push_back(_bits);
  sbAttrib_.push_back(GLX_GREEN_SIZE);    sbAttrib_.push_back(_bits);
  sbAttrib_.push_back(GLX_BLUE_SIZE);     sbAttrib_.push_back(_bits);
  sbAttrib_.push_back(GLX_ALPHA_SIZE);    sbAttrib_.push_back(_bits);
  sbAttrib_.push_back(GLX_DEPTH_SIZE);    sbAttrib_.push_back(24);
  sbAttrib_.push_back(GLX_RENDER_TYPE);   sbAttrib_.push_back(GLX_RGBA_BIT);
  sbAttrib_.push_back(GLX_DRAWABLE_TYPE); sbAttrib_.push_back(GLX_PBUFFER_BIT);
  sbAttrib_.push_back(None);

  pbAttrib_.clear();
  pbAttrib_.push_back(GLX_PBUFFER_WIDTH);       pbAttrib_.push_back(_w);
  pbAttrib_.push_back(GLX_PBUFFER_HEIGHT);      pbAttrib_.push_back(_h);
  pbAttrib_.push_back(GLX_PRESERVED_CONTENTS);  pbAttrib_.push_back(true);
  pbAttrib_.push_back(None);


  // Create the pbuffer
  dpy_      = glXGetCurrentDisplay();
  currctx_  = glXGetCurrentContext();
  currdraw_ = glXGetCurrentDrawable();

  fbc_ = glXChooseFBConfig(dpy_, DefaultScreen(dpy_), &sbAttrib_[0], &n);
  if (!fbc_) 
  {
    std::cerr << "glXChooseFBConfig failed.\n";
    return;
  }

  pbuf_ = glXCreatePbuffer(dpy_, fbc_[0], &pbAttrib_[0]);
  if (!pbuf_) 
  {
    std::cerr << "glXCreatePbuffer failed.\n";
    return;
  }

  pbufctx_ = glXCreateNewContext(dpy_, fbc_[0], GLX_RGBA_TYPE, currctx_, true);
  if (!pbufctx_) 
  {
    std::cerr << "glXCreateNewContextWithConfigSGIX failed.\n";
    return;
  }
}


//-----------------------------------------------------------------------------


PBuffer::~PBuffer()
{
  glXDestroyContext(dpy_, currctx_);
  glXDestroyPbuffer(dpy_, pbuf_);
}


//-----------------------------------------------------------------------------


void PBuffer::resize(int _w, int _h)
{
  pbAttrib_[1] = _w;
  pbAttrib_[3] = _h;
  glXDestroyPbuffer(dpy_, pbuf_);
  pbuf_ = glXCreatePbuffer(dpy_, fbc_[0], &pbAttrib_[0]);
  if (!pbuf_) std::cerr << "Resizing pbuffer failed.\n";
}


//-----------------------------------------------------------------------------


int PBuffer::bits()
{
  return sbAttrib_[3];
}


//-----------------------------------------------------------------------------


void PBuffer::activate()
{
  if (!glXMakeCurrent(dpy_, pbuf_, pbufctx_))
    std::cerr << "PBuffer:activate() failed.\n";
}


//-----------------------------------------------------------------------------


void PBuffer::deactivate()
{
  if (!glXMakeCurrent(dpy_, currdraw_, currctx_))
    std::cerr << "PBuffer:deactivate() failed.\n";
}


//-----------------------------------------------------------------------------


const int PBuffer::width() const
{
  return pbAttrib_[1];
}


//-----------------------------------------------------------------------------


const int PBuffer::height() const
{
  return pbAttrib_[3];
}


//=============================================================================
#endif // Linx only
//=============================================================================

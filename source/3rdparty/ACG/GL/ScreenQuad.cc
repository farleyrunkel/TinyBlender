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
//  CLASS ScreenQuad - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include <ACG/GL/acg_glew.hh>
#include "ScreenQuad.hh"
#include <ACG/ShaderUtils/GLSLShader.hh>
#include <ACG/GL/GLError.hh>
#include <ACG/GL/globjects.hh>

//== NAMESPACES ===============================================================


namespace ACG {


//== IMPLEMENTATION ========================================================== 

ScreenQuad::ScreenQuad () :
  vbo_(0),
  decl_(0),
  texDrawProg_(0)
{
}

//----------------------------------------------------------------------------

ScreenQuad::~ScreenQuad ()
{
  if (vbo_)
    glDeleteBuffers(1, &vbo_);

  delete decl_;
  delete texDrawProg_;
}

//----------------------------------------------------------------------------

ScreenQuad& ScreenQuad::instance ()
{
  static ScreenQuad singleton;
  return singleton;
}

//----------------------------------------------------------------------------

void ScreenQuad::init ()
{
  if (!decl_)
  {
    decl_ = new VertexDeclaration();
    decl_->addElement(GL_FLOAT, 3, ACG::VERTEX_USAGE_POSITION);
  }

  if (!vbo_)
  {
    float quad[] = 
    {
      -1.0f,  1.0f, -1.0f,
      -1.0f, -1.0f, -1.0f, 
      1.0f,  1.0f, -1.0f,
      1.0f, -1.0f, -1.0f
    };

    glGenBuffers(1, &vbo_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    ACG::glCheckErrors();
  }


  if (!texDrawProg_)
  {
    // save active program
    GLint curProg = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &curProg);

    texDrawProg_ = GLSL::loadProgram("ScreenQuad/screenquad.glsl", "ScreenQuad/tex2D.glsl");

    // restore active program
    glUseProgram(curProg);
  }
}

//----------------------------------------------------------------------------

void ScreenQuad::draw (GLSL::Program* _prog)
{
  if (_prog)
    _prog->use();

  ScreenQuad& quad = instance();

  quad.intDraw(_prog);
}

//----------------------------------------------------------------------------

void ScreenQuad::drawInstanced( int _count, GLSL::Program* _prog /*= 0*/ )
{
  if (_prog)
    _prog->use();

  ScreenQuad& quad = instance();

  quad.intDraw(_prog, _count);
}

//----------------------------------------------------------------------------

void ScreenQuad::intDraw (GLSL::Program* _prog, int _numInstances)
{
  if (!vbo_)
  {
    init();
  }


  glBindBuffer(GL_ARRAY_BUFFER, vbo_);

  if (_prog)
    decl_->activateShaderPipeline(_prog);
  else
    decl_->activateFixedFunction();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  if (_numInstances < 1)
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  else
  {
#ifdef GL_VERSION_3_1
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, _numInstances);
#else
    std::cerr << "error: instanced ScreenQuad draw - outdated glew version" << std::endl;
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
#endif
  }

  if (_prog)
    decl_->deactivateShaderPipeline(_prog);
  else
    decl_->deactivateFixedFunction();
}

void ScreenQuad::drawTexture2D( GLuint _texture, const Vec2f& _offset /*= Vec2f(0.0f, 0.0f)*/, const Vec2f& _size /*= Vec2f(1.0f, 1.0f)*/ )
{
  ScreenQuad& quad = instance();

  if (!quad.texDrawProg_)
    quad.init();

  if (quad.texDrawProg_)
  {
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, _texture);


    quad.texDrawProg_->use();

    quad.texDrawProg_->setUniform("Tex", 0); // texslot 0

    quad.texDrawProg_->setUniform("offset", _offset);
    quad.texDrawProg_->setUniform("size", _size);


    quad.intDraw(quad.texDrawProg_);


    quad.texDrawProg_->disable();
  }
}

//----------------------------------------------------------------------------

//=============================================================================
} // namespace ACG
//=============================================================================


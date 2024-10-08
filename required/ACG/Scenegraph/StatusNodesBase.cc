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
#include "StatusNodesT.hh"

//== INCLUDES =================================================================

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================

StatusNodesBase::StatusNodesBase() : heVBO_(0),
    eIBO_(0),
    fIBO_(0),
    vIBO_(0),
    pIBO_(0)
{

}

void StatusNodesBase::createHEVBO()
{
  if(!heVBO_)
  {
    glGenBuffers(1, &heVBO_);
  }
}


void StatusNodesBase::createIBO(GLuint& _name)
{
  if(!_name)
  {
    glGenBuffers(1, &_name);
  }
}

StatusNodesBase::~StatusNodesBase()
{
  if(heVBO_)
    glDeleteBuffers(1,&heVBO_);
  if(eIBO_)
    glDeleteBuffers(1,&eIBO_);
  if(fIBO_)
    glDeleteBuffers(1,&fIBO_);
  if(vIBO_)
    glDeleteBuffers(1,&vIBO_);
  if(pIBO_)
    glDeleteBuffers(1,&pIBO_);
}

void StatusNodesBase::bindHEVBO()
{
  glGetIntegerv(GL_ARRAY_BUFFER_BINDING,&prevBuffer);
  if(!heVBO_)
    createHEVBO();
  glBindBuffer(GL_ARRAY_BUFFER, heVBO_);
}

void StatusNodesBase::bindIBO(GLuint& _name)
{
  glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING,&prevBuffer);
  if(!_name)
    createIBO(_name);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _name);
}

void StatusNodesBase::unbindHEVBO()
{
  glBindBuffer(GL_ARRAY_BUFFER, prevBuffer);
}

void StatusNodesBase::unbindIBO()
{
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, prevBuffer);
}

void StatusNodesBase::updateHEVBOPoints(size_t numberOfElements_, size_t sizeOfElements_, void* data_)
{
  bindHEVBO();
  glBufferData(GL_ARRAY_BUFFER,numberOfElements_ * sizeOfElements_, data_, GL_STATIC_DRAW);
  unbindHEVBO();
}

void StatusNodesBase::updateIBOData(GLuint& bufferName_, size_t numberOfElements_, size_t sizeOfElements_, void* data_)
{
  bindIBO(bufferName_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,numberOfElements_ * sizeOfElements_, data_, GL_STATIC_DRAW);
  unbindIBO();
}
//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

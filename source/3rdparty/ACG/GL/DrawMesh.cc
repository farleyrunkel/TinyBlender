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
#include "DrawMesh.hh"

namespace ACG {

DrawMeshBase::DrawMeshBase() :
        vbo_(0), ibo_(0),
        numTris_(0), numVerts_(0),
        meshComp_(0),
        lineIBO_(0),
        heVBO_(0),
        indexType_(0),
        pickVertexIBO_(0) {

    vertexDecl_ = new VertexDeclaration;
    vertexDeclEdgeCol_ = new VertexDeclaration;
    vertexDeclHalfedgeCol_ = new VertexDeclaration;
    vertexDeclHalfedgePos_ = new VertexDeclaration;
}

DrawMeshBase::~DrawMeshBase() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (ibo_) glDeleteBuffers(1, &ibo_);
    if (lineIBO_) glDeleteBuffers(1, &lineIBO_);
    if (heVBO_) glDeleteBuffers(1, &heVBO_);

    delete vertexDecl_;
    delete vertexDeclEdgeCol_;
    delete vertexDeclHalfedgeCol_;
    delete vertexDeclHalfedgePos_;

    if (pickVertexIBO_) glDeleteBuffers(1, &pickVertexIBO_);
}

void DrawMeshBase::deleteIbo() {
    if (ibo_)
        glDeleteBuffers(1, &ibo_);
    ibo_ = 0;
}

void DrawMeshBase::bindVbo() {
    if (!vbo_)
      glGenBuffers(1, &vbo_);

    ACG::GLState::bindBufferARB(GL_ARRAY_BUFFER_ARB, vbo_);
}

void DrawMeshBase::bindIbo() {
    if (!ibo_)
      glGenBuffers(1, &ibo_);

    ACG::GLState::bindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, ibo_);
}

void DrawMeshBase::bindLineIbo() {
    if (!lineIBO_)
      glGenBuffers(1, &lineIBO_);

    ACG::GLState::bindBufferARB(GL_ELEMENT_ARRAY_BUFFER, lineIBO_);
}

void DrawMeshBase::bindHEVbo() {
  glGetIntegerv(GL_ARRAY_BUFFER_BINDING,&prevVBO_);
    if (!heVBO_)
      glGenBuffers(1, &heVBO_);

    ACG::GLState::bindBufferARB(GL_ARRAY_BUFFER_ARB, heVBO_);
}

void DrawMeshBase::unbindHEVbo() {
    ACG::GLState::bindBufferARB(GL_ARRAY_BUFFER_ARB, prevVBO_);
}

void DrawMeshBase::bindPickVertexIbo() {
    if (!pickVertexIBO_)
        glGenBuffers(1, &pickVertexIBO_);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB, pickVertexIBO_);
}

void DrawMeshBase::createIndexBuffer() {
        glBufferData(GL_ELEMENT_ARRAY_BUFFER_ARB,
            numTris_ * 3 * sizeof(unsigned int),
            meshComp_->getIndexBuffer(), GL_STATIC_DRAW_ARB);
}

void DrawMeshBase::fillLineBuffer(size_t n_edges, void *data) {
    // 2 or 4 byte indices:
    if (indexType_ == GL_UNSIGNED_SHORT)
      glBufferData(GL_ELEMENT_ARRAY_BUFFER_ARB,
                   n_edges * 2 * sizeof(unsigned short),
                   data, GL_STATIC_DRAW_ARB);
    else
      glBufferData(GL_ELEMENT_ARRAY_BUFFER_ARB,
                   n_edges * 2 * sizeof(unsigned int),
                   data, GL_STATIC_DRAW_ARB);
}

void DrawMeshBase::fillHEVBO(size_t numberOfElements_, size_t sizeOfElements_, void* data_)
{
  bindHEVbo();
  glBufferData(GL_ARRAY_BUFFER,numberOfElements_ * sizeOfElements_, data_, GL_STATIC_DRAW);
  unbindHEVbo();
}

void DrawMeshBase::fillVertexBuffer() {
    if (!vertices_.empty())
      glBufferData(GL_ARRAY_BUFFER_ARB, numVerts_ * vertexDecl_->getVertexStride(), &vertices_[0], GL_STATIC_DRAW_ARB);
}

void DrawMeshBase::fillInvVertexMap(size_t n_vertices, void *data) {
    glBufferData(GL_ELEMENT_ARRAY_BUFFER_ARB, sizeof(int) * n_vertices, data, GL_STATIC_DRAW);
}

} /* namespace ACG */

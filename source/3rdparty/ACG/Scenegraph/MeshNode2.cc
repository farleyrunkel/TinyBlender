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
#include "MeshNode2T.hh"
#include "MeshNode2T_impl.hh"

namespace ACG {
namespace SceneGraph {

MeshNodeBase::MeshNodeBase(BaseNode* _parent, std::string _name) :
    BaseNode(_parent, _name),
    drawMeshBase_(0),
    polyEdgeBuf_(0),
    polyEdgeBufSize_(0),
    polyEdgeBufTex_(0) {
}

void MeshNodeBase::supplyDrawMesh(DrawMeshBase *drawMeshBase) {
    /*
     * We take the luxury of checking these conditions even in release
     * mode as this method is rarely called.
     */
    if (drawMeshBase_)
        throw std::runtime_error("MeshNodeBase::supplyDrawMesh called "
                "more than once.");
    if (!drawMeshBase)
        throw std::runtime_error("MeshNodeBase::supplyDrawMesh called "
                "with NULL parameter.");

    drawMeshBase_ = drawMeshBase;
}

void MeshNodeBase::updatePolyEdgeBuf()
{
if(!ACG::compatibilityProfile())
{
  // drawMeshBase_ must have been supplied.
  assert(drawMeshBase_);

  MeshCompiler * const mc = drawMeshBase_->getMeshCompiler();
  if (mc && !mc->isTriangleMesh())
  {
    // create/update the poly-edge buffer
    if (!polyEdgeBuf_)
      glGenBuffers(1, &polyEdgeBuf_);

    const int nTris = mc->getNumTriangles();

    const int newBufSize = (nTris/2+1);

    if (polyEdgeBufSize_ != newBufSize)
    {
      glBindBuffer(GL_TEXTURE_BUFFER, polyEdgeBuf_);

      // The poly-edge buffer is a texture buffer that stores one byte for each triangle, which encodes triangle edge properties.
      // An inner edge is an edge that was added during the triangulation of a n-poly,
      // whereas outer edges are edges that already exist in the input mesh object.
      // This information is used in the wireframe/hiddenline geometry shader to identify edges, which should not be rendered.
      // Buffer storage:
      // each triangle uses 3 bits to mark edges as visible or hidden
      //  outer edge -> bit = 1 (visible)
      //  inner edge -> bit = 0 (hidden)
      // each byte can store edges for two triangles and the remaining 2 bits are left unused

      polyEdgeBufSize_ = newBufSize;
      unsigned char* polyEdgeBufData = new unsigned char[newBufSize];

      // set zero
      memset(polyEdgeBufData, 0, newBufSize);

      // build buffer
      for (int i = 0; i < nTris; ++i)
      {
        int byteidx = i>>1;
        int bitidx = (i&1) * 3;

        for (int k = 0; k < 3; ++k)
          if (mc->isFaceEdge(i, k))
            polyEdgeBufData[byteidx] += 1 << (k + bitidx);
      }

      glBufferData(GL_TEXTURE_BUFFER, polyEdgeBufSize_, polyEdgeBufData, GL_STATIC_DRAW);


      delete [] polyEdgeBufData;
      polyEdgeBufData = 0;

      // create texture object for the texture buffer

      if (!polyEdgeBufTex_)
      {
        glGenTextures(1, &polyEdgeBufTex_);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_BUFFER, polyEdgeBufTex_);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_R8UI, polyEdgeBuf_);

        glBindTexture(GL_TEXTURE_2D, 0);
      }

      ACG::glCheckErrors();
    }
  }
}
}


} /* namespace SceneGraph */
} /* namespace ACG */

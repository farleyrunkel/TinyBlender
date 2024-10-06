/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2016, RWTH-Aachen University                 *
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

#ifndef ACG_SCENEGRAPH_FLOATINGSUBTREENODE_HH_
#define ACG_SCENEGRAPH_FLOATINGSUBTREENODE_HH_

/*
 * This class is available on C++11 compilers only.
 */
#if (defined(_MSC_VER) && (_MSC_VER >= 1900)) || __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)

#include <ACG/Math/GLMatrixT.hh>
#include <ACG/Scenegraph/BaseNode.hh>
#include <ACG/Scenegraph/DrawModes.hh>

namespace ACG {
namespace SceneGraph {

/**
 * Nodes in the subtree rooted at an instance of this class will float over
 * the scene.
 *
 * More precisely, a fixed model view matrix is set for the entire subtree,
 * (i.e. the transformation of the subtree will be independent of the viewport)
 * and the (assuming a modern IRenderer based renderer is used) the overlay
 * flag of all render objects emitted by the subtree is set (i.e. any render
 * objects of the subtree will be rendered in a second pass unaffected by
 * the z-buffer of non-overlayed render objects).
 */
class ACGDLLEXPORT FloatingSubtreeNode : public BaseNode {
    public:
        ACG_CLASSNAME(FloatingSubtreeNode);

        FloatingSubtreeNode(GLMatrixd modelview_override,
                BaseNode *_parent = 0,
                const std::string &_name = "<ModelViewOverrideNode>");

        virtual ~FloatingSubtreeNode();

        void setModelViewOverride(GLMatrixd modelview_override);

        const GLMatrixd &modelViewOverride() const {
            return modelview_override_;
        }

        void enableModelViewOverride(bool value) {
            enable_modelview_override_ = value;
        }

        bool isModelViewOverrideEnabled() {
            return enable_modelview_override_;
        }

        void enableOverlay(bool value) {
            enable_overlay_ = value;
        }

        bool isOverlayEnabled() {
            return enable_overlay_;
        }

        void enter(GLState &_state,
                const DrawModes::DrawMode &_drawMode) override;

        void enter(IRenderer* _renderer, GLState& _state,
                const DrawModes::DrawMode& _drawMode) override;

        void leave(GLState &_state,
                const DrawModes::DrawMode &_drawMode) override;

        void leave(IRenderer* _renderer, GLState& _state,
                const DrawModes::DrawMode& _drawMode) override;

        void enterPick(GLState &_state, PickTarget _target,
                const DrawModes::DrawMode &_drawMode) override;

        void leavePick(GLState &_state, PickTarget _target,
                const DrawModes::DrawMode &_drawMode) override;

        void boundingBox(Vec3d &_bbMin, Vec3d &_bbMax) override;

    private:
        GLMatrixd modelview_override_, modelview_override_inv_;
        bool enable_modelview_override_, enable_overlay_;
};

} /* namespace Scenegraph */
} /* namespace ACG */

#endif /* C++11 */

#endif /* ACG_SCENEGRAPH_FLOATINGSUBTREENODE_HH_ */

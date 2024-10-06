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

/*
 * This class is available on C++11 compilers only.
 */
#if (defined(_MSC_VER) && (_MSC_VER >= 1900)) || __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)

#include "../../ACG/Scenegraph/FloatingSubtreeNode.hh"

#include <ACG/GL/IRenderer.hh>

namespace ACG {
namespace SceneGraph {

FloatingSubtreeNode::FloatingSubtreeNode(
        GLMatrixd modelview_override,
        BaseNode *_parent, const std::string &_name)

    : BaseNode(_parent, _name),
      modelview_override_(std::move(modelview_override)),
      enable_modelview_override_(true),
      enable_overlay_(true) {

    modelview_override_inv_ = modelview_override_;
    modelview_override_inv_.invert();
}

FloatingSubtreeNode::~FloatingSubtreeNode() {

}

void FloatingSubtreeNode::enter(GLState &_state,
        const DrawModes::DrawMode &_drawMode) {
    if (enable_modelview_override_) {
        _state.push_modelview_matrix();
        _state.set_modelview(modelview_override_, modelview_override_inv_);
    }
}

void FloatingSubtreeNode::enter(IRenderer* _renderer, GLState& _state,
        const DrawModes::DrawMode& _drawMode) {
    FloatingSubtreeNode::enter(_state, _drawMode);
}

void FloatingSubtreeNode::leave(GLState &_state,
        const DrawModes::DrawMode &_drawMode) {
    if (enable_modelview_override_) {
        _state.pop_modelview_matrix();
    }
}

void FloatingSubtreeNode::leave(IRenderer* _renderer, GLState& _state,
        const DrawModes::DrawMode& _drawMode) {
    FloatingSubtreeNode::leave(_state, _drawMode);
    if (enable_overlay_) {
        for (auto &obj_it : _renderer->getCollectedSubtreeObjects()) {
            obj_it.overlay = true;
        }
    }
}


void FloatingSubtreeNode::enterPick(GLState &_state, PickTarget _target,
        const DrawModes::DrawMode &_drawMode) {
    FloatingSubtreeNode::enter(_state, _drawMode);
}

void FloatingSubtreeNode::leavePick(GLState &_state, PickTarget _target,
        const DrawModes::DrawMode &_drawMode) {
    FloatingSubtreeNode::leave(_state, _drawMode);
}

void FloatingSubtreeNode::setModelViewOverride(GLMatrixd modelview_override) {
   modelview_override_ = std::move(modelview_override);
   modelview_override_inv_ = modelview_override_;
   modelview_override_inv_.invert();
}

void FloatingSubtreeNode::boundingBox(Vec3d &_bbMin, Vec3d &_bbMax) {
    if (enable_modelview_override_) {
        if (status() != BaseNode::HideChildren) {
            BoundingBoxAction action;
            BaseNode::ChildIter cIt, cEnd(childrenEnd());

            // Process all children which are not second pass
            for (cIt = childrenBegin(); cIt != cEnd; ++cIt)
                if (~(*cIt)->traverseMode() & BaseNode::SecondPass)
                    traverse(*cIt, action);

            // Process all children which are second pass
            for (cIt = childrenBegin(); cIt != cEnd; ++cIt)
                if ((*cIt)->traverseMode() & BaseNode::SecondPass)
                    traverse(*cIt, action);

            // Transform bounding box to view space
            Vec3d minVS = modelview_override_.transform_point(action.bbMin());
            Vec3d maxVS = modelview_override_.transform_point(action.bbMax());

            _bbMin.minimize(minVS);
            _bbMax.maximize(maxVS);
        }
    }
}

} /* namespace Scenegraph */
} /* namespace ACG */

#endif /* C++11 */

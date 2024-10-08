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


#ifndef RESOURCEMANAGERNODE_HH_
#define RESOURCEMANAGERNODE_HH_

#include "BaseNode.hh"

namespace ACG {
namespace SceneGraph {

/**
 * The sole purpose of this node is to guarantee deletion
 * of a resource the lifecycle of which should be bound to
 * the parent node.
 *
 * A use case would be a MeshNodeT<TriMesh> which displays
 * a TriMesh that is not needed anywhere else. Add a
 * PtrResourceManagerNodeT encapsulating that mesh as a child
 * node to the MeshNode.
 */
template<typename T>
class PtrResourceManagerNodeT : public ACG::SceneGraph::BaseNode {
    ACG_CLASSNAME(PtrResourceManagerNodeT);

    public:
        PtrResourceManagerNodeT(
                T* resource, BaseNode* _parent=0,
                std::string _name="<unknown>")
            : BaseNode(_parent, _name), resource(resource) {}

        PtrResourceManagerNodeT(
                BaseNode* _parent=0, std::string _name="<unknown>")
            : BaseNode(_parent, _name), resource(0) {}

        ~PtrResourceManagerNodeT() {
            delete resource;
        }

        /**
         * Deletes the currently managed resource (if any)
         * and takes ownership of the newly supplied one (if any).
         */
        void reset(T* new_resource = 0) {
            delete resource;
            resource = new_resource;
        }

    private:
        T* resource;
};

/**
 * This class does the same as PtrResourceManagerNodeT but uses
 * delete[] instead of delete.
 */
template<typename T>
class ArrayResourceManagerNodeT : public ACG::SceneGraph::BaseNode {
    ACG_CLASSNAME(ArrayResourceManagerNodeT);

    public:
        ArrayResourceManagerNodeT(
                T* resource, BaseNode* _parent=0,
                std::string _name="<unknown>")
            : BaseNode(_parent, _name), resource(resource) {}

        ArrayResourceManagerNodeT(
                BaseNode* _parent=0, std::string _name="<unknown>")
            : BaseNode(_parent, _name), resource(0) {}

        ~ArrayResourceManagerNodeT() {
            delete[] resource;
        }

        /**
         * Deletes the currently managed resource (if any)
         * and takes ownership of the newly supplied one (if any).
         */
        void reset(T* new_resource = 0) {
            delete[] resource;
            resource = new_resource;
        }

    private:
        T* resource;
};

} /* namespace SceneGraph */
} /* namespace ACG */
#endif /* RESOURCEMANAGERNODE_HH_ */

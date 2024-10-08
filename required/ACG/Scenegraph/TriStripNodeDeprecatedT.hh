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
//  CLASS TriStripNodeDeprecatedT
//
//=============================================================================


#ifndef ACG_TRISTRIPNODET_HH
#define ACG_TRISTRIPNODET_HH


//== INCLUDES =================================================================


#include <OpenMesh/Tools/Utils/StripifierT.hh>
#include "MeshNodeDeprecatedT.hh"


//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================



/** \class TriStripNodeDeprecatedT TriStripNodeDeprecatedT.hh <ACG/Scenegraph/TriStripNodeDeprecatedT.hh>

    This node draws a mesh using triangle strips.
*/

template <class Mesh>
class TriStripNodeDeprecatedT : public MeshNodeDeprecatedT<Mesh>
{
public:

  typedef OpenMesh::StripifierT<Mesh>  MyStripifier;
  typedef MeshNodeDeprecatedT<Mesh>              Base;
  typedef typename Base::FaceMode      FaceMode;


  /// Default constructor
  TriStripNodeDeprecatedT(Mesh&        _mesh,
		BaseNode*    _parent=0,
		std::string  _name="<TriStripNodeDeprecatedT>" )
    : Base(_mesh, _parent, _name),
      strips_(_mesh)
  {}


  /// Destructor
  ~TriStripNodeDeprecatedT() {}


  ACG_CLASSNAME(TriStripNodeDeprecatedT);


  /// build triangle strips, delete face indices
  void update_strips()
  {
    std::vector<unsigned int>().swap(Base::indices_);
    strips_.stripify();
  }


  /// build face indices, delete strips
  virtual void update_topology()
  {
    strips_.clear();
    Base::update_topology();
  }



private:

  virtual void draw_faces(FaceMode _mode)
  {
    if (Base::mesh_.is_trimesh()  &&
	     _mode == Base::PER_VERTEX &&
	     strips_.is_valid())
    {
      typename MyStripifier::StripsIterator strip_it   = strips_.begin();
      typename MyStripifier::StripsIterator strip_last = strips_.end();

      for (; strip_it!=strip_last; ++strip_it)
	          glDrawElements(GL_TRIANGLE_STRIP,
		       strip_it->size(),
		       GL_UNSIGNED_INT,
		       &(*strip_it)[0] );
    }
    else Base::draw_faces(_mode);
  }


  MyStripifier  strips_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_TRISTRIPNODET_HH defined
//=============================================================================


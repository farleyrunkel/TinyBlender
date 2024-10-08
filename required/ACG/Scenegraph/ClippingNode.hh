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
//  CLASS ClippingNode
//
//=============================================================================


#ifndef ACG_CLIPPING_NODE_HH
#define ACG_CLIPPING_NODE_HH


//== INCLUDES =================================================================

#include "BaseNode.hh"

#include <ACG/GL/RenderObject.hh>
#include <string>
#include <fstream>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {
  

//== CLASS DEFINITION =========================================================

  
/** \class ClippingNode ClippingNode.hh <ACG/Scenegraph/ClippingNode.hh>

    Set material and some other stuff for this node and all its
    children.  All changes will be done in the enter() method undone
    in the leave() method.
**/

class ACGDLLEXPORT ClippingNode : public BaseNode
{
public:

  /// Default constructor. Applies all properties.
  ClippingNode( BaseNode*           _parent = 0,
		const std::string&  _name = "<ClippingNode>" )
    : BaseNode(_parent, _name),
      slice_width_(0),
      offset_(0),
      mod_(this)
  {
    plane0_[0] = 0.0;
    plane0_[1] = 0.0;
    plane0_[2] = 0.0;
    plane0_[3] = 0.0;

    plane1_[0] = 0.0;
    plane1_[1] = 0.0;
    plane1_[2] = 0.0;
    plane1_[3] = 0.0;

    offset_plane0_[0] = 0.0;
    offset_plane0_[1] = 0.0;
    offset_plane0_[2] = 0.0;
    offset_plane0_[3] = 0.0;

    offset_plane1_[0] = 0.0;
    offset_plane1_[1] = 0.0;
    offset_plane1_[2] = 0.0;
    offset_plane1_[3] = 0.0;
  }


  /// Destructor.
  virtual ~ClippingNode() {}

  /// set class name
  ACG_CLASSNAME(ClippingNode);

  /// enable clipping plane
  void enter(IRenderer* _renderer, GLState& _state, const DrawModes::DrawMode& _drawmode) override;

  /// disable clipping plane
  void leave(IRenderer* _renderer, GLState& _state, const DrawModes::DrawMode& _drawmode) override;

  /// enable clipping plane
  void enter(GLState& _state, const DrawModes::DrawMode& _drawmode) override;

  /// disable clipping plane
  void leave(GLState& _state, const DrawModes::DrawMode& _drawmode) override;

  /// set position and normal of plane
  void set_plane(const Vec3f& _position, const Vec3f& _normal, float _eps=0.0);

  /// get position
  const Vec3f& position() const { return position_; }

  /// get normal
  const Vec3f& normal() const { return normal_; }

  /// get slice width
  float slice_width() const { return slice_width_; }

  /// sweep plane along normal by _dist
  void set_offset(float _dist);

  /// get first plane equation 
  Vec4d plane0() const { return Vec4d(offset_plane0_[0], offset_plane0_[1], offset_plane0_[2], offset_plane0_[3]); }

  /// get second plane equation 
  Vec4d plane1() const { return Vec4d(offset_plane1_[0], offset_plane1_[1], offset_plane1_[2], offset_plane1_[3]); }
  
private:

  Vec3f     position_, normal_;
  GLdouble  plane0_[4], plane1_[4], offset_plane0_[4], offset_plane1_[4];
  float     slice_width_, offset_;

  // modifiers for RenderObject based rendering

  class ACGDLLEXPORT ClippingShaderModifier : public ShaderModifier
  {
  public:
    // this modifier computes the vertex distance to the clip planes and writes them to the gl_ClipDistancei outputs
    // it adds new uniforms:
    //  vec4 g_SlicePlane0;
    //  vec4 g_SlicePlane1;
    //   ..

    explicit ClippingShaderModifier(int _numClipPlanes);
    virtual ~ClippingShaderModifier() {}

    void modifyVertexIO(ShaderGenerator* _shader);

    void modifyVertexEndCode(QStringList* _code);

  private:

    int numClipPlanes_;
  };

  class ACGDLLEXPORT ClippingObjectModifier : public RenderObjectModifier
  {
  public:
    explicit ClippingObjectModifier(const ClippingNode* _node);
    virtual ~ClippingObjectModifier() {}

    void apply(RenderObject* _obj);

  private:
    const ClippingNode* node_;
  };


  ClippingObjectModifier mod_;

  // shader mod for 1 clipping plane
  static ClippingShaderModifier shaderMod1_;

  // shader mod for 2 clipping planes
  static ClippingShaderModifier shaderMod2_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_CLIPPING_NODE_HH defined
//=============================================================================

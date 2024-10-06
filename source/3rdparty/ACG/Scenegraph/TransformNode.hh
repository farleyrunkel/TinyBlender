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
//  CLASS TransformNode
//
//=============================================================================

#ifndef ACG_TRANSFORM_NODE_HH
#define ACG_TRANSFORM_NODE_HH


//== INCLUDES =================================================================


#include "BaseNode.hh"
#include "../Math/GLMatrixT.hh"
#include "../Math/VectorT.hh"
#include "../Math/QuaternionT.hh"


//== NAMESPACES  ==============================================================


namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================


/** \class TransformNode TransformNode.hh <ACG/Scenegraph/TransformNode.hh>

    TransformNode - general 3D geometric transformation node.
    Provides functionality to translate, scale or rotate around a
    specified center. This is the base class for e.g. the
    TrackballNode and the ManipulatorNode, that only add a GUI for
    generating the transformations.

    Note that in order for the transformations to apply in
    each traversal of the scenegraph, one has to call
    apply_transformation(true) once.
*/

class ACGDLLEXPORT TransformNode : public BaseNode
{
public:


  /// Constructor
  TransformNode( BaseNode* _parent=0,
		 const std::string& _name="<TransformNode>" );

  /// Destructor.
  virtual ~TransformNode(){}

  /// set name
  ACG_CLASSNAME(TransformNode);


  /// set current GL-color and GL-material
  void enter(GLState& _state, const DrawModes::DrawMode& _drawmode) override;
  /// restores original GL-color and GL-material
  void leave(GLState& _state, const DrawModes::DrawMode& _drawmode) override;



  /// set center
  void set_center(const Vec3d& _c) { center_ = _c; }
  /// get center
  const Vec3d& center() const { return center_; }


  /** Reset transformation to identity, i.e. reset rotation, translating and
      scaling factor. */
  void loadIdentity();
  /** set the current transformation to the identity transformation,
      i.e. change the center() to the transfomed center and loadIdentity().
  */
  virtual void setIdentity();



  //--- set values ---


  /// Add a translation to the current Transformation.
  void translate(const Vec3d& _v);

  /// translation setter
  void setTranslation(const Vec3d& _v);

  /** Add a rotation to the current Transformation.
      Assume angle in degree and axis normalized */
  void rotate(double _angle, const Vec3d& _axis);

  /// rotation setter
  void setRotation(const Quaterniond& rotation);

  /// Add scaling to the current Transformation.
  void scale(double _s) { scale(Vec3d(_s, _s, _s)); }

  /// Add scaling to the current Transformation.
  void scale(const Vec3d& _s);

  /// Add scaling to the current Transformation.
  void scale(const GLMatrixd& _m);


  //--- get values ---


  /// Returns a const reference to the current transformation matrix
  const GLMatrixd& matrix() const { return matrix_; }
  /// return inverse matrix
  const GLMatrixd& inverse_matrix() const { return inverse_matrix_; }

  /// return rotation axis & angle
  void rotation(Vec3d& _axis, double& _angle) const {
    quaternion_.axis_angle(_axis, _angle);
    _angle *= 180.0/M_PI;
  }
  /// return rotation matrix
  const GLMatrixd& rotation() const {
    return rotation_matrix_;
  }
  /// return inverse rotation matrix
  const GLMatrixd& inverse_rotation() const {
    return inverse_rotation_matrix_;
  }


  /// returns ref. to translation vector
  const Vec3d& translation() const { return translation_; }

  /// return scale matrix
  const GLMatrixd& scale() const {
    return scale_matrix_;
  }
  /// return inverse scale matrix
  const GLMatrixd& inverse_scale() const {
    return inverse_scale_matrix_;
  }

  bool apply_transformation() { return applyTransformation_; }

  void apply_transformation(bool _applyTransformation) { applyTransformation_ = _applyTransformation; }


  // ortho 2d mode
  bool is2D(){return is2DObject_;};
  void set2D(bool _2d){is2DObject_ = _2d;};

  bool isPerSkeletonObject(){return isPerSkeletonObject_;};
  void setPerSkeletonObject(bool _is){isPerSkeletonObject_ = _is;};
  void setPerSkeletonModelView(GLMatrixd _is){perSkeletonModelView_ = _is;};

  void ortho2DMode(GLState& _state);
  void perSkeletonMode(GLState& _state);
  void update2DOffset(ACG::Vec2d _offset){offset_ += _offset;};
  void scale2D(double _scale){scaleFactor2D_ = _scale;};
  void setImageDimensions(ACG::Vec2i _dim){imageDimensions_ = _dim;};

private:

  /// update matrix
  void updateMatrix();


  // ELEMENTS
  GLMatrixd         matrix_, inverse_matrix_;
  GLMatrixd         rotation_matrix_, inverse_rotation_matrix_, scale_matrix_, inverse_scale_matrix_;
  Vec3d             center_;
  Vec3d             translation_;
  Quaterniond       quaternion_;


  bool applyTransformation_;

public:
  // ortho 2d mode
  bool is2DObject_;
  bool isPerSkeletonObject_;
  GLMatrixd perSkeletonModelView_;
  double scaleFactor2D_;
  ACG::Vec2i imageDimensions_;
  ACG::Vec2d offset_;

};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_TRANSFORM_NODE_HH
//=============================================================================

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
//  CLASS GlutPrimitiveNode
//
//=============================================================================


#ifndef ACG_GLUT_PRIMITIVE_NODE_HH
#define ACG_GLUT_PRIMITIVE_NODE_HH


//== INCLUDES =================================================================


#include "BaseNode.hh"
#include "DrawModes.hh"
#include <string>

#include <ACG/GL/GLPrimitives.hh>


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================


/** \class GlutPrimitiveNode GlutPrimitiveNode.hh <ACG/Scenegraph/GlutPrimitiveNode.hh>

    This class is able to render all glut primitives (listed in
    GlutPrimitiveType).
**/

class ACGDLLEXPORT GlutPrimitiveNode : public BaseNode
{

public:

  /// Lists all available primivites
  enum GlutPrimitiveType
  {
    CONE=0, 
    CUBE, 
    DODECAHEDRON, 
    ICOSAHEDRON, 
    OCTAHEDRON, 
    SPHERE, 
    TETRAHEDRON, 
    TORUS
  }; 

  
  struct Primitive
  {
    Vec3d position; // position
    Vec3d axis;    // direction / axis vector

    GlutPrimitiveType type;    // glut primitive type
    
    ACG::Vec4f color; // color
    
    // glut primitive resolution
    double       size;
    double       innersize; // size of inner loop for torus, height for cone
    unsigned int slices, stacks;
    
    // Constructor
    explicit Primitive(GlutPrimitiveType _t) :

        // default axis is negative z
        axis(Vec3d(0,0,1)),

        // Set the type
        type(_t),

        // set default resolution
        size(1.0),
        innersize(1.0),
        slices(20),
        stacks(20)
    {
    }

    Primitive(GlutPrimitiveType _t, Vec3d _p, Vec3d _a, ACG::Vec4f _c) :
      position(_p),
      axis(_a),
      type(_t),
      color(_c),

      // set default resolution
      size(1.0),
      innersize(1.0),
      slices(20),
      stacks(20)
    {
      
    }

    // Copy Constructor
    Primitive( const Primitive& _p)
    {
      // use defined = operator
      *this = _p;
    }

    // = operator
    Primitive& operator=(const Primitive& _p)
    {
      type      = _p.type;
      position  = _p.position;
      axis      = _p.axis;
      color     = _p.color;
      size      = _p.size;  
      innersize = _p.innersize;
      slices    = _p.slices;
      stacks    = _p.stacks;

      return *this;
    }
  };
  
  
  GlutPrimitiveNode( BaseNode*                _parent=0,
                     const std::string &      _name="<GlutPrimitive>" );


  GlutPrimitiveNode( GlutPrimitiveType        _type,
                     BaseNode*                _parent=0,
                     const std::string &      _name="<GlutPrimitive>" );


  /// destructor
  virtual ~GlutPrimitiveNode() {
    if(sphere_)
      delete sphere_;
    if(cone_)
      delete cone_;
  }

  /**
   * Adds a primitive and returns its index.
   *
   * @return the index of the new primitive.
   */
  size_t add_primitive(GlutPrimitiveType _type, Vec3d _pos, Vec3d _axis, ACG::Vec4f _color);

  void clear(){primitives_.clear();};
  
  /// set position
  void set_position(const Vec3d& _p, int _idx = 0);
  /// get position
  const Vec3d get_position(int _idx = 0) const;

  /// get a primitive
  Primitive& get_primitive(int _idx){return primitives_[_idx];};
  
  /// set size
  void set_size(double _s, int _idx = 0);
  /// get size
  double get_size(int _idx = 0) const;
  
  ACG_CLASSNAME(GlutPrimitiveNode);

  /// return available draw modes
  DrawModes::DrawMode availableDrawModes() const override;

  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  
  /// drawing the primitive
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;
  void draw_obj(GLState& _state, size_t _idx) const;
  
  /// picking
  void pick(GLState& _state, PickTarget _target) override;
  
  /** \brief Disable internal color processing
  *
  * Disables the internal color processing of the primitives. If disabled,
  * a Materialnodes settings will apply here.
  */
  void setColorInternal(bool _set) { setColor_ = _set; };

  /** \brief Add the objects to the given renderer
   *
   * @param _renderer The renderer which will be used. Add your geometry into this class
   * @param _state    The current GL State when this object is called
   * @param _drawMode The active draw mode
   * @param _mat      Current material
   */
  void getRenderObjects(IRenderer* _renderer, GLState&  _state , const DrawModes::DrawMode&  _drawMode , const Material* _mat) override;

private:
  
  std::vector<Primitive> primitives_;
  
  bool setColor_;
  
  // Sphere rendering
  ACG::GLSphere*   sphere_;
  ACG::GLCone*   cone_;

};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_GLUT_PRIMITIVE_NODE_HH
//=============================================================================

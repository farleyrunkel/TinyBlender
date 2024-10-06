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
//  CLASS PointNode
//
//=============================================================================


#ifndef ACG_COORDSYSNODE_HH
#define ACG_COORDSYSNODE_HH


//== INCLUDES =================================================================

#include "BaseNode.hh"
#include "DrawModes.hh"
#include <ACG/GL/GLPrimitives.hh>
#include <ACG/ShaderUtils/GLSLShader.hh>
#include <vector>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== CLASS DEFINITION =========================================================


/** \class TextNode CoordsysNode.hh <ACG/Scenegraph/CoordsysNode.hh>

    CoordsysNode renders A coordinate system.
   
   TODO: Den Fall mode_ == POSITION implementieren. 25.11.08

**/

class ACGDLLEXPORT CoordsysNode : public BaseNode
{

public:
  
  /// projection mode
  enum ProjectionMode {
    ORTHOGRAPHIC_PROJECTION, //!< orthographic
    PERSPECTIVE_PROJECTION   //!< perspective
  };
  
  enum CoordsysMode
  {
    POSITION,   ///< Draws the Coordsys at the coordsys origin
    SCREENPOS   ///< Draws the Coordsys at the upper right position on the screen
  };

  /** default constructor
   * @param _parent Define the parent Node this node gets attached to
   * @param _name   Name of this Node
   * @param _mode   upper right of the screen or position based
   * @param _projectionMode Draw an orthogonal coordinate system or also enable projection mode
   */
  CoordsysNode(
      BaseNode* _parent = 0,
      std::string    _name = "<TextNode>",
      CoordsysMode   _mode = SCREENPOS,
      ProjectionMode _projectionMode = PERSPECTIVE_PROJECTION);

  /// destructor
  ~CoordsysNode();

  /// static name of this class
  ACG_CLASSNAME(CoordsysNode);

  /// return available draw modes
  ACG::SceneGraph::DrawModes::DrawMode  availableDrawModes() const override;

  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;

  /// draw Coordsys
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;

  /// add renderobjects for shader pipeline renderer
  void getRenderObjects(IRenderer* _renderer, GLState& _state, const DrawModes::DrawMode& _drawMode, const Material* _mat) override;
  
  /// draw Coordsys for object picking
  void pick(GLState& _state, PickTarget _target) override;
	
  /// set mode to either POSITION or SCREENPOS
  void setMode(const CoordsysMode _mode);
	
  /// set mode to either ORTHOGRAPHIC_PROJECTION or PERSPECTIVE_PROJECTION
  void setProjectionMode(const ProjectionMode _mode);
	
  /// set position of the coordsys
  void setPosition(const Vec3f& _pos);
	
  /// get current mode
  CoordsysMode getMode() const;
	
  /// get current projection mode
  ProjectionMode getProjectionMode() const;
	
  private:

    void drawCoordsys(GLState&  _state);
    void drawCoordsys(IRenderer* _renderer, RenderObject* _baseRO);
    void drawCoordsysPick(GLState&  _state, GLSL::Program* _pickShader = 0);
    void clearPickArea(GLState&  _state, bool _draw, GLfloat _depth, GLSL::Program* _pickShader = 0);
    void boundingCircle(std::vector<Vec2f> &_in, Vec2f &_center, float &_radius);

    CoordsysMode mode_;
    ProjectionMode projectionMode_;
	
    Vec3f pos3f_;


    ACG::GLSphere*   sphere_;
    ACG::GLCylinder* cylinder_;
    ACG::GLCone*     cone_;
    ACG::GLDisk*     disk_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_COORDSYSNODE_HH defined

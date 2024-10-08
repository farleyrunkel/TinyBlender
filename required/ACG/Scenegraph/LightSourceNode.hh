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
//  CLASS LightSourceNode
//
//=============================================================================

#ifndef ACG_LIGHTSOURCE_NODE_HH
#define ACG_LIGHTSOURCE_NODE_HH


//== INCLUDES =================================================================

#include "BaseNode.hh"
#include "../GL/gl.hh"
#include <string>
#include <vector>


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {
  

//== CLASS DEFINITION =========================================================

  
/** \class LightSourceNode LightSourceNode.hh <ACG/Scenegraph/LightSourceNode.hh>

    Set LightSources (0 to 7) for this node and all its children.  
    All changes will be done in the enter() method undone
    in the leave() method.
**/





class ACGDLLEXPORT LightSourceNode : public BaseNode
{

  /// Structure to hold options for one LightSource
  struct LightSource
  {
    //default Constructor
    LightSource()
    {
      // set OpenGL defaults
      enabled = false;
      fixedPosition = false;

      ambientColor  = Vec4f(0.1f,0.1f,0.1f,1.0f);
      diffuseColor  = Vec4f(1.0f,1.0f,1.0f,1.0f);
      specularColor = Vec4f(1.0f,1.0f,1.0f,1.0f);
      position      = Vec4f(0.0f,0.0f,1.0f,0.0f);
      realPosition  = Vec4f(0.0f,0.0f,1.0f,0.0f);
      spotDirection = Vec3f(0.0f,0.0f,-1.0f);
      spotExponent  = 0;
      spotCutoff    = 180;

      constantAttenuation  = 1;
      linearAttenuation    = 0;
      quadraticAttenuation = 0;
    }

    bool enabled;
    bool fixedPosition;
    Vec4f ambientColor;
    Vec4f diffuseColor;
    Vec4f specularColor;
    Vec4f position;
    Vec4f realPosition;
    Vec3f spotDirection;
    float spotExponent;
    float spotCutoff;
    float constantAttenuation;
    float linearAttenuation;
    float quadraticAttenuation;
  };


public:

  /// Default constructor. Applies all properties.
  LightSourceNode( BaseNode*           _parent = 0,
		   const std::string&  _name = "<LightSourceNode>");

  /// Destructor.
  virtual ~LightSourceNode() {}

    
  ACG_CLASSNAME(LightSourceNode);

  /// set current Light Sources
  void enter(GLState& _state, const DrawModes::DrawMode& _drawmode) override;
  /// restores original Light Sources
  void leave(GLState& _state, const DrawModes::DrawMode& _drawmode) override;

  /// enable LightSource _nr
  void enable(GLenum _nr)
  { lights_[gl2index(_nr)].enabled = true; }

  /// disable LightSource _nr
  void disable(GLenum _nr)
  { lights_[gl2index(_nr)].enabled = false; }

  /// set position ( _pos = 1) or direction ( _pos = 0) of LightSource
  void set_position(GLenum _nr, Vec4f _pos)
  { lights_[gl2index(_nr)].position = _pos; }

  /// set position for Point-LightSource
  void set_position(GLenum _nr, Vec3f _pos)
  { set_position(_nr, Vec4f(_pos[0], _pos[1], _pos[2], 1)); }

  /// set direction for directional LightSource
  void set_direction(GLenum _nr, Vec3f _pos)
  { set_position(_nr, Vec4f(_pos[0], _pos[1], _pos[2], 0)); }

  /// set Ambient color for LightSource _nr
  void set_ambient_color( GLenum _nr, Vec4f _color)
  { lights_[gl2index(_nr)].ambientColor = _color; }

  /// set Diffuse color for LightSource _nr
  void set_diffuse_color( GLenum _nr, Vec4f _color)
  { lights_[gl2index(_nr)].diffuseColor = _color; }

  /// set Specular color for LightSource _nr
  void set_specular_color( GLenum _nr, Vec4f _color)
  { lights_[gl2index(_nr)].specularColor = _color; }

  /// make LightSource fixed or moveable with ModelViewMatrix
  void fixed_position(GLenum _nr, bool _state)
  { lights_[gl2index(_nr)].fixedPosition = _state; }

private:

  /// return index in vector for GL_LIGHT*
  int gl2index( GLenum _nr)
  { return( _nr - GL_LIGHT0); }

  /// return GL_LIGHT* for light _nr
  GLenum index2gl( int _nr)
  { return( _nr + GL_LIGHT0); }

  /// set _light Options in OpenGL for GL_LIGHT#_index
  void set_parameters(GLenum _index, LightSource& _light);

  /// get _light Options in OpenGL for GL_LIGHT#_index
  void get_parameters(GLenum _index, LightSource& _light);

private:

  /// store LightSources of this node
  std::vector<LightSource> lights_;

  /// save old LightSources
  std::vector<LightSource> lightsSave_;
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_LIGHTSOURCE_NODE_HH defined
//=============================================================================


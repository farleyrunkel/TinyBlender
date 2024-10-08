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
//  CLASS LightSourceNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "LightSourceNode.hh"


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

  
//== IMPLEMENTATION ========================================================== 


LightSourceNode::LightSourceNode( BaseNode*            _parent, 
				  const std::string&   _name) 
  : BaseNode(_parent, _name)
{
  lights_.resize(8);
  lightsSave_.resize(8);
  enable(GL_LIGHT0);;
}

    
//----------------------------------------------------------------------------

void LightSourceNode::enter(GLState& _state, const DrawModes::DrawMode& /* _drawmode */ ) 
{
  // save old lights
  for(unsigned int i=0; i<lightsSave_.size(); i++)
  {
    // save only if enabled
    if(glIsEnabled(index2gl(i)))
    {
      lightsSave_[i].enabled = true;

      /// transfer GL-preferences to lightsSave_
      get_parameters(index2gl(i), lightsSave_[i]);
    }
    else lightsSave_[i].enabled = false;
  }

  // set new lights
  for(unsigned int i=0; i<lights_.size(); i++)
  {
    if(lights_[i].enabled)
    {
      // correct Position for fixed Lights
      if(lights_[i].fixedPosition)
	lights_[i].realPosition = 
	  _state.inverse_modelview() * lights_[i].position;
      else lights_[i].realPosition = lights_[i].position;

      ACG::GLState::enable(index2gl(i));
      set_parameters(index2gl(i), lights_[i]);
    }
    else ACG::GLState::disable(index2gl(i));

  }
}


//----------------------------------------------------------------------------


void LightSourceNode::leave(GLState& /* _state */ , const DrawModes::DrawMode& /* _drawmode*/ )
{
  // restore old enabled lights
  for(unsigned int i=0; i<lights_.size(); i++)
  {
    if(lightsSave_[i].enabled)
    {
      ACG::GLState::enable(index2gl(i));
      set_parameters(index2gl(i), lightsSave_[i]);
    }
    else ACG::GLState::disable(index2gl(i));
  }
}

//----------------------------------------------------------------------------

void LightSourceNode::set_parameters(GLenum _index, LightSource& _light)
{

  // set preferences of _light for GL_LIGHT#_index
  glLightfv(_index, GL_AMBIENT,  (GLfloat *)_light.ambientColor.data());
  glLightfv(_index, GL_DIFFUSE,  (GLfloat *)_light.diffuseColor.data());
  glLightfv(_index, GL_SPECULAR,  (GLfloat *)_light.specularColor.data());

  glLightfv(_index, GL_POSITION,  (GLfloat *)_light.realPosition.data());
  glLightfv(_index, GL_SPOT_DIRECTION,  (GLfloat *)_light.spotDirection.data());

  glLightf(_index, GL_SPOT_EXPONENT,  _light.spotExponent);
  glLightf(_index, GL_SPOT_CUTOFF,  _light.spotCutoff);
  glLightf(_index, GL_CONSTANT_ATTENUATION,  _light.constantAttenuation);
  glLightf(_index, GL_LINEAR_ATTENUATION,  _light.linearAttenuation);
  glLightf(_index, GL_QUADRATIC_ATTENUATION,  _light.quadraticAttenuation);
}

//----------------------------------------------------------------------------

void LightSourceNode::get_parameters(GLenum _index, LightSource& _light)
{
  // get preferences of GL_LIGHT#_index and store them in _light
  glGetLightfv(_index, GL_AMBIENT,  (GLfloat *)_light.ambientColor.data());
  glGetLightfv(_index, GL_DIFFUSE,  (GLfloat *)_light.diffuseColor.data());
  glGetLightfv(_index, GL_SPECULAR,  (GLfloat *)_light.specularColor.data());
  glGetLightfv(_index, GL_POSITION,  (GLfloat *)_light.position.data());
  glGetLightfv(_index, GL_SPOT_DIRECTION,  (GLfloat *)_light.spotDirection.data());

  glGetLightfv(_index, GL_SPOT_EXPONENT,  &_light.spotExponent);
  glGetLightfv(_index, GL_SPOT_CUTOFF,  &_light.spotCutoff);
  glGetLightfv(_index, GL_CONSTANT_ATTENUATION,  &_light.constantAttenuation);
  glGetLightfv(_index, GL_LINEAR_ATTENUATION,  &_light.linearAttenuation);
  glGetLightfv(_index, GL_QUADRATIC_ATTENUATION,  &_light.quadraticAttenuation);
}
//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

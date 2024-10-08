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
//  CLASS ShaderNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "ShaderNode.hh"


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================


ShaderNode::ShaderNode( BaseNode*            _parent,
			const std::string&   _name)
  : BaseNode(_parent, _name),
    shaderDir_("")
{
}

ShaderNode::~ShaderNode() {
  for ( std::map<size_t,ShaderInfo>::iterator it = shaders.begin(); it != shaders.end(); ++it) {
    if ( it->second.initialized ) {
      
      if ( it->second.program != 0 )
        delete it->second.program;
      
      if ( it->second.vertexShader != 0 )
        delete it->second.vertexShader;
      
      if ( it->second.fragmentShader != 0 )
        delete it->second.fragmentShader;
    }
  }
}

//----------------------------------------------------------------------------
bool
ShaderNode::
hasShader( DrawModes::DrawMode _drawmode, bool _pick ) {
  
  if ( !_drawmode.isAtomic() ) {
    std::cerr << "hasShader: Error, draw mode not atomic!" << std::endl;
    return false;
  }
  
  std::map<size_t,ShaderInfo>::iterator it;
  
  if ( _pick ) {
    it = pickShaders.find(_drawmode.getIndex());
    if ( it == pickShaders.end() )
      return false;
  } else {
    it = shaders.find(_drawmode.getIndex());
    if ( it == shaders.end() )
      return false;
  }
  
  return it->second.initialized;
}

//----------------------------------------------------------------------------


void
ShaderNode::enter(GLState& /*_state*/, const DrawModes::DrawMode& _drawmode  )
{
  for ( std::map<size_t,ShaderInfo>::iterator it = shaders.begin(); it != shaders.end(); ++it) {
    if ( _drawmode.containsAtomicDrawMode(DrawModes::DrawMode(it->first)) && it->second.initialized ) {
      it->second.program->use();
    }
  }
}

//----------------------------------------------------------------------------

void
ShaderNode::enterPick(GLState& /*_state*/, PickTarget /*_target*/, const DrawModes::DrawMode& _drawmode  )
{
  for ( std::map<size_t,ShaderInfo>::iterator it = pickShaders.begin(); it != pickShaders.end(); ++it) {
    if ( _drawmode.containsAtomicDrawMode(DrawModes::DrawMode(it->first)) && it->second.initialized ) {
      it->second.program->use();
    }
  }  
}

//----------------------------------------------------------------------------
std::string
ShaderNode::vertexShaderName(DrawModes::DrawMode _drawmode, bool _pick) {
  
  if ( !_drawmode.isAtomic() ) {
    std::cerr << "vertexShaderName: Error, draw mode not atomic!" << std::endl;
    return std::string("");
  }
  
  std::map<size_t,ShaderInfo>::iterator it;
  
  if ( _pick ) {
    it = pickShaders.find(_drawmode.getIndex());
    if ( it == pickShaders.end() )
      return  std::string("");
  } else {
    it = shaders.find(_drawmode.getIndex());
    if ( it == shaders.end() )
      return  std::string("");
  }
  
  if ( it->second.initialized )
    return it->second.vertexShaderFile;
  
  return  std::string("");
}

//----------------------------------------------------------------------------
std::string
ShaderNode::fragmentShaderName(DrawModes::DrawMode _drawmode, bool _pick) {
  
  if ( !_drawmode.isAtomic() ) {
    std::cerr << "fragmentShaderName: Error, draw mode not atomic!" << std::endl;
    return std::string("");
  }
  
  std::map<size_t,ShaderInfo>::iterator it;
  
  if ( _pick ) {
    it = pickShaders.find(_drawmode.getIndex());
    if ( it == pickShaders.end() )
      return  std::string("");
  } else {
    it = shaders.find(_drawmode.getIndex());
    if ( it == shaders.end() )
      return  std::string("");
  }
  
  if ( it->second.initialized )
    return it->second.fragmentShaderFile;
  
  return  std::string("");
}


//----------------------------------------------------------------------------


void ShaderNode::leave(GLState& /*_state*/, const DrawModes::DrawMode& _drawmode )
{
  for ( std::map<size_t,ShaderInfo>::iterator it = shaders.begin(); it != shaders.end(); ++it)
    if ( _drawmode.containsAtomicDrawMode(DrawModes::DrawMode(it->first)) && it->second.initialized )
      it->second.program->disable();
}

//----------------------------------------------------------------------------


void ShaderNode::leavePick(GLState& /*_state*/, PickTarget /*_target*/, const DrawModes::DrawMode& _drawmode )
{
  for ( std::map<size_t,ShaderInfo>::iterator it = pickShaders.begin(); it != pickShaders.end(); ++it)
    if ( _drawmode.containsAtomicDrawMode(DrawModes::DrawMode(it->first)) && it->second.initialized )
      it->second.program->disable();
}

//----------------------------------------------------------------------------

/// Get the shader for the given drawMode
GLSL::PtrProgram
ShaderNode::
getShader( DrawModes::DrawMode _drawmode, bool _pick ) {
  
  if ( !_drawmode.isAtomic() ) {
    std::cerr << "getShader: Error, draw mode not atomic!" << std::endl;
    return 0;
  }
  
  std::map<size_t,ShaderInfo>::iterator it;
  
  if ( _pick ) {
    it = pickShaders.find(_drawmode.getIndex());
    
    if ( it == pickShaders.end() )
      return  0;
    
    if ( it->second.initialized )
      return it->second.program;
    else 
      return 0;
    
  } else {
    it = shaders.find(_drawmode.getIndex());
    
    if ( it == shaders.end() )
      return  0;
    
    if ( it->second.initialized )
      return it->second.program;
    else 
      return 0;
  }

  // No shader found for this mode
  return  0;
}

//----------------------------------------------------------------------------

void
ShaderNode::
disableShader (DrawModes::DrawMode _drawmode) {


  if ( !_drawmode.isAtomic() ) {
    std::cerr << "disableShader: Error, draw mode not atomic!" << std::endl;
    return;
  }

  size_t index = _drawmode.getIndex();

  // Cleanup old shaders for this mode, if they exist
  if ( shaders[index].initialized ) {
    if ( shaders[index].program != 0 )
      delete shaders[index].program;

    if ( shaders[index].vertexShader != 0 )
      delete shaders[index].vertexShader;

    if ( shaders[index].fragmentShader != 0 )
      delete shaders[index].fragmentShader;

    shaders[index].initialized    = false;
  }
}

//----------------------------------------------------------------------------

void
ShaderNode::
setShader( DrawModes::DrawMode _drawmode ,
           const std::string&  _vertexShader,
           const std::string&  _fragmentShader,
           std::string _pickVertexShader,
           std::string _pickFragmentShader) {

  if ( !ACG::openGLVersion(2,0) ) {
    std::cerr << "Shaders not supported with OpenGL Version less than 2.0" << std::endl;
    return;
  }

  if ( shaderDir_ == "" ) {
    std::cerr << "No shader dir set for shadernode. Unable to load shaders!" << std::endl;
    return;
  }
  
  if ( !_drawmode.isAtomic() ) {
    std::cerr << "setShader: Error, draw mode not atomic!" << std::endl;
    return;
  }

  disableShader (_drawmode);
  size_t index = _drawmode.getIndex();

  shaders[index].vertexShaderFile   = shaderDir_ + _vertexShader;
  shaders[index].fragmentShaderFile = shaderDir_ + _fragmentShader;

  const char* vertexShaderFilePath   = shaders[index].vertexShaderFile.c_str();
  const char* fragmentShaderFilePath = shaders[index].fragmentShaderFile.c_str();
  shaders[index].vertexShader            = GLSL::loadVertexShader(vertexShaderFilePath);
  shaders[index].fragmentShader          = GLSL::loadFragmentShader(fragmentShaderFilePath);
  shaders[index].program                 = GLSL::PtrProgram(new GLSL::Program());

  if ( (shaders[index].vertexShader == 0) ||
        (shaders[index].fragmentShader == 0) ||
        (shaders[index].program == 0) ) {
    std::cerr << "Unable to load shaders" << shaders[index].vertexShaderFile <<
                  " or " << shaders[index].fragmentShaderFile << std::endl;
    shaders[index].vertexShader   = 0;
    shaders[index].fragmentShader = 0;
    shaders[index].program        = 0;
    shaders[index].initialized    = false;
    return;
  }

  shaders[index].program->attach(shaders[index].vertexShader);
  shaders[index].program->attach(shaders[index].fragmentShader);
  shaders[index].program->link();

  shaders[index].initialized = true;

  
  // Cleanup old shaders for this mode, if they exist
  if ( pickShaders[index].initialized ) {
    if ( pickShaders[index].program != 0 )
        delete pickShaders[index].program;

    if ( pickShaders[index].vertexShader != 0 )
      delete pickShaders[index].vertexShader;

    if ( pickShaders[index].fragmentShader != 0 )
      delete pickShaders[index].fragmentShader;

    pickShaders[index].initialized    = false;
  }

  if (_pickVertexShader.length () > 0 && _pickFragmentShader.length () > 0)
  {
    pickShaders[index].vertexShaderFile   = shaderDir_ + _pickVertexShader;
    pickShaders[index].fragmentShaderFile = shaderDir_ + _pickFragmentShader;

    const char* vertexShaderFilePath2   = pickShaders[index].vertexShaderFile.c_str();
    const char* fragmentShaderFilePath2 = pickShaders[index].fragmentShaderFile.c_str();
    pickShaders[index].vertexShader        = GLSL::loadVertexShader(vertexShaderFilePath2);
    pickShaders[index].fragmentShader      = GLSL::loadFragmentShader(fragmentShaderFilePath2);
    pickShaders[index].program             = GLSL::PtrProgram(new GLSL::Program());

    if ( (pickShaders[index].vertexShader == 0) ||
          (pickShaders[index].fragmentShader == 0) ||
          (pickShaders[index].program == 0) ) {
      std::cerr << "Unable to load pick shaders" << pickShaders[index].vertexShaderFile <<
                    " or " << pickShaders[index].fragmentShaderFile << std::endl;
      pickShaders[index].vertexShader   = 0;
      pickShaders[index].fragmentShader = 0;
      pickShaders[index].program        = 0;
      pickShaders[index].initialized    = false;
      return;
    }

    pickShaders[index].program->attach(pickShaders[index].vertexShader);
    pickShaders[index].program->attach(pickShaders[index].fragmentShader);
    pickShaders[index].program->link();

    pickShaders[index].initialized = true;
  }
}

//----------------------------------------------------------------------------

void
ShaderNode::
setShaderDir( std::string _shaderDir) {
  shaderDir_ = _shaderDir;
}

DrawModes::DrawMode
ShaderNode::
availableDrawModes() const
{
  DrawModes::DrawMode drawModes(DrawModes::NONE);

  for ( std::map<size_t,ShaderInfo>::const_iterator it = shaders.begin(); it != shaders.end(); ++it) {
    // If the shader for this drawmode is initialized, this node supports the given draw mode.
    // Then we add it to the list of supported draw modes
    if ( it->second.initialized) {
      drawModes |= DrawModes::DrawMode(it->first);
    }
  }

  return drawModes;

}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

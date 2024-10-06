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
//  CLASS BaseNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "BaseNode.hh"


//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================


unsigned int BaseNode::last_id_used__ = 0;


//----------------------------------------------------------------------------


BaseNode::
BaseNode(BaseNode* _parent, std::string _name)
  : multipassStatus_(ALLPASSES),
    multipassNode_(PASS_1),
    parent_(_parent),
    name_(_name),
    status_(Active),
    drawMode_(DrawModes::DEFAULT),
    pickingEnabled_(true),
    dirty_ (false),
    traverseMode_ (BaseNode::NodeFirst),
    uniformPool_(0),
    renderModifier_(0)
{
  id_ = ++last_id_used__;
  if (_parent!=0) _parent->push_back(this);

  DrawModes::initializeDefaultDrawModes();
}


//----------------------------------------------------------------------------


BaseNode::
BaseNode(BaseNode* _parent, BaseNode* _child, std::string _name)
  : multipassStatus_(ALLPASSES),
    multipassNode_(PASS_1),
    parent_(_parent),
    name_(_name),
    status_(Active),
    drawMode_(DrawModes::DEFAULT),
    pickingEnabled_(true),
    dirty_ (false),
    traverseMode_ (BaseNode::NodeFirst)
{
  assert(_parent != 0 && _child != 0);

  id_ = ++last_id_used__;

  _parent->push_back(this);
  _child->set_parent(this);

  DrawModes::initializeDefaultDrawModes();
}


//----------------------------------------------------------------------------


BaseNode::~BaseNode()
{
  // remove myself from parent's children
  if (parent_!=0)
  {
    ChildIter me(parent_->find(this));
    assert(me != parent_->childrenEnd());
    parent_->remove(me);
  }


  // remove me (as parent) from my children
  for (BaseNode::ChildIter cIt=childrenBegin(); cIt!=childrenEnd(); ++cIt)
    (*cIt)->parent_ = 0;
}


//----------------------------------------------------------------------------


void
BaseNode::
set_parent(BaseNode* _parent)
{
  if (parent_)
  {
    ChildIter me(parent_->find(this));
    if (me != parent_->childrenEnd())
      parent_->remove(me);
  }

  parent_ = _parent;

  if (parent_)
  {
    ChildIter me(parent_->find(this));
    if (me == parent_->childrenEnd())
      parent_->push_back(this);
  }
}


//----------------------------------------------------------------------------


void
BaseNode::delete_subtree()
{
  while (!children_.empty())
    children_.front()->delete_subtree();
  delete this;
}

//----------------------------------------------------------------------------

void
BaseNode::enterPick(GLState& _state, PickTarget /*_target*/, const DrawModes::DrawMode& _drawMode)
{
  enter (_state, _drawMode);
}

//----------------------------------------------------------------------------

void
BaseNode::leavePick(GLState& _state, PickTarget /*_target*/, const DrawModes::DrawMode& _drawMode)
{
  leave (_state, _drawMode);
}

//----------------------------------------------------------------------------

void BaseNode::multipassStatusSetActive(const unsigned int _i, bool _active) {

  if ( _i == NOPASS ) {
    multipassStatus_ = NOPASS;
  } else if ( _i == ALLPASSES ) {
    if ( _active )
      multipassStatus_ = ALLPASSES;
    else
      multipassStatus_ = NOPASS;  
  } else {
    if ( _active ) 
      multipassStatus_ |=  (1 << (_i == 0 ? 0 : _i - 1));
    else
      multipassStatus_ &=  ~(1 << (_i == 0 ? 0 : _i - 1));
  }
  
}

//----------------------------------------------------------------------------

bool BaseNode::multipassStatusActive(const unsigned int _i) const {

  if ( multipassStatus_ == NOPASS )
    return false;
  else if ( multipassStatus_ & ALLPASSES )
    return true;
  else  
    return ((1 << (_i == 0 ? 0 : _i - 1)) & multipassStatus_) != 0;
  
}

//----------------------------------------------------------------------------

void BaseNode::multipassNodeSetActive(const unsigned int _i , bool _active) {
  
  if ( _i == NOPASS ) {
    multipassNode_ = NOPASS;
  } else if ( _i == ALLPASSES ) {
    if ( _active )
      multipassNode_ = ALLPASSES;
    else
      multipassNode_ = NOPASS;  
  } else {
    if ( _active ) 
      multipassNode_ |=  (1 << (_i == 0 ? 0 : _i - 1));
    else
      multipassNode_ &=  ~(1 << (_i == 0 ? 0 : _i - 1));
  }
  
}

//----------------------------------------------------------------------------

bool BaseNode::multipassNodeActive(const unsigned int _i) const {
  
  if ( multipassNode_ == NOPASS )
    return false;
  else if ( multipassNode_ & ALLPASSES )
    return true;
  else  
    return ((1 << (_i == 0 ? 0 : _i - 1)) & multipassNode_) != 0;
  
}

//----------------------------------------------------------------------------

void BaseNode::setRenderObjectShaders( const std::string& _vertexShaderFile, const std::string& _geometryShaderFile, const std::string& _fragmentShaderFile, bool _relativePaths, ACG::SceneGraph::DrawModes::DrawModePrimitive _primitiveType ) {

  ShaderSet s;
  s.vs_ = _vertexShaderFile;
  s.gs_ = _geometryShaderFile;
  s.fs_ = _fragmentShaderFile;
  s.relativePaths_ = _relativePaths;

  shaderSettings_[_primitiveType] = s;
}

//----------------------------------------------------------------------------

void BaseNode::setRenderObjectShaders( const std::string& _vertexShaderFile, const std::string& _tessControlShaderFile, const std::string& _tessEvalShaderFile, const std::string& _geometryShaderFile, const std::string& _fragmentShaderFile, bool _relativePaths, ACG::SceneGraph::DrawModes::DrawModePrimitive _primitiveType ) {

  ShaderSet s;
  s.vs_ = _vertexShaderFile;
  s.gs_ = _geometryShaderFile;
  s.fs_ = _fragmentShaderFile;
  s.tcs_ = _tessControlShaderFile;
  s.tes_ = _tessEvalShaderFile;
  s.relativePaths_ = _relativePaths;

  shaderSettings_[_primitiveType] = s;
}

//----------------------------------------------------------------------------

void BaseNode::setRenderObjectTexture( int _samplerSlot, GLuint _texId, GLenum _texType ) {
  
  ACG::RenderObject::Texture t;
  t.id = _texId;
  t.type = _texType;
  t.shadow = false;

  textureSettings_[_samplerSlot] = t;
}

//----------------------------------------------------------------------------

void BaseNode::applyRenderObjectSettings( DrawModes::DrawModePrimitive _primitive, RenderObject* _obj ) const {

  // Copy texture settings
  for (std::map<int, RenderObject::Texture>::const_iterator it = textureSettings_.begin(); it != textureSettings_.end(); ++it)
    _obj->addTexture(it->second, size_t(it->first), false);

  // Copy uniforms from provided pool
  if (uniformPool_)
    _obj->addUniformPool(*uniformPool_);


std::map<DrawModes::DrawModePrimitive, ShaderSet>::const_iterator shaderSet = shaderSettings_.find(_primitive);

  bool defaultShaders = shaderSet == shaderSettings_.end();

  if (!defaultShaders) {

    // prepend openflipper shader dir
    if (shaderSet->second.relativePaths_) {
      _obj->shaderDesc.tessControlTemplateFile = 
        _obj->shaderDesc.tessEvaluationTemplateFile = 
        _obj->shaderDesc.vertexTemplateFile = 
        _obj->shaderDesc.geometryTemplateFile =         
        _obj->shaderDesc.fragmentTemplateFile = ShaderProgGenerator::getShaderDir();
    }

    _obj->shaderDesc.vertexTemplateFile += shaderSet->second.vs_.c_str();
    _obj->shaderDesc.tessControlTemplateFile += shaderSet->second.tcs_.c_str();
    _obj->shaderDesc.tessEvaluationTemplateFile += shaderSet->second.tes_.c_str();
    _obj->shaderDesc.geometryTemplateFile += shaderSet->second.gs_.c_str();
    _obj->shaderDesc.fragmentTemplateFile += shaderSet->second.fs_.c_str();
  }

  if (renderModifier_)
    renderModifier_->apply(_obj);
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

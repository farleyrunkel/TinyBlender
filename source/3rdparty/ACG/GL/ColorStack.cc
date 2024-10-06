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
//  CLASS ColorStack - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "ColorStack.hh"
#include <ACG/GL/GLState.hh>
#include <iostream>


//== NAMESPACES ===============================================================


namespace ACG {


//== IMPLEMENTATION ========================================================== 

ColorStack::ColorStack () :
  initialized_(false),
  root_ (0),
  currentNode_ (0),
  error_(false)
{
}

//----------------------------------------------------------------------------

ColorStack::~ColorStack ()
{
  if (root_)
    delete root_;
}

//----------------------------------------------------------------------------

void ColorStack::initialize (GLState* _state)
{
  if (initialized_)
  {
    delete root_;
  }
  error_ = false;
  translator_.initialize (_state);
  root_ = currentNode_ = new ColorStack::Node (0, 0, &translator_);
  initialized_ = true;
}

//----------------------------------------------------------------------------

bool ColorStack::setMaximumIndex (size_t _idx)
{
  if (initialized_)
  {
    bool rv = currentNode_->setMaximumIndex (_idx);
    if (!rv)
      error_ = true;
    return rv;
  }
  return false;
}

//----------------------------------------------------------------------------

void ColorStack::setIndex (size_t _idx)
{
  if (initialized_)
  {
    if (!currentNode_->setIndex (_idx))
      error_ = true;
  }
}

//----------------------------------------------------------------------------

Vec4uc ColorStack::getIndexColor (size_t _idx)
{
  if (initialized_)
  {
    Vec4uc rv;
    if (!currentNode_->getIndexColor (_idx, rv))
      error_ = true;
    else
      return rv;
  }
  
  return Vec4uc (0, 0, 0, 0);
}

//----------------------------------------------------------------------------

void ColorStack::pushIndex (size_t _idx)
{
  if (initialized_)
    currentNode_ = currentNode_->pushIndex (_idx);
}

//----------------------------------------------------------------------------

void ColorStack::popIndex ()
{
  if (initialized_)
    currentNode_ = currentNode_->popIndex ();
}

//----------------------------------------------------------------------------

std::vector<size_t> ColorStack::colorToStack (Vec4uc _rgba) const
{
  std::vector<size_t> rv(0);
  if (initialized_ && !error_)
  {
    const size_t idx = translator_.color2index (_rgba);
    if (idx >= root_->startIndex () && idx < root_->endIndex ())
      root_->colorToStack (rv, idx);
  }
  return rv;
}

//----------------------------------------------------------------------------

size_t ColorStack::freeIndicies() const
{
  if (initialized_)
  {
    return translator_.max_index () - currentNode_->endIndex ();
  }
  else
    return 0; 
}

//----------------------------------------------------------------------------

size_t ColorStack::currentIndex () const
{
  if (initialized_)
  {
    return currentNode_->colorIndex ();
  }
  return 0;
}


//----------------------------------------------------------------------------

ColorStack::Node::Node (size_t _idx, Node *_parent, ColorTranslator *_ct) :
  parent_(_parent),
  index_(_idx),
  translator_(_ct),
  colorStartIdx_(0),
  colorEndIdx_(0)
{
  if (parent_)
    startIdx_ = endIdx_ = parent_->endIndex ();
  else
    startIdx_ = endIdx_ = 1;
}

//----------------------------------------------------------------------------

ColorStack::Node::~Node ()
{
  for (std::vector<Node *>::iterator it = nodes_.begin (); it != nodes_.end(); ++it)
    delete (*it);
}

//----------------------------------------------------------------------------

bool ColorStack::Node::setMaximumIndex (size_t _idx)
{
  if (_idx == 0)
    _idx = 1;

  if (colorStartIdx_ == 0 && translator_->max_index () > endIdx_ + _idx)
  {
    colorStartIdx_ = endIdx_;
    endIdx_ = colorEndIdx_ = colorStartIdx_ + _idx;
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------

bool ColorStack::Node::setIndex (size_t _idx) const
{
  if (colorStartIdx_ && colorStartIdx_ + _idx < colorEndIdx_)
  {
    if(ACG::compatibilityProfile())                               //only change glColor in compat mode as shader pipeline 
      glColor(translator_->index2color(colorStartIdx_ + _idx));   //renderer on core profiles does not rely on glColor
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------

bool ColorStack::Node::getIndexColor (size_t _idx, Vec4uc &_rgba) const
{
  if (colorStartIdx_ && colorStartIdx_ + _idx < colorEndIdx_)
  {
    _rgba = translator_->index2color(colorStartIdx_ + _idx);
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------

ColorStack::Node * ColorStack::Node::pushIndex (size_t _idx)
{
  ColorStack::Node *n = new ColorStack::Node (_idx, this, translator_);
  nodes_.push_back (n);
  return n;
}

//----------------------------------------------------------------------------

ColorStack::Node * ColorStack::Node::popIndex ()
{
  parent_->endIdx_ = endIdx_;
  return parent_;
}

//----------------------------------------------------------------------------

void ColorStack::Node::colorToStack (std::vector<size_t> &_stack, size_t _index)
{
  if (_index >= colorStartIdx_ && _index < colorEndIdx_)
  {
    _stack.push_back (_index - colorStartIdx_);
  }
  else
  {
    for (std::vector<Node *>::iterator it = nodes_.begin (); it != nodes_.end(); ++it)
    {
      ColorStack::Node *n = (*it);
      if (_index >= n->startIndex () && _index < n->endIndex ())
        n->colorToStack (_stack, _index);
    }
  }
  _stack.push_back (index_);
}

//=============================================================================
} // namespace ACG
//=============================================================================


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
//  CLASS QuadNode - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include "QuadNode.hh"


//== NAMESPACES ============================================================== 


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ========================================================== 


QuadNode::QuadNode( BaseNode*          _parent,
                    const std::string& _name  )
  : BaseNode(_parent, _name)
{
}
  

//----------------------------------------------------------------------------


QuadNode::~QuadNode() 
{
}


//----------------------------------------------------------------------------


void
QuadNode::boundingBox( Vec3d & _bbMin, Vec3d & _bbMax )
{
  Vec3f bbMin(NumLimitsT<float>::max(),NumLimitsT<float>::max(),NumLimitsT<float>::max());
  Vec3f bbMax(-NumLimitsT<float>::max(),-NumLimitsT<float>::max(),-NumLimitsT<float>::max());
  
  PointVector::const_iterator p_it  = point_.begin(), p_end = point_.end();
  
  for ( ; p_it != p_end; ++p_it )
  {
    bbMin.minimize( *p_it );
    bbMax.maximize( *p_it );
  }
  
  Vec3d bbMind = ACG::Vec3d(bbMin);
  Vec3d bbMaxd = ACG::Vec3d(bbMax);
  
  _bbMin.minimize(bbMind);
  _bbMax.maximize(bbMaxd);    
}


//----------------------------------------------------------------------------

  
DrawModes::DrawMode
QuadNode::
availableDrawModes() const
{
  DrawModes::DrawMode drawModes(DrawModes::NONE);

  drawModes |= DrawModes::WIREFRAME;
  drawModes |= DrawModes::HIDDENLINE;
  drawModes |= DrawModes::SOLID_FLAT_SHADED;
  //  drawModes |= DrawModes::SOLID_FACES_COLORED;

  return drawModes;
}


//----------------------------------------------------------------------------


void
QuadNode::
draw(GLState& /* _state */ , const DrawModes::DrawMode& _drawMode)
{
  if (_drawMode & DrawModes::WIREFRAME ||
      _drawMode & DrawModes::HIDDENLINE )
  {
    ACG::GLState::disable(GL_LIGHTING);
    ACG::GLState::shadeModel(GL_FLAT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    draw_wireframe();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }


  if (_drawMode & DrawModes::SOLID_FLAT_SHADED ||
      _drawMode & DrawModes::HIDDENLINE )
  {
    ACG::GLState::enable(GL_LIGHTING);
    ACG::GLState::shadeModel(GL_FLAT);
    ACG::GLState::depthRange(0.01, 1.0);
    draw_faces();
    ACG::GLState::depthRange(0.0, 1.0);
  }

  if (_drawMode & DrawModes::SOLID_FACES_COLORED)
  {
    ACG::GLState::disable(GL_LIGHTING);
    ACG::GLState::shadeModel(GL_FLAT);
    ACG::GLState::depthRange(0.01, 1.0);
    draw_faces();
    ACG::GLState::depthRange(0.0, 1.0);
  }
}


//----------------------------------------------------------------------------


void
QuadNode::
draw_vertices()
{
  //  glDrawArrays(GL_POINTS, 0, mesh_.n_vertices());
}


//----------------------------------------------------------------------------


void
QuadNode::draw_faces()
{
  glBegin(GL_QUADS);

  unsigned int i = 0;
  unsigned int j = 0;

  for ( ; i < point_.size(); i += 4, j += 1 )
  {
    glNormal(  normal_[j] );
    
    glVertex( point_[i + 0] );
    glVertex( point_[i + 1] );
    glVertex( point_[i + 2] );
    glVertex( point_[i + 3] );
  }

  glEnd();
}


//----------------------------------------------------------------------------


void
QuadNode::draw_wireframe()
{
  glBegin(GL_QUADS);
  for ( unsigned int i = 0; i < point_.size(); ++i )
    glVertex( point_[i] );
  glEnd();
}


//----------------------------------------------------------------------------


void
QuadNode::pick( GLState & _state, PickTarget /* _target */  )
{
  _state.pick_set_maximum (1);
  _state.pick_set_name (0);
  draw_faces();
}


//----------------------------------------------------------------------------

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

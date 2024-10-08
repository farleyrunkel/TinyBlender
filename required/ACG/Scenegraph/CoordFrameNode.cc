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
//  CLASS CoordFrameNode - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================


#include "CoordFrameNode.hh"
#include "SceneGraph.hh"
#include "../GL/stipple_alpha.hh"

#include <cstdio>


//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ========================================================== 


CoordFrameNode::
CoordFrameNode(BaseNode* _parent, const std::string& _name)
  : MaterialNode(_parent, _name),
    bb_min_(Vec3f(0,0,0)),
    bb_max_(Vec3f(0,0,0))
{  
}


//-----------------------------------------------------------------------------


DrawModes::DrawMode
CoordFrameNode::availableDrawModes() const
{
  return ( DrawModes::WIREFRAME           |
	   DrawModes::SOLID_FLAT_SHADED   );
}


//-----------------------------------------------------------------------------


void 
CoordFrameNode::boundingBox(Vec3d& _bbMin, Vec3d& _bbMax)
{
  _bbMin.minimize(bb_min_);
  _bbMax.maximize(bb_max_);
}
  

//-----------------------------------------------------------------------------


void 
CoordFrameNode::draw(GLState& /* _state */ , const DrawModes::DrawMode& /* _drawMode */ )
{
  // draw bounding box

  ACG::GLState::disable(GL_LIGHTING);
  ACG::GLState::shadeModel(GL_FLAT);

  glBegin(GL_LINE_LOOP);
  glVertex3f(bb_min_[0], bb_min_[1], bb_min_[2]);
  glVertex3f(bb_max_[0], bb_min_[1], bb_min_[2]);
  glVertex3f(bb_max_[0], bb_max_[1], bb_min_[2]);
  glVertex3f(bb_min_[0], bb_max_[1], bb_min_[2]);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(bb_min_[0], bb_min_[1], bb_max_[2]);
  glVertex3f(bb_max_[0], bb_min_[1], bb_max_[2]);
  glVertex3f(bb_max_[0], bb_max_[1], bb_max_[2]);
  glVertex3f(bb_min_[0], bb_max_[1], bb_max_[2]);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(bb_min_[0], bb_min_[1], bb_min_[2]);
  glVertex3f(bb_min_[0], bb_min_[1], bb_max_[2]);
  glVertex3f(bb_max_[0], bb_min_[1], bb_min_[2]);
  glVertex3f(bb_max_[0], bb_min_[1], bb_max_[2]);
  glVertex3f(bb_max_[0], bb_max_[1], bb_min_[2]);
  glVertex3f(bb_max_[0], bb_max_[1], bb_max_[2]);
  glVertex3f(bb_min_[0], bb_max_[1], bb_min_[2]);
  glVertex3f(bb_min_[0], bb_max_[1], bb_max_[2]);
  glEnd();



  // draw planes: transparently filled
  Vec3f v0, v1, v2, v3;


  stipple_alpha(0.25);

  std::vector<float>::const_iterator p_it  = x_planes_.begin();
  std::vector<float>::const_iterator p_end = x_planes_.end();
  char axis  = 'x';

  for (bool finished(false); !finished; )
  {
      // break check
      if (p_it == p_end)
      {
          switch (axis)
          {
          case 'x':
              p_it  = y_planes_.begin();
              p_end = y_planes_.end();
              axis  = 'y';
              break;

          case 'y':
              p_it  = z_planes_.begin();
              p_end = z_planes_.end();
              axis  = 'z';
              break;

          default:
              finished = true;
              break;
          }
          continue;
      }


      switch (axis)
      {
      case 'x':
          v0 = Vec3f(*p_it, bb_min_[1], bb_min_[2]);
          v1 = Vec3f(*p_it, bb_max_[1], bb_min_[2]);
          v2 = Vec3f(*p_it, bb_max_[1], bb_max_[2]);
          v3 = Vec3f(*p_it, bb_min_[1], bb_max_[2]);
          break;

      case 'y':
          v0 = Vec3f(bb_min_[0], *p_it, bb_min_[2]);
          v1 = Vec3f(bb_max_[0], *p_it, bb_min_[2]);
          v2 = Vec3f(bb_max_[0], *p_it, bb_max_[2]);
          v3 = Vec3f(bb_min_[0], *p_it, bb_max_[2]);
          break;

      case 'z':
          v0 = Vec3f(bb_min_[0], bb_min_[1], *p_it);
          v1 = Vec3f(bb_max_[0], bb_min_[1], *p_it);
          v2 = Vec3f(bb_max_[0], bb_max_[1], *p_it);
          v3 = Vec3f(bb_min_[0], bb_max_[1], *p_it);
          break;
      };


      // quads
      glBegin(GL_QUADS);
      glVertex(v0);  glVertex(v1);  glVertex(v2);  glVertex(v3);
      glEnd();


      // outlines
      glBegin(GL_LINE_LOOP);
      glVertex(v0);  glVertex(v1);  glVertex(v2);  glVertex(v3);
      glEnd();


      // text
      //sprintf(s, "%c=%f", axis, *p_it);
      //glText(v0, s);  glText(v1, s);  glText(v2, s);  glText(v3, s);


      ++p_it;
  }


  stipple_alpha(1.0);
}


//-----------------------------------------------------------------------------


void 
CoordFrameNode::update_bounding_box()
{
  BoundingBoxAction bb_action;
  traverse(this, bb_action);

  bb_min_ = bb_action.bbMin();
  bb_max_ = bb_action.bbMax();
}


//-----------------------------------------------------------------------------


void 
CoordFrameNode::set_bounding_box(const Vec3f& _bb_min, const Vec3f& _bb_max)
{
  bb_min_ = _bb_min;
  bb_max_ = _bb_max;
}


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

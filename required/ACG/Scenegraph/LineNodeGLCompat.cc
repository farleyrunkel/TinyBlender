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
//  CLASS LineNode - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================
#include <ACG/GL/acg_glew.hh>
#include "LineNode.hh"
#include <ACG/GL/IRenderer.hh>

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== IMPLEMENTATION ==========================================================

void
LineNode::
drawCompat(GLState&  _state  , const DrawModes::DrawMode& _drawMode)
{
  if (_drawMode & DrawModes::WIREFRAME)
  {
    ACG::GLState::disable(GL_LIGHTING);


   // if (line_mode_ == LineSegmentsMode)
   //   glBegin(GL_LINES);
   // else
   //   glBegin(GL_LINE_STRIP);


    if (line_mode_ == LineSegmentsMode)
    {
      // first check if (new standard) 4-channel colors are specified
      if( (points_.size()/2 == colors4f_.size()) )
      {
        // enable blending of lines
        GLboolean blendb;
        glGetBooleanv( GL_BLEND, &blendb);
        glEnable(GL_BLEND);
        // blend ontop of prev. drawn mesh
        GLboolean depthmaskb;
        glGetBooleanv( GL_DEPTH_WRITEMASK, &depthmaskb);
        glDepthMask(GL_FALSE);

        glBegin(GL_LINES);

        ConstPointIter p_it=points_.begin(), p_end=points_.end();
        ConstColor4fIter c_it=colors4f_.begin();

        Color4f c(1.0f,1.0f,1.0f,1.0f);
        if(c_it != colors4f_.end()) {
          c = *c_it;
        }

        int cnt = 0;
        for (; p_it!=p_end; ++p_it)
        {
          if ((cnt > 0) && (cnt % 2 == 0) && (c_it+1) != colors4f_.end()) {
            ++c_it;
            c = *c_it;
          }

          glColor(c);
          glVertex(*p_it);

          ++cnt;
        }

        glEnd();

        // disable blending of lines
        if( blendb == GL_FALSE )
          glDisable(GL_BLEND);

        // enable depth mask
        if( depthmaskb == GL_TRUE )
          glDepthMask(GL_TRUE);

      }
      else if ((line_mode_ == LineSegmentsMode) && (points_.size()/2 == colors_.size()) )
      {
        glBegin(GL_LINES);
        ConstPointIter p_it=points_.begin(), p_end=points_.end();
        ConstColorIter c_it=colors_.begin();

        Color c((char)255, (char)255, (char)255);
        if(c_it != colors_.end()) {
          c = *c_it;
        }

        int cnt = 0;
        for (; p_it!=p_end; ++p_it)
        {
          if ((cnt > 0) && (cnt % 2 == 0) && (c_it+1) != colors_.end()) {
            ++c_it;
            c = *c_it;
          }

          glColor(c);
          glVertex(*p_it);

          ++cnt;
        }
        glEnd();
      }
      else
      {
        glBegin(GL_LINES);

        ConstPointIter p_it=points_.begin(), p_end=points_.end();

        for (; p_it!=p_end; ++p_it)
        {
          glVertex(*p_it);
        }

        glEnd();
      }
    }
    else
    {
      _state.set_color(_state.base_color());
      glBegin(GL_LINE_STRIP);
      ConstPointIter p_it=points_.begin(), p_end=points_.end();
      for (; p_it!=p_end; ++p_it)
        glVertex(*p_it);
      glEnd();
    }

    //glEnd();
  }
}

//----------------------------------------------------------------------------

void LineNode::pickCompat(GLState&  _state , PickTarget _target)
{
  if (n_points() == 0)
    return;

  // Bind the vertex array
  ACG::GLState::bindBuffer(GL_ARRAY_BUFFER_ARB, 0);
  ACG::GLState::vertexPointer( &(points_)[0] );
  ACG::GLState::enableClientState(GL_VERTEX_ARRAY);

  const size_t n_edges = n_points() - 1;

  switch (_target)
  {
    case PICK_EDGE:
    {
      _state.pick_set_maximum (n_edges);
      pick_edgesCompat(_state, 0);
      break;
    }

    case PICK_ANYTHING:
    {
      _state.pick_set_maximum (n_edges);
      pick_edgesCompat(_state, 0);
      break;
    }

    default:
      break;
  }

  //Disable the vertex array
  ACG::GLState::disableClientState(GL_VERTEX_ARRAY);
}

//----------------------------------------------------------------------------

void LineNode::pick_edgesCompat(GLState& _state, unsigned int _offset)
{
  // Check if we have any edges to draw (% 0 causes division by zero on windows)
  if (n_points() < 2)
    return;

  const float line_width_old = _state.line_width();
  _state.set_line_width(picking_line_width());
  _state.pick_set_name (0);

  glDepthRange(0.0, 0.99);

  if (line_mode_ == PolygonMode)
  {
    const size_t n_edges = n_points() - 1;
    for (size_t i = 0; i < n_edges; ++i)
    {
      _state.pick_set_name(i + _offset);
      glBegin(GL_LINES);
      glArrayElement(static_cast<GLint>(i));
      glArrayElement(static_cast<GLint>(i + 1));
      glEnd();
    }
  }
  else if (line_mode_ == LineSegmentsMode)
  {
    const size_t n_edges = n_points() / 2;
    for (size_t i = 0; i < n_edges; ++i)
    {
      _state.pick_set_name(i + _offset);
      glBegin(GL_LINES);
      glArrayElement(static_cast<GLint>(2*i));
      glArrayElement(static_cast<GLint>(2*i + 1));
      glEnd();
    }
  }

  glDepthRange(0.0, 1.0);

  _state.set_line_width(line_width_old);
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

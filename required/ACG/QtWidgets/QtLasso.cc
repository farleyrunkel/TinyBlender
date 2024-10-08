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
//  CLASS QtLasso - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================


//ACG
#include "QtLasso.hh"
#include "QtColorTranslator.hh"

// stdc++



//== NAMESPACES ===============================================================

namespace ACG {


//== IMPLEMENTATION ==========================================================



#define MASK(x,y) (mask_[(x)+(y)*mask_width_])


//-----------------------------------------------------------------------------


QtLasso::
QtLasso(GLState& _glstate) :
  glstate_(_glstate),
  mask_(0),
  mask_width_(0),
  mask_height_(0),
  is_active_(false)
{
}


//-----------------------------------------------------------------------------


QtLasso::
~QtLasso()
{
  free_mask();
}


//-----------------------------------------------------------------------------


void
QtLasso::
slotMouseEvent(QMouseEvent* _event)
{
  bool emit_signal = false;


  // setup 2D projection
  unsigned int width  = glstate_.viewport_width();
  unsigned int height = glstate_.viewport_height();

  ACG::GLState::drawBuffer( GL_FRONT );

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  GLMatrixf orthoProj;
  orthoProj.identity();
  orthoProj.ortho(0.0f, float(width-1), 0.0f, float(height-1), -1.0f, 1.0f);
  glLoadMatrixf(orthoProj.data());


  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  ACG::GLState::disable(GL_DEPTH_TEST);
  ACG::GLState::disable(GL_LIGHTING);
  ACG::GLState::disable(GL_DITHER);
  glLineWidth(2.0);
  glColor3ub(0, 255, 10);

  glFinish();



  // process mouse event
  switch(_event->type())
  {
    case QEvent::MouseButtonPress:
    {
      Vec2i p(_event->pos().x(), height-_event->pos().y()-1);

      // initialize
      if (!is_active())
      {
        is_active_ = true;
        first_point_ = last_point_ = p;
      }

      // draw line
      glBegin(GL_LINES);
      glVertex(last_point_);
      glVertex(p);
      glEnd();
      last_point_ = rubberband_point_ = p;
      break;
    }


    case QEvent::MouseMove:
    {
      if (is_active())
      {
        Vec2i p(_event->pos().x(), height-_event->pos().y());

        // draw freehand
        if (_event->modifiers() & Qt::LeftButton)
        {
          glBegin(GL_LINES);
          glVertex(last_point_);
          glVertex(p);
          glEnd();
          last_point_ = rubberband_point_ = p;
        }

        // draw rubber band
        else
        {
          ACG::GLState::enable(GL_COLOR_LOGIC_OP);
          glLogicOp(GL_XOR);
          glBegin(GL_LINES);
          glVertex(last_point_);
          glVertex(rubberband_point_);
          glVertex(last_point_);
          glVertex(p);
          glEnd();
          ACG::GLState::disable(GL_COLOR_LOGIC_OP);
          rubberband_point_ = p;
        }
      }
      break;
    }


    case QEvent::MouseButtonDblClick:
    {
      if (is_active())
      {
        // close polygon
        glBegin(GL_LINES);
        glVertex(last_point_);
        glVertex(first_point_);
        glEnd();
        glFinish();


        // mark reference point (0,0) with border color
        glPointSize(1.0);
        glBegin(GL_POINTS);
        glVertex2i(0, 0);
        glEnd();
        glPointSize(glstate_.point_size());


        // create mask and precompute matrix
        create_mask();
        is_active_ = false;

        emit_signal = true;
      }
      break;
    }


    default: // avoid warning
      break;
  }


  // restore GL settings
  ACG::GLState::drawBuffer(GL_BACK);
  glReadBuffer(GL_BACK);

  glLineWidth(glstate_.line_width());
  glColor4fv(glstate_.base_color().data());
  ACG::GLState::enable(GL_DEPTH_TEST);

  glMatrixMode(GL_PROJECTION );
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glFinish();



  // emit signal
  if (emit_signal)
  {
    if (_event->modifiers() & Qt::ShiftModifier)
      emit(signalLassoSelection(AddToSelection));

    else if (_event->modifiers() & Qt::ControlModifier)
      emit(signalLassoSelection(DelFromSelection));

    else
      emit(signalLassoSelection(NewSelection));
  }
}


//-----------------------------------------------------------------------------


void
QtLasso::
create_mask()
{
  unsigned int  x, y, xx, yy, i;
  GLubyte       *fbuffer;
  QRgb          rgb, borderRgb;


  // GL context
  const unsigned int w = glstate_.viewport_width();
  const unsigned int h = glstate_.viewport_height();
  const unsigned int size = w*h;


  // alloc mask
  free_mask();
  mask_ = new unsigned char[size];
  mask_width_ = w;
  mask_height_ = h;


  // alloc framebuffer
  fbuffer = new GLubyte[3*size];
  assert( fbuffer );


  // read framebuffer
  glReadBuffer( GL_FRONT );
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, fbuffer);
  glReadBuffer( GL_BACK );


  // read lasso color
  borderRgb = qRgb( fbuffer[0], fbuffer[1], fbuffer[2] );
  fbuffer[0] = fbuffer[1] = fbuffer[2] = 0;


  // mark lasso pixels
  for (y = 0; y < h; ++y)
  {
    const unsigned int offset = y*w;

    for (x = 0; x < w; ++x)
    {
      i = 3*(offset + x);
      rgb = qRgb(fbuffer[i], fbuffer[i+1], fbuffer[i+2]);
      mask_[offset+x] = (rgb == borderRgb) ? 3 : 1;
    }
  }


  // seed fill
  std::vector<Vec2i> toDo;
  toDo.reserve(w*h);
  toDo.push_back(Vec2i(0,0));

  while (!toDo.empty())
  {
    Vec2i p = toDo.back();
    toDo.pop_back();

    x = p[0];
    y = p[1];
    unsigned char &s = MASK(x, y);

    if (s != 3)
    {
      s = 0;

      xx = x-1; yy = y;
      if ((xx<w) && (MASK(xx,yy)==1))
      {
        toDo.push_back(Vec2i(xx,yy));
        MASK(xx,yy) = 2;
      }

      xx = x+1; yy = y;
      if ((xx<w) && (MASK(xx,yy)==1))
      {
        toDo.push_back(Vec2i(xx,yy));
        MASK(xx,yy) = 2;
      }

      xx = x; yy = y-1;
      if ((yy<h) && (MASK(xx,yy)==1))
      {
        toDo.push_back(Vec2i(xx,yy));
        MASK(xx,yy) = 2;
      }

      xx = x; yy = y+1;
      if ((yy<h) && (MASK(xx,yy)==1))
      {
        toDo.push_back(Vec2i(xx,yy));
        MASK(xx,yy) = 2;
      }
    }
  }


  // free
  delete[] fbuffer;
}


//-----------------------------------------------------------------------------


void
QtLasso::
free_mask()
{
  if (mask_)
  {
    delete[] mask_;
    mask_ = 0;
    mask_width_ = mask_height_ = 0;
  }
}


//-----------------------------------------------------------------------------


bool
QtLasso::
is_vertex_selected(const Vec3d& _v)
{
  unsigned int  x, y, w, h;


  // size changed?
  w = glstate_.viewport_width();
  h = glstate_.viewport_height();
  if ((w != mask_width_) || (h != mask_height_))
  {
    std::cerr << "Lasso: viewport size has changed.\n";
    return false;
  }


  // project vertex to 2D integer coordinates
  Vec3d v = glstate_.project(_v);
  x = (unsigned int)(v[0] + 0.5);
  y = (unsigned int)(v[1] + 0.5);


  // near and far plane clipping
  if (v[2] < 0.0 || v[2] > 1.0)
    return false;


  // visible ?
  return ((v[2]>0.0) && (x<w) && (y<h) && (MASK(x,y)));
}


//=============================================================================
} // namespace ACG
//=============================================================================

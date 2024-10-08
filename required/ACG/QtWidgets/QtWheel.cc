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
//  CLASS QtWheel - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include "QtWheel.hh"

#include <cmath>
#include <iostream>

//#include <QDrawutil>

#include <QPainter>
#include <QMenu>
#include <QTimer>
#include <qdrawutil.h>


//== NAMESPACES ===============================================================


namespace ACG {
namespace QtWidgets {


//== IMPLEMENTATION ==========================================================


QtWheel::QtWheel(QWidget* _parent,
	       const char* /* _name */ ,
	       Orientation _orientation)
  : QFrame(_parent),
    angle_(0.0),
    lastAngle_(0.0),
    gear_(1.0),
    gearShift_(0),
    orientation_(_orientation),
    ticks_(36),
    marker_(false),
    dragging_(false),
    tracking_(false),
    palette_(palette())
{

  palette_.setColor( QPalette::Dark,  QColor(0,0,0));

  setFrameStyle( QtWheel::Panel | QtWheel::Raised );
  setLineWidth(2);

  setContextMenuPolicy ( Qt::CustomContextMenu );

  connect (this, SIGNAL (customContextMenuRequested(QPoint) ),
           this, SLOT( slotCustomContextMenuRequested(QPoint) ));

}


QtWheel::~QtWheel() {
}

//----------------------------------------------------------------------------

void QtWheel::mousePressEvent(QMouseEvent* _e)
{
  if (_e->button()==Qt::LeftButton) {
    pos_=_e->pos();
    dragging_=true;
    lastAngle_=angle_;
  }
}

void QtWheel::mouseReleaseEvent(QMouseEvent* _e)
{
  if (_e->button()==Qt::LeftButton)
  {
    dragging_=false;

     turn(_e->pos());
     emit angleChangedTo(angle_);
     emit angleChangedBy(angle_-lastAngle_);
  }
}


void QtWheel::mouseMoveEvent(QMouseEvent* _e)
{
  if (_e->buttons()&Qt::LeftButton)
  {
    float dAngle=turn(_e->pos());

    if (tracking_ && dAngle!=0.0) { // 0.0 is explicitly returned
      lastAngle_=angle_-dAngle;

      emit angleChangedTo(angle_);
      emit angleChangedBy(dAngle);
    }
  }
}


double QtWheel::turn(const QPoint& _pos)
{
  QPoint dPos=(_pos-pos_);
  pos_=_pos;

  int d = orientation_== Horizontal ? dPos.x() : dPos.y();

  double dAngle=0.0;

  if (d!=0) {
    // full width/height = 180 deg for gear()==1
    dAngle=double(d)/double(size_)*M_PI*gear_;
    angle_+=dAngle;

    redrawPixmap();
    repaint();
  }
  return dAngle;
}

//----------------------------------------------------------------------------

void QtWheel::mouseDoubleClickEvent(QMouseEvent* _e) {
  if (_e->button()==Qt::LeftButton) {
    int sz,x;
    if (orientation_== Horizontal) {
      sz=width();  x=_e->pos().x();
    }
    else {
      sz=height(); x=_e->pos().y();
    }

    if (x<sz/2) {
      if (gearShift_<8) {
        ++gearShift_;
        gear_*=2.0;

        redrawPixmap();
        repaint();

        emit gearUp();
      }
    }
    else {
      if (gearShift_>-8) {
        --gearShift_;
        gear_/=2.0;

        redrawPixmap();
        repaint();

        emit gearDown();
      }
    }
  }
}

//----------------------------------------------------------------------------

void QtWheel::keyPressEvent(QKeyEvent* _e) {

  //
  // This does not work! Do we really need/want keyboard input???
  //

  if (dragging_)
    return;

  double dAngle=0.0;

  if (!_e->isAutoRepeat())
    lastAngle_=angle_;

  switch (_e->key()) {
    case Qt::Key_Left:
    case Qt::Key_Up: {
      dAngle= -M_PI/double(ticks_)*gear_;
    }
    break;

    case Qt::Key_Right:
    case Qt::Key_Down: {
      dAngle= +M_PI/double(ticks_)*gear_;
    }
    break;

    default: return;
  }

  if (tracking_)
    lastAngle_=angle_;

  angle_+=dAngle;

  redrawPixmap();
  repaint();

  if (tracking_) {
    emit angleChangedTo(angle_);
    emit angleChangedBy(angle_-lastAngle_);
  }
}

void QtWheel::keyReleaseEvent(QKeyEvent* _e) {
  switch (_e->key()) {
    case Qt::Key_Left:
    case Qt::Key_Up:
    case Qt::Key_Right:
    case Qt::Key_Down: {
      if (!tracking_) {
        emit angleChangedTo(angle_);
        emit angleChangedBy(angle_-lastAngle_);
      }
    };
    break;

    default: return;
  }
}

//----------------------------------------------------------------------------

void QtWheel::resizeEvent(QResizeEvent* _e) {
  QFrame::resizeEvent(_e);
  redrawPixmap();
}

void QtWheel::paintEvent(QPaintEvent* _e) {
  if (isVisible()) {
    QFrame::paintEvent(_e);


    QPainter painter(this);
    QRect r=contentsRect();

    /// @todo: bitBlt(this,r.left(),r.top(),pixmap_);
    painter.drawPixmap( r.left(), r.top(), pixmap_ );

  }
}

//----------------------------------------------------------------------------

void QtWheel::redrawPixmap() {
 QRect r=contentsRect();

 if (r.width()<=0 || r.height()<=0) {
   pixmap_ = QPixmap( 0, 0 );
   return;
 }

 if (pixmap_.size()!=r.size())
   pixmap_ = QPixmap(r.size());

  QPainter paint;

  paint.begin( &pixmap_);
  pixmap_.fill( palette().window().color() );

  // coords of wheel frame
  QRect contents = contentsRect();
  contents.moveTopLeft(QPoint(0,0)); // transform to pixmap coord sys

  QPen pen(Qt::black, 1);
  paint.setPen(pen);

  if (orientation_ == Horizontal) {

    shrinkRect(contents, 3, 2);

    // draw a black frame
    paint.drawRect(contents);
    shrinkRect(contents, 1, 0);
    paint.drawRect(contents);
    shrinkRect(contents, 3, 2);

    int x0 = contents.left();
    int y0 = contents.top();
    int w0 = contents.width();
    int h0 = contents.height();

    size_=w0;

    if (gearShift_>0) {
      QBrush b; b.setColor(QColor(Qt::red)); b.setStyle(Qt::SolidPattern);
      int w=std::min(4*gearShift_,w0-8);
      paint.fillRect(x0+8,y0-1,w,h0+2,b);
    }
    else if (gearShift_<0) {
      QBrush b; b.setColor(QColor(Qt::blue));
      int w=std::min(-4*gearShift_,w0-8); b.setStyle(Qt::SolidPattern);
      paint.fillRect(x0+w0-w-8,y0-1,w,h0+2,b);
    }

    // draw the wheel
    double step = 2 * M_PI / (double) ticks_;
    for (int i = 0; i < ticks_; i++) {
      double x = sin(angle_ + i * step);
      double y = cos(angle_ + i * step);
      if (y>0) {
        qDrawShadeLine( &paint,
                        (int) (x0+(w0+x*w0)/2.0f), y0,
                        (int) (x0+(w0+x*w0)/2.0f), y0+h0,
                        palette_, false, 1, 1);
      }
    }
  }

  else if (orientation_ == Vertical) {

    shrinkRect(contents, 2, 3);

    // draw a black frame
    paint.drawRect(contents);
    shrinkRect(contents, 0, 1);
    paint.drawRect(contents);
    shrinkRect(contents, 2, 3);


    int x0 = contents.left();
    int y0 = contents.top();
    int w0 = contents.width();
    int h0 = contents.height();

    size_=h0;

    if (gearShift_>0) {
      QBrush b; b.setColor(QColor(Qt::red)); b.setStyle(Qt::SolidPattern);
      int h=-std::min(-4*gearShift_,h0-8);
      paint.fillRect(x0-1,y0+8,w0+2,h,b);
    }
    else if (gearShift_<0) {
      QBrush b; b.setColor(QColor(Qt::blue)); b.setStyle(Qt::SolidPattern);
      int h=-std::min(4*gearShift_,h0-8);
      paint.fillRect(x0-1,y0+h0-h-8,w0+2,h,b);
    }

    // draw the wheel
    double step = 2 * M_PI / (double) ticks_;
    for (int i = 0; i < ticks_; i++) {
      double x = sin(angle_ + i * step);
      double y = cos(angle_ + i * step);
      if (y>0) {
	     qDrawShadeLine( &paint,
                        x0,    (int) (y0+(h0+x*h0)/2.0f),
                        x0+w0, (int) (y0+(h0+x*h0)/2.0f),
                        palette_, false, 1, 1);
      } // if y
    } // for ticks_
  } // if Vertical
  paint.end();
}

//-----------------------------------------------------------------------------

void QtWheel::shrinkRect(QRect& _rect, int _dx, int _dy) {
  _rect.setLeft(_rect.left()+_dx);
  _rect.setRight(_rect.right()-_dx);
  _rect.setTop(_rect.top()+_dy); // dy -> -dy
  _rect.setBottom(_rect.bottom()-_dy);
}

//-----------------------------------------------------------------------------

QSizePolicy QtWheel::sizePolicy() const
{
  if (orientation_== Horizontal)
    return QSizePolicy(QSizePolicy::Preferred,
		       QSizePolicy::Minimum);
  else
    return QSizePolicy(QSizePolicy::Minimum,
		       QSizePolicy::Preferred);
}

//-----------------------------------------------------------------------------

QSize QtWheel::sizeHint() const
{
  if (orientation_==Horizontal)
    return QSize(120,20);
  else
    return QSize(20,120);
}

//-----------------------------------------------------------------------------


double QtWheel::clip(double _angle) {
  return fmod(_angle,2*M_PI);
}

double QtWheel::deg(double _angle) {
  return _angle*180.0/M_PI;
}

//-----------------------------------------------------------------------------

void QtWheel::slotCustomContextMenuRequested ( const QPoint & pos ) {

  QMenu* menu = new QMenu(this);
  QAction *hide = menu->addAction("Hide wheel");
  connect( hide, SIGNAL(triggered()) , this, SIGNAL(hideWheel()) );
  menu->popup( mapToGlobal(pos) );

}


//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================

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
//  CLASS QWheel
//
//=============================================================================

#ifndef ACG_QT_WHEEL_HH
#define ACG_QT_WHEEL_HH


//== INCLUDES =================================================================

#include <QPalette>
#include <QFrame>
#include <QMouseEvent>
#include <QPixmap>
#include <QKeyEvent>
#include <QResizeEvent>
#include <QPaintEvent>

#include "../Config/ACGDefines.hh"


//== FORWARDDECLARATIONS ======================================================

class QPaintEvent;
class QMouseEvent;
class QPainter;


//== NAMESPACES ===============================================================


namespace ACG {
namespace QtWidgets {


//== CLASS DEFINITION =========================================================


/** \class QWheel QtWheel.hh <ACG/Scenegraph/QtWheel.hh>
    \brief Inventor-like wheel widget.

    The wheel widget is typically turned with the mouse.  Dragging the
    wheel from one side of the widget to the other will changed the
    angle by \a pi (=180 deg()).  Use gear() to make the angle change
    faster or slower.

    The user can manually \em shift the gear by double clicking the
    left (top) or right (bottom) part of the widget.  gear() doubles
    or halves, cf gearUp(), gearDown(). The current gear setting is
    displayed by \a red (faster) or \a blue (slower) bars on the
    left/right (bottom/top) side of the widget.
**/

class ACGDLLEXPORT QtWheel : public QFrame
{
  Q_OBJECT
public:

  /// Orientation of the widget
  enum Orientation { Horizontal, Vertical };

  //
  // CONSTRUCTORS
  //

  /// Constructor.
  QtWheel(QWidget* _parent=0,
	 const char* _name=0,
	 Orientation _orientation = Horizontal);

  /// Destructor.
  virtual ~QtWheel();


  //
  // METHODS
  //

  /// get number of ticks on the wheel
  int ticks() const { return ticks_; }
  /// set \c _number of ticks on the wheel (default: 36)
  void setTicks(int _number) {
    ticks_=_number; redrawPixmap(); repaint();
  }

  /// get current angle of the wheel (radiants, unbounded, cf. clip())
  double angle() const { return angle_; }
  /// set current angle of the wheel (radiants, unbounded, cf. clip())
  void setAngle(double _angle) {
    angle_=_angle; lastAngle_=0.0; redrawPixmap(); repaint();
  }
  /** Get difference angle.
      You may call this method from a \a slot connected to
      angleChangedTo() and obtain the difference angle (equal to the
      argument to angleChangedBy()).
   */
  double diffAngle() const { return angle_-lastAngle_; }

  /// Are there markers on the wheel?
  bool marker() const { return marker_; }
  /// enable/disable markers on the wheel
  void setMarker(bool _b) {
    marker_=_b; redrawPixmap(); repaint();
  }

  /// get orientation
  Orientation orientation() const;
  /// set orientation
  void setOrientation(Orientation _orientation) {
    orientation_=_orientation; redrawPixmap(); repaint();
  }

  /// Is the user currently dragging the wheel whith the mouse?
  bool draggingWheel() const { return dragging_; }

  /** Enables wheel tracking (default: \a true)

      If tracking is enabled, then the QtWheel emits signals
      whenever the it is dragged. Otherwise signals are
      emitted only when a dragging process is finished.
   */
  void setTracking(bool _b) { tracking_=_b; }
  /// Is tracking on?
  bool tracking() const { return tracking_; }

  /** How fast does angle() changed when the wheel is turned?
      The default setting is 1.0. Then angle() will be changed
      by pi (180 degrees) when dragging the wheel from one end
      of the widget to the other. In general this angular
      difference will be multiplied by gear(), i.e. for values
      >1,0 the angle changes faster.
   */
  void setGear(double _g) { gear_=_g; }
  /// How fast does angle() change when the wheel is turned? (see above)
  double gear() const { return gear_; }

  /// reimplemented
  virtual QSizePolicy sizePolicy() const;
  /// reimplemented
  virtual QSize sizeHint() const;

  /** Clips \c _angle to range [0,2pi[.
      All angle()s provided by this widget are \a unbounded in a sense
      that a full turn of the wheel is 2pi \a not 0. As a consequence
      two full turns are 4pi, etc.

      This methods clips the angle again [0,2pi[
      \sa deg()
   */
  static double clip(double _angle);

  /// maps \c _angle from radiants to degrees (works also for clip()ped angles)
  static double deg(double _angle);

signals:
  /** The angle changed to the absolute \c _angle (cf. clip()).
      \a Note: Do not use together with angleChangedBy() (see there)!
    */
  void angleChangedTo(double _angle);
  /** The angle chaned by the relative \c _angle (cf. clip()).

      \a Note: Do \a not use this signal together with angleChangedBy()!
      The reason for this is that you cannot be sure in which order
      the you get the signals!

      If you need both values the angle and the relative difference
      then use this signal and angle() or angleChangedTo() and
      diffAngle() respectively!
   */
  void angleChangedBy(double _angle);

  /** The gear() has been "shifted" by double clicking the widget.
      Up-shifting will \a double the value of gear(). The signal is
      emitted \a after gear() has been changed.
   */
  void gearUp();
  /// Like gearUp() but the value of gear() halves.
  void gearDown();

  /// Emitted when the wheel will be hidden by the context menu
  void hideWheel();

protected:
  /// reimplemented
  virtual void mousePressEvent(QMouseEvent*);
  /// reimplemented
  virtual void mouseReleaseEvent(QMouseEvent*);
  /// reimplemented
  virtual void mouseMoveEvent(QMouseEvent*);
  /// reimplemented
  virtual void mouseDoubleClickEvent(QMouseEvent*);
  /// reimplemented
  virtual void keyPressEvent(QKeyEvent*);
  /// reimplemented
  virtual void keyReleaseEvent(QKeyEvent*);

  /// reimplemented
  virtual void resizeEvent(QResizeEvent*);
  /// reimplemented
  virtual void paintEvent(QPaintEvent*);
  /// reimplemented
  virtual void drawContents(QPainter*) {};


  /// draw wheel to pixmap (double buffering)
  virtual void redrawPixmap();

private slots:
  void slotCustomContextMenuRequested ( const QPoint & pos );

private:

  /// Copy constructor. Never used!
  QtWheel(const QtWheel&);
  /// Assignment operator. Never used!
  QtWheel& operator=(const QtWheel&);

  //// turn wheel to new \c _pos, returns delta(angle)
  double turn(const QPoint& _pos);
  void updateMenu();                  //!< updates the ticks of the menu
  void shrinkRect(QRect&, int, int);  //!< expands a rectangle in x/y direction

  double       angle_;                //!< current angle of the wheel
  double       lastAngle_;            //!< last angle, depends on tracking_
  int          size_;                 //!< size of wheel in pixels

  double       gear_;                 //!< speed of revolution
  int          gearShift_;            //!< click-shifted gear by 2^gearShift_

  Orientation  orientation_;          //!< orientation of the widget

  int          ticks_;                //!< number of ticks on the wheel
  bool         marker_;               //!< should ticks be marked by colors?

  QPoint       pos_;                  //!< recent mouse position
  bool         dragging_;             //!< currently dragging the slider?
  bool         tracking_;             //!< tracking on?

  QPalette     palette_;              //!< color group
  QPixmap      pixmap_;               //!< pixmap of the wheel
};

//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================
#endif // ACG_QWHEEL_HH defined
//=============================================================================


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
//  CLASS QtFlapBox
//
//=============================================================================

#ifndef ACG_QT_FLAPBOX_HH
#define ACG_QT_FLAPBOX_HH

//== INCLUDES =================================================================

#include <iostream>

#include <QPushButton>
#include <QBoxLayout>
#include <QResizeEvent>
#include <QFrame>
#include <QScrollArea>
#include "../Config/ACGDefines.hh"

//== NAMESPACES ===============================================================

namespace ACG {
namespace QtWidgets {

//== CLASS DEFINITION =========================================================

class ACGDLLEXPORT QtFlapBox : public QScrollArea
{
  Q_OBJECT

public:

#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
  QtFlapBox( QWidget * _parent = 0, Qt::WindowFlags _f = 0 );
#else
  QtFlapBox( QWidget * _parent = 0, Qt::WindowFlags _f = Qt::WindowFlags() );
#endif

  ~QtFlapBox ();

  int addItem( QWidget * _widget, const QIcon & _icon, const QString & _text );
  int addItem( QWidget * _widget, const QString & _text );

  int count () const;

  int indexOf( QWidget * _widget ) const;
  int insertItem( int _index, QWidget * _widget,
		  const QIcon & _icon, const QString & _text );
  int insertItem( int _index, QWidget * _widget, const QString & _text );

  bool isItemEnabled( int _index ) const;
  QIcon itemIcon( int _index ) const;

  QString itemText( int _index ) const;
  QString itemToolTip( int _index ) const;
  void removeItem( int _index );
  void setItemEnabled( int _index, bool _enabled );
  void setItemIcon( int _index, const QIcon & _icon );
  void setItemText( int _index, const QString & _text );
  void setItemToolTip( int _index, const QString & _toolTip );
  QWidget * widget( int _index ) const;

  // the following methods are specific for QtFlapBox

  bool isItemHidden( int _index ) const;
  void setItemHidden( int _index, bool _hidden );

  virtual QSize sizeHint() const;


  virtual void resizeEvent ( QResizeEvent * _event );

signals:

  void sizeHintChanged();


//   void toggled( int index, bool state );


// public slots:

//   void toggle( int index );


// private slots:

//   void flapToggled( bool );

  //  virtual QSize sizeHint () const;

private:


  struct Flap
  {
    QPushButton * button;
    QWidget     * widget;

    void setText( const QString & _text )
    {
      button->setText( _text );
    }

    void setIcon( const QIcon & _icon )
    {
      button->setIcon( _icon );
    }

    void setToolTip( const QString & _tip )
    {
      button->setToolTip( _tip );
    }

    QString text() const
    {
      return button->text();
    }

    QIcon icon() const
    {
      return button->icon();
    }

    QString toolTip() const
    {
      return button->toolTip();
    }

    bool operator==( const Flap & _other ) const
    {
      return widget == _other.widget;
    }
  };



private slots:

  void buttonClicked();
  void widgetDestroyed(QObject*);

private:

  typedef QList< Flap > FlapList;

        Flap * flap( QWidget * _widget ) const;
  const Flap * flap( int _index ) const;
        Flap * flap( int _index );

  void updateFlaps();
  void relayout();

  FlapList      flapList;
  QVBoxLayout * boxlayout;
  QWidget     * container;

};

//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================
#endif // ACG_QT_FLAPBOX_HH defined
//=============================================================================

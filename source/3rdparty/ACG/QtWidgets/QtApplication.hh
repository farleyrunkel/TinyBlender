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
//  CLASS QtApplication
//
//=============================================================================

#ifndef QTAPPLICATION_HH
#define QTAPPLICATION_HH


//== INCLUDES =================================================================


// Qt
#include <QApplication>
#include <QObject>
#include <QEvent>
#include <QElapsedTimer>
#include <QTimerEvent>

// stdc++
#include <vector>
#include <string>
#include <iostream>

#include "../Config/ACGDefines.hh"


//== FORWARD DECLARATIONS =====================================================


namespace ACG {
namespace QtWidgets {
  class QtMacroDialog;
}
}


//== NAMESPACES ===============================================================


namespace ACG {
namespace QtWidgets {


//== CLASS DEFINITION =========================================================


class ACGDLLEXPORT QtApplication : public QApplication
{
  Q_OBJECT

public:

  QtApplication(int _argc, char** _argv);
  virtual  ~QtApplication() { cleanUpEventBuffer(); }


public slots:

  // save recorded Events to file
  void saveFile(const char* _filename);

  // load recorded Events from file
  void loadFile(const char* _filename);

  // start playback
  void play();

  // stop playback or record
  void stop();

  // record
  void record();

  // change loop status for playback
  void loop(bool _b);


public:

  // filter events
  bool notify (QObject* _receiver, QEvent* _event);


private:

  // information for one event
  struct FootPrint
  {
    int          time;
    std::string  name;
    std::string  classname;
    std::string  parent;
    QPoint       cursorPos;
    QPoint       position;
    QSize        size;
    QEvent      *event;
  };

  typedef std::vector<FootPrint>  FootPrints;
  typedef FootPrints::iterator    FootPrintIter;


  // record and play events
  void recordEvent(QObject* _receiver, QEvent* _event);
  void playbackEvent(FootPrint & _fp);


  // store and restore sizes of all top-level widgets
  void storeTopLevelSizes();
  void restoreTopLevelSizes();

  // load and save sizes of top-level widgets
  void saveTopLevelSizes(std::ostream & _os);
  void loadTopLevelSizes(std::istream & _is);


  // find Widget for event _fp
  QWidget* findWidget(FootPrint & _fp);
  // timer callback
  void timerEvent(QTimerEvent* _e);
  // clear event buffer (pointers to QEvent)
  void cleanUpEventBuffer();


private:

  // state variables
  bool record_;
  bool playback_;
  bool play_loop_;

  // counter
  unsigned int eventnr_;

  // timer ID
  int timer_id_;

  // stop Time
  QElapsedTimer time_;

  // vector to store events and top-levels sizes
  FootPrints events_, toplevels_;

  // macro dialog for interaction
  QtMacroDialog* dialog_;

  // Pointer to MainWidget
  QWidget* mainWidget_;

  // Main widget pos
  QPoint mainWidgetDiff_;

  // Old Main widget pos
  QPoint oldMainWidgetPos_;
};

//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================
#endif // MACROAPPLICATION_HH defined
//=============================================================================

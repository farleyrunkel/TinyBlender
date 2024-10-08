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
//  CLASS QtClippingDialog
//
//=============================================================================


#ifndef ACG_QTCLIPPINGDIALOG_HH
#define ACG_QTCLIPPINGDIALOG_HH


//== INCLUDES =================================================================


#include "ui_QtClippingDialogUi.h"
#include "../Config/ACGDefines.hh"

#include <QDialog>


//== FORWARDDECLARATIONS ======================================================


namespace ACG {
  namespace SceneGraph {
    class ClippingNode;
    class BaseNode;
  }
}


//== NAMESPACES ===============================================================


namespace ACG {
namespace QtWidgets {


//== CLASS DEFINITION =========================================================


class ACGDLLEXPORT QtClippingDialog : public QDialog
{
  Q_OBJECT

public:

  /// Default constructor
  QtClippingDialog( QWidget                  * _parent,
		    SceneGraph::ClippingNode * _node );

  /// Destructor
  ~QtClippingDialog() {}


  void update_values();
  void show();
  void set_sweep_range(float _radius);


signals:

  void signalNodeChanged(ACG::SceneGraph::BaseNode* _node);

private slots:

  void set_plane();
  void sweep_plane(int _i);

private:

  SceneGraph::ClippingNode * node_;
  Ui::QtClippingDialogUi ui_;
};


//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================
#endif // ACG_QTCLIPPINGDIALOG_HH defined
//=============================================================================


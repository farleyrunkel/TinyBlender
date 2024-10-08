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
//  CLASS QtCoordFrameDialog
//
//=============================================================================


#ifndef ACG_QTCOORDFRAMEDIALOG_HH
#define ACG_QTCOORDFRAMEDIALOG_HH


//== INCLUDES =================================================================


#include "ui_QtCoordFrameDialogUi.h"
#include "../Math/VectorT.hh"
#include <vector>

#include <QDialog>
#include <QWidget>


//== FORWARDDECLARATIONS ======================================================


namespace ACG {
  namespace SceneGraph {
    class CoordFrameNode;
    class BaseNode;
  }
}


//== NAMESPACE ================================================================


namespace ACG {
namespace QtWidgets {


//== CLASS DEFINITION =========================================================


class ACGDLLEXPORT QtCoordFrameDialog : public QDialog
{
  Q_OBJECT

public:

  QtCoordFrameDialog( QWidget* parent,
		      SceneGraph::CoordFrameNode* _node,
		      Qt::WindowFlags fl = Qt::Widget );

  ~QtCoordFrameDialog() {}


  void show();


signals:

  void signalNodeChanged(ACG::SceneGraph::BaseNode* _node);

private slots:

  void add_x_plane();
  void add_y_plane();
  void add_z_plane();
  void del_x_plane();
  void del_y_plane();
  void del_z_plane();
  void mod_x_plane();
  void mod_y_plane();
  void mod_z_plane();
  void apply_changes();
  void undo_changes();

private:

  void update_values();

  void combo2planes(const QComboBox* _combo,
		    std::vector<float>& _planes);
  void planes2combo(const std::vector<float>& _planes,
		    QComboBox* _combo);


private:

  SceneGraph::CoordFrameNode* node_;

  std::vector<float> x_planes_bak_;
  std::vector<float> y_planes_bak_;
  std::vector<float> z_planes_bak_;

  Ui::QtCoordFrameDialogUi ui_;
};


//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================
#endif // ACG_QTCOORDFRAMEDIALOG_HH defined
//=============================================================================


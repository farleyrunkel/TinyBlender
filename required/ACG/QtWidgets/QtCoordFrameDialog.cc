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
//  CLASS QtCoordFrameDialog - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "QtCoordFrameDialog.hh"
#include "../Scenegraph/CoordFrameNode.hh"


//== NAMESPACES ============================================================== 


namespace ACG {
namespace QtWidgets {


//== IMPLEMENTATION ========================================================== 


QtCoordFrameDialog::
QtCoordFrameDialog( QWidget*                     _parent,
		    SceneGraph::CoordFrameNode*  _node,
		    Qt::WindowFlags                   _fl )
  
  : QDialog(_parent, _fl),
    node_(_node)
  
{
  ui_.setupUi( this );

   connect( ui_.OkButton, SIGNAL( clicked() ), this, SLOT( apply_changes() ) );
   connect( ui_.OkButton, SIGNAL( clicked() ), this, SLOT( accept() ) );
   connect( ui_.ApplyButton, SIGNAL( clicked() ), this, SLOT( apply_changes() ) );
   connect( ui_.CancelButton, SIGNAL( clicked() ), this, SLOT( reject() ) );
   connect( ui_.CancelButton, SIGNAL( clicked() ), this, SLOT( undo_changes() ) );
   connect( ui_.x_add_button, SIGNAL( clicked() ), this, SLOT( add_x_plane() ) );
   connect( ui_.x_mod_button, SIGNAL( clicked() ), this, SLOT( mod_x_plane() ) );
   connect( ui_.x_del_button, SIGNAL( clicked() ), this, SLOT( del_x_plane() ) );
   connect( ui_.y_add_button, SIGNAL( clicked() ), this, SLOT( add_y_plane() ) );
   connect( ui_.y_mod_button, SIGNAL( clicked() ), this, SLOT( mod_y_plane() ) );
   connect( ui_.y_del_button, SIGNAL( clicked() ), this, SLOT( del_y_plane() ) );
   connect( ui_.z_add_button, SIGNAL( clicked() ), this, SLOT( add_z_plane() ) );
   connect( ui_.z_mod_button, SIGNAL( clicked() ), this, SLOT( mod_z_plane() ) );
   connect( ui_.z_del_button, SIGNAL( clicked() ), this, SLOT( del_z_plane() ) );

  update_values();
}


//-----------------------------------------------------------------------------


void 
QtCoordFrameDialog::show()
{
  update_values();
  QDialog::show();
}


//-----------------------------------------------------------------------------


void
QtCoordFrameDialog::update_values()
{
  x_planes_bak_ = node_->x_planes();
  y_planes_bak_ = node_->y_planes();
  z_planes_bak_ = node_->z_planes();

  planes2combo(x_planes_bak_, ui_.x_combobox);
  planes2combo(y_planes_bak_, ui_.y_combobox);
  planes2combo(z_planes_bak_, ui_.z_combobox);


  QString s, title;

  title = ( QString("X-Planes: [") +
	    s.setNum(node_->bb_min()[0], 'f', 4) +
	    QString(", ") +
	    s.setNum(node_->bb_max()[0], 'f', 4) +
	    QString("]") );
  ui_.x_groupbox->setTitle(title);

  title = ( QString("Y-Planes: [") +
	    s.setNum(node_->bb_min()[1], 'f', 4) +
	    QString(", ") +
	    s.setNum(node_->bb_max()[1], 'f', 4) +
	    QString("]") );
  ui_.y_groupbox->setTitle(title);

  title = ( QString("Z-Planes: [") +
	    s.setNum(node_->bb_min()[2], 'f', 4) +
	    QString(", ") +
	    s.setNum(node_->bb_max()[2], 'f', 4) +
	    QString("]") );
  ui_.z_groupbox->setTitle(title);
}


//-----------------------------------------------------------------------------


void 
QtCoordFrameDialog::combo2planes(const QComboBox* _combo, 
				 std::vector<float>& _planes)
{
  unsigned int i(0), N(_combo->count());

  _planes.clear();
  _planes.reserve(N);

  for (; i<N; ++i)
    _planes.push_back(_combo->itemText(i).toFloat());
}


void 
QtCoordFrameDialog::planes2combo(const std::vector<float>& _planes,
				 QComboBox* _combo)
{
  std::vector<float>::const_iterator p_it, p_end;
  QString s;

  _combo->clear();
  for (p_it=_planes.begin(), p_end=_planes.end(); p_it!=p_end; ++p_it)
    _combo->addItem(s.setNum(*p_it, 'f', 3));
}


//-----------------------------------------------------------------------------


void QtCoordFrameDialog::apply_changes()
{
  std::vector<float> planes;

  combo2planes(ui_.x_combobox, planes);
  node_->set_x_planes(planes);

  combo2planes(ui_.y_combobox, planes);
  node_->set_y_planes(planes);

  combo2planes(ui_.z_combobox, planes);
  node_->set_z_planes(planes);

  emit signalNodeChanged(node_);
}


//-----------------------------------------------------------------------------


void QtCoordFrameDialog::undo_changes()
{
  node_->set_x_planes(x_planes_bak_);
  node_->set_y_planes(y_planes_bak_);
  node_->set_z_planes(z_planes_bak_);

  emit signalNodeChanged(node_);
}


//-----------------------------------------------------------------------------


void QtCoordFrameDialog::add_x_plane()
{
  ui_.x_combobox->addItem(ui_.x_combobox->currentText());
  apply_changes();
}

void QtCoordFrameDialog::mod_x_plane()
{
  ui_.x_combobox->setItemText(ui_.x_combobox->currentIndex(),ui_.x_combobox->currentText());
  apply_changes();
}

void QtCoordFrameDialog::del_x_plane()
{
  ui_.x_combobox->removeItem(ui_.x_combobox->currentIndex());
  apply_changes();
}


//-----------------------------------------------------------------------------


void QtCoordFrameDialog::add_y_plane()
{
  ui_.y_combobox->addItem(ui_.y_combobox->currentText());
  apply_changes();
}

void QtCoordFrameDialog::mod_y_plane()
{
  ui_.y_combobox->setItemText(ui_.y_combobox->currentIndex(),ui_.y_combobox->currentText());
  apply_changes();
}

void QtCoordFrameDialog::del_y_plane()
{
  ui_.y_combobox->removeItem(ui_.y_combobox->currentIndex());
  apply_changes();
}


//-----------------------------------------------------------------------------


void QtCoordFrameDialog::add_z_plane()
{
  ui_.z_combobox->addItem(ui_.z_combobox->currentText());
  apply_changes();
}

void QtCoordFrameDialog::mod_z_plane()
{
  ui_.z_combobox->setItemText(ui_.z_combobox->currentIndex(),ui_.z_combobox->currentText());
  apply_changes();
}

void QtCoordFrameDialog::del_z_plane()
{
  ui_.z_combobox->removeItem(ui_.z_combobox->currentIndex());
  apply_changes();
}


//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================

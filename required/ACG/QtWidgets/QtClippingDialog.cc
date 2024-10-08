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
//  CLASS QtClippingDialog - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================


#include "QtClippingDialog.hh"

#include "../Scenegraph/ClippingNode.hh"


//== NAMESPACES ===============================================================


namespace ACG {
namespace QtWidgets {


//== IMPLEMENTATION ========================================================== 


QtClippingDialog::
QtClippingDialog( QWidget                  * _parent,
		  SceneGraph::ClippingNode * _node )
  : QDialog( _parent ),
    node_( _node )
{
  ui_.setupUi( this );

  connect( ui_.setButton, SIGNAL( clicked() ),
	   this, SLOT( set_plane() ) );
  connect( ui_.sweepRangeSlider, SIGNAL( valueChanged(int) ),
	   this, SLOT( sweep_plane(int) ) );

  update_values();
  connect( ui_.okButton, SIGNAL( clicked() ),
	   this, SLOT( accept() ) );

  layout()->setSizeConstraint( QLayout::SetFixedSize );

}
 

//-----------------------------------------------------------------------------


void 
QtClippingDialog::show()
{
  update_values();
  QDialog::show();
}


//-----------------------------------------------------------------------------


void 
QtClippingDialog::update_values()
{
  QString s;

  const Vec3f& position = node_->position();
  ui_.positionXLineEdit->setText(s.setNum(position[0], 'f', 3));
  ui_.positionYLineEdit->setText(s.setNum(position[1], 'f', 3));
  ui_.positionZLineEdit->setText(s.setNum(position[2], 'f', 3));

  const Vec3f& normal = node_->normal();
  ui_.normalXLineEdit->setText(s.setNum(normal[0], 'f', 3));
  ui_.normalYLineEdit->setText(s.setNum(normal[1], 'f', 3));
  ui_.normalZLineEdit->setText(s.setNum(normal[2], 'f', 3));

  float slice_width = node_->slice_width();
  ui_.sliceWidthLineEdit->setText(s.setNum(slice_width, 'f', 3));
}


//-----------------------------------------------------------------------------


void 
QtClippingDialog::set_plane()
{
  // set plane
  Vec3f position( ui_.positionXLineEdit->text().toFloat(),
		  ui_.positionYLineEdit->text().toFloat(),
		  ui_.positionZLineEdit->text().toFloat() );

  Vec3f normal( ui_.normalXLineEdit->text().toFloat(),
		ui_.normalYLineEdit->text().toFloat(),
		ui_.normalZLineEdit->text().toFloat() );

  float slice_width = ui_.sliceWidthLineEdit->text().toFloat();

  node_->set_plane(position, normal, slice_width);


  // reset slider
  ui_.sweepRangeSlider->setValue(0);


  emit signalNodeChanged(node_);
}


//-----------------------------------------------------------------------------


void 
QtClippingDialog::set_sweep_range(float _radius)
{
  QString s;
  ui_.sweepRangeLineEdit->setText(s.setNum(_radius, 'f', 3));
}


//-----------------------------------------------------------------------------


void 
QtClippingDialog::sweep_plane(int _i)
{
  float range  = ui_.sweepRangeLineEdit->text().toFloat();
  float offset = ((float)_i) / 100.0 * range;

  node_->set_offset(offset);

  emit signalNodeChanged(node_);
}


//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================

/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2016, RWTH-Aachen University                 *
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
//  CLASS QtPrincipalAxisDialog - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include "QtPrincipalAxisDialog.hh"

#include <QtGui>
#include <limits>

//== NAMESPACES ===============================================================

namespace ACG {

//== IMPLEMENTATION ========================================================== 


//-----------------------------------------------------------------------------

static Vec4f byte2float(unsigned char r, unsigned char g, unsigned char b) {
    return Vec4f(r/255.0, g/255.0, b/255.0, 1.0);
}

static float colorSchemeDistance(const QtPrincipalAxisDialog::ColorScheme a,
        const QtPrincipalAxisDialog::ColorScheme b) {

    float result = 0;
    for (int i = 0; i < 3; ++i) {
        result += (a[i] - b[i]).norm();
    }

    return result;
}

const QtPrincipalAxisDialog::ColorScheme QtPrincipalAxisDialog::color_schemes_[3] = {
    { byte2float(232, 28, 23), byte2float(0, 110, 225), byte2float(0, 179, 0) },
    { byte2float(26, 178, 0), byte2float(17, 0, 255), byte2float(204, 77, 0) },
    { byte2float(255, 53, 139), byte2float(1, 176, 240), byte2float(174, 238, 0) },
};
static const size_t N_COLOR_SCHEMES = 3;

void 
QtPrincipalAxisDialog::
get_parameters()
{
//     QRadioButton* drawStyle3D;
//     QRadioButton* drawStyle2D;
//     QRadioButton* colorModeAxis;
//     QRadioButton* colorModeSign;
//     QCheckBox* showTensor1;
//     QCheckBox* showTensor2;
//     QCheckBox* showTensor3;
//     QLineEdit* maxDrawRadius;
//     QLineEdit* minDrawRadius;
//     QLineEdit* radiusScale;
//     QPushButton* OkButton;
//     QPushButton* CancelButton;

  if( pnode_.draw_style_ == PrincipalAxisNode::DS_3D)
    drawStyle3D->setChecked(true);
  else
    drawStyle2D->setChecked(true);

  if( pnode_.color_mode_ == PrincipalAxisNode::CM_Axis)
    colorModeAxis->setChecked(true);
  else
    colorModeSign->setChecked(true);
  
  showTensor1->setChecked( pnode_.show_tensor_component_[0] > 0);
  showTensor2->setChecked( pnode_.show_tensor_component_[1] > 0);
  showTensor3->setChecked( pnode_.show_tensor_component_[2] > 0);

  showTensor1B->setChecked( pnode_.show_tensor_component_[0] > 1);
  showTensor2B->setChecked( pnode_.show_tensor_component_[1] > 1);
  showTensor3B->setChecked( pnode_.show_tensor_component_[2] > 1);

  QString d;

  maxDrawRadius->setText(d.setNum(pnode_.max_draw_radius_));
  minDrawRadius->setText(d.setNum(pnode_.min_draw_radius_));
  radiusScale->setText  (d.setNum(pnode_.cylinder_radius_scale_));
  minSpacing->setText  (d.setNum(pnode_.min_spacing_));

  ColorScheme cs;
  pnode_.get_axes_colors(cs);
  setColorScheme(cs);
}


//-----------------------------------------------------------------------------


void 
QtPrincipalAxisDialog::
set_parameters()
{
  // set drawstyle
  if( drawStyle3D->isChecked() )
    pnode_.draw_style_ = PrincipalAxisNode::DS_3D;
  else
    pnode_.draw_style_ = PrincipalAxisNode::DS_2D;

  // set colormode
  if( colorModeAxis->isChecked())
    pnode_.color_mode_ = PrincipalAxisNode::CM_Axis;
  else
    pnode_.color_mode_ = PrincipalAxisNode::CM_Sign;


  // set drawTensorComponents
  pnode_.show_tensor_component_[0] = showTensor1->isChecked();
  pnode_.show_tensor_component_[1] = showTensor2->isChecked();
  pnode_.show_tensor_component_[2] = showTensor3->isChecked();

  // Both tensor directions?
  if( showTensor1B->isChecked())
    pnode_.show_tensor_component_[0] = 2;
  if( showTensor2B->isChecked())
    pnode_.show_tensor_component_[1] = 2;
  if( showTensor3B->isChecked())
    pnode_.show_tensor_component_[2] = 2;


  pnode_.max_draw_radius_       = maxDrawRadius->text().toDouble();
  pnode_.min_draw_radius_       = minDrawRadius->text().toDouble();
  pnode_.cylinder_radius_scale_ = radiusScale->text().toDouble();
  pnode_.min_spacing_           = minSpacing->text().toDouble();

  pnode_.set_axes_colors(getSelectedColorScheme());

  pnode_.auto_update_range();
}


//-----------------------------------------------------------------------------


void 
QtPrincipalAxisDialog::
slotOkButton()
{
  set_parameters();
  close();
}


//-----------------------------------------------------------------------------


void
QtPrincipalAxisDialog::
slotCancelButton()
{
  close();
}

const QtPrincipalAxisDialog::ColorScheme &QtPrincipalAxisDialog::getSelectedColorScheme() {
    if (colorscheme_1_rb->isChecked()) {
        return color_schemes_[0];
    } else if (colorscheme_2_rb->isChecked()) {
        return color_schemes_[1];
    } else if (colorscheme_3_rb->isChecked()) {
        return color_schemes_[2];
    } else {
        // Fallback.
        return color_schemes_[0];
    }
}

void QtPrincipalAxisDialog::setColorScheme(ColorScheme cs) {
    float best_match = std::numeric_limits<float>::infinity();
    size_t best_match_i = std::numeric_limits<size_t>::max();

    for (size_t i = 0; i < N_COLOR_SCHEMES; ++i) {
        float match = colorSchemeDistance(cs, color_schemes_[i]);
        if (match < best_match) {
            best_match = match;
            best_match_i = i;
        }
    }

    switch (best_match_i) {
        case 0:
            colorscheme_1_rb->setChecked(true);
            break;
        case 1:
            colorscheme_2_rb->setChecked(true);
            break;
        case 2:
            colorscheme_3_rb->setChecked(true);
            break;
        default:
            break;
    }
}


//=============================================================================
} // namespace ACG
//=============================================================================

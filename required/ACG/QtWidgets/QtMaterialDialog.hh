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
//  CLASS QtMaterialDialog
//
//=============================================================================


#pragma once


//== INCLUDES =================================================================


#include "ui_QtMaterialDialogUi.h"

#include "../Math/VectorT.hh"
#include "../GL/gl.hh"

#include <QColor>
#include <QDialog>

//== FORWARDDECLARATIONS ======================================================


namespace ACG {
  namespace SceneGraph {
    class MaterialNode;
    class BaseNode;
  }
}


//== NAMESPACE ================================================================


namespace ACG {
namespace QtWidgets {


//== CLASS DEFINITION =========================================================


class ACGDLLEXPORT QtMaterialDialog : public QDialog
{
  Q_OBJECT

public:

  QtMaterialDialog( QWidget                  * _parent,
		              SceneGraph::MaterialNode * _node );

  ~QtMaterialDialog() {}


signals:

  void signalNodeChanged( ACG::SceneGraph::BaseNode * _node );

private:


  QColor convertColor( Vec4f  _color);
  Vec4f  convertColor( QColor _color);

  void setButtonColor( QtColorChooserButton* _button,
		                   const Vec4f&          _color );

private slots:

  void changeBaseColor(QColor _newColor);
  void changeAmbientColor(QColor _newColor);
  void changeDiffuseColor(QColor _newColor);
  void changeSpecularColor(QColor _newColor);
  void changeOverlayColor(QColor _newColor);
  void changeShine(int _new);
  void changePointSize(double _new);
  void changeLineWidth(double _new);
  void changeRoundPoints(bool _b);
  void changeLineSmooth(bool _b);
  void changeBackfaceCulling(bool _b);
  void changeAlphaTest(bool _b);
  void changeAlphaValue(int _new);
  void changeBlending(bool _b);
  void changeBlendingParam1(const QString& _name);
  void changeBlendingParam2(const QString& _name);
  void changeColorMaterial(bool _b);
  void changeMultiSampling(bool _b);

  void changeActive(bool toggle);

  void enableProperty();
  void enableProperty(int i);
  void enableProperty(double d);

  QString paramToStr(GLenum param);

  void applyChanges();
  void undoChanges();

  void reject();

private:



  unsigned int  applyProperties_;

  Vec4f    color_,            bak_color_,
           ambient_,          bak_ambient_,
           diffuse_,          bak_diffuse_,
           specular_,         bak_specular_,
           overlay_,          bak_overlay_;
  float    shine_,            bak_shine_;
  float    point_size_,       bak_point_size_;
  float    line_width_,       bak_line_width_;
  bool     round_points_,     bak_round_points_;
  bool     line_smooth_,      bak_line_smooth_;
  bool     backfaceCulling_,  bak_backfaceCulling_;
  bool     alphaTest_,        bak_alphaTest_;
  float    alphaValue_,       bak_alphaValue_;
  bool     blending_,         bak_blending_;
  GLenum   blendParam1_,      bak_blendParam1_;
  GLenum   blendParam2_,      bak_blendParam2_;
  bool     colorMaterial_,    bak_colorMaterial_;
  bool     multiSampling_,    bak_multiSampling_;

  bool     baseColorActive_,       bak_baseColorActive_;
  bool     materialActive_,        bak_materialActive_;
  bool     pointSizeActive_,       bak_pointSizeActive_;
  bool     lineWidthActive_,       bak_lineWidthActive_;
  bool     roundPointsActive_,     bak_roundPointsActive_;
  bool     lineSmoothActive_,      bak_lineSmoothActive_;
  bool     alphaTestActive_,       bak_alphaTestActive_;
  bool     blendingActive_,        bak_blendingActive_;
  bool     backfaceCullingActive_, bak_backfaceCullingActive_;
  bool     colorMaterialActive_,   bak_colorMaterialActive_;
  bool     multiSamplingActive_,    bak_multiSamplingActive_;

  SceneGraph::MaterialNode * node_;

  Ui::QtMaterialDialogUi ui_;

};


//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================


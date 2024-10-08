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


#include "QtColorChooserButton.hh"

#include <QStyleOption>
#include <QStylePainter>
#include <QColorDialog>

#include <iostream>

QtColorChooserButton::QtColorChooserButton(QWidget *parent) :
        QPushButton(parent), color_(0xE0, 0x20, 0x20) {

    init();
}
QtColorChooserButton::QtColorChooserButton(const QString &text, QWidget *parent) :
        QPushButton(text, parent), color_(0xE0, 0x20, 0x20) {

    init();
}
QtColorChooserButton::QtColorChooserButton(const QIcon& icon, const QString &text, QWidget *parent) :
        QPushButton(icon, text, parent), color_(0xE0, 0x20, 0x20) {

    init();
}

QtColorChooserButton::~QtColorChooserButton() {
}

void QtColorChooserButton::init() {
    connect(this, SIGNAL( clicked() ), this, SLOT( onClick() ) );
}

void QtColorChooserButton::onClick() {
    QColor newColor = QColorDialog::getColor(color_, this, "Pick Color", QColorDialog::ShowAlphaChannel);
    if (newColor.isValid()) {
      color_ = newColor;
      emit colorChanged(color_);
    }
}

void QtColorChooserButton::paintEvent(QPaintEvent *ev) {
    QStyleOptionButton buttonOptions;
    initStyleOption(&buttonOptions);
    QStylePainter painter(this);

    const int textWd = buttonOptions.fontMetrics.horizontalAdvance(buttonOptions.text);
    QRect textRect = buttonOptions.rect;
    textRect.setWidth(std::min(textRect.width(), textWd));
    painter.drawItemText(textRect, Qt::TextSingleLine | Qt::TextShowMnemonic | Qt::AlignVCenter,
                         this->palette(), this->isEnabled(), buttonOptions.text, QPalette::ButtonText);

    buttonOptions.rect.adjust(textWd, 0, 0, 0);
    painter.drawControl(QStyle::CE_PushButtonBevel, buttonOptions);

    QRect colorRect = this->style()->subElementRect(QStyle::SE_PushButtonFocusRect, &buttonOptions, this);
    QStyleOptionFrame frameOptions;
    frameOptions.state = QStyle::State_Sunken;
    frameOptions.rect = colorRect;
    frameOptions.features = QStyleOptionFrame::None;

    /*
     * Draw checker board pattern.
     */
    if (this->isEnabled()) {
        static const int checkerSize = 7;
        static const QColor checkerColA(0xFF, 0xFF, 0xFF);
        static const QColor checkerColB(0x30, 0x30, 0x30);
        painter.setClipRect(colorRect, Qt::IntersectClip);
        for (int x = 0; x < colorRect.width() / checkerSize + 1; ++x) {
            for (int y = 0; y < colorRect.height() / checkerSize + 1; ++y) {
                painter.fillRect(colorRect.x() + x * checkerSize, colorRect.y() + y * checkerSize,
                                 checkerSize, checkerSize,
                                 (x % 2 == y % 2) ? checkerColA : checkerColB);
            }
        }

        painter.fillRect(colorRect, color_);
    }
    painter.drawPrimitive(QStyle::PE_FrameButtonBevel, frameOptions);
}

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

#ifndef QTHISTOGRAM_HH
#define QTHISTOGRAM_HH

// Based on ValenceHistogramWidget by hc

#include <memory>

#include <QWidget>
#include "../Config/ACGDefines.hh"
#include "../Utils/Histogram.hh"
#include "../Utils/ColorCoder.hh"

namespace ACG {
namespace QtWidgets {

class ACGDLLEXPORT QtHistogramWidget : public QWidget {
    Q_OBJECT

    public:
        explicit QtHistogramWidget(QWidget *parent);
        ~QtHistogramWidget();

        QtHistogramWidget(const QtHistogramWidget &other) = delete;
        QtHistogramWidget& operator=(const QtHistogramWidget &other) = delete;

        void setHistogram(std::unique_ptr<Histogram> histogram);
        void setColorCoder(std::unique_ptr<IColorCoder> color_coder);

    protected:
        void paintEvent(QPaintEvent *event);
        QColor getColor(double val); // val in [0..1]

        std::unique_ptr<Histogram> histogram_ = nullptr;
        double label_distance_ = 100;

        QColor color_; // ignored if we have a color coder
        std::unique_ptr<IColorCoder> color_coder_ = nullptr;

};


} // namespace QtWidgets
} // namespace ACG



#endif // QTHISTOGRAM_HH

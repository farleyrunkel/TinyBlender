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

// Based on QtHistogram by hc

#include "QtHistogramWidget.hh"

#include <QPainter>

namespace ACG {
namespace QtWidgets {

QtHistogramWidget::QtHistogramWidget(QWidget *parent)
    : QWidget(parent),
      color_(QColor::fromRgbF(0.518, 0.573, 0.643, 1.0))
{}

QtHistogramWidget::~QtHistogramWidget() = default;

void QtHistogramWidget::setHistogram(std::unique_ptr<Histogram> histogram) {
    histogram_ = std::move(histogram);
    this->update();
}

void QtHistogramWidget::setColorCoder(std::unique_ptr<IColorCoder> color_coder) {
    color_coder_ = std::move(color_coder);
    this->update();
}

void QtHistogramWidget::paintEvent(QPaintEvent *event) {
    if (!histogram_) {
        QWidget::paintEvent(event);
        return;
    }

    /*
     * Analyze histogram/
     */
    const std::vector<size_t> &bins = histogram_->getBins();
    const std::vector<double> &bin_widths = histogram_->getBinWidths();
    const double total_width = histogram_->getTotalWidth();


    size_t hist_max =  bins.size() > 0 ? *std::max_element(bins.begin(), bins.end()) : 0;

    /*
     * Establish regions
     */
    const qreal labelHeight = 16;
    QRectF paint_rect = this->contentsRect();
    QRectF bargraph_rect = paint_rect;
    bargraph_rect.setBottom(bargraph_rect.bottom() - labelHeight);
    QRectF label_rect = paint_rect;
    label_rect.setTop(bargraph_rect.bottom());
    QPainter painter(this);

    /*
     * Painter attributes.
     */
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setFont(this->font());

    const qreal avg_width = bargraph_rect.width() / bins.size();
    const qreal gap = (avg_width > 8) ? 1.0 : 0.0;
    const qreal label_gap = 4;
    const qreal y_scale = bargraph_rect.height() / hist_max;

    const qreal total_gap = (bins.size() - 1) * gap;
    const qreal total_barwidth = bargraph_rect.width() - total_gap;

    QRectF barRect;
    /*
     * Draw.
     */
    double cumulative_width = 0.0;
    qreal xpos = 0;
    qreal lastLabelX = label_rect.left();
    const size_t n_bins = bins.size();
    for (size_t idx = 0; idx < n_bins; ++idx) {
        const double bin_width = bin_widths[idx];
        const qreal bar_width = total_barwidth * (bin_width / total_width);
        // Bar
        painter.setPen(Qt::NoPen);
        // relative position t in [0..1] for the middle of the current bin
        const double t = (cumulative_width + bin_width/2) / total_width;
        cumulative_width += bin_width;

        painter.setBrush(getColor(t));

        barRect.setWidth(bar_width - gap);
        barRect.setHeight(y_scale * bins[idx]);
        barRect.moveBottomLeft(bargraph_rect.bottomLeft() + QPoint(xpos, 0));

        if (gap > 0.0)
            painter.drawRoundedRect(barRect, 3, 3, Qt::AbsoluteSize);
        else
            painter.drawRect(barRect);

        // Label
        painter.setPen(Qt::black);
        qreal labelX = 0;
        QString labelText;
        switch (histogram_->getLabelType()) {
        case Histogram::LabelType::PerBin:
            labelX = barRect.center().x();
            labelText = histogram_->getBinLabel(idx);
            break;
        case Histogram::LabelType::PerBoundary:
            labelX = barRect.x();
            labelText = histogram_->getBoundaryLabel(idx);
            break;
        }
        QRectF labelBB = painter.boundingRect(
                    QRectF(labelX - (label_distance_/2), label_rect.y(),
                           label_distance_, label_rect.height()),
                    Qt::AlignHCenter | Qt::AlignBottom, labelText);

        if (labelBB.left() >= lastLabelX + label_gap) {
            painter.drawText(labelBB, Qt::AlignHCenter | Qt::AlignBottom,
                    labelText);
            lastLabelX = labelBB.right();
        }
        xpos += bar_width;
    }
    // TODO: draw last perBoundary label?
}



QColor QtHistogramWidget::getColor(double val)
{
    if (color_coder_) {
        return color_coder_->color_qcolor(val);
    } else {
        return color_;
    }
}

}
}

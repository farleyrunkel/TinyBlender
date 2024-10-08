/*===========================================================================*\
*                                                                            *
*                              OpenFlipper                                   *
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
*                                                                            *
\*===========================================================================*/




// Include the plot header to get the qwt version
#include <qwt_plot.h>

// This file is for QWT >= 6.0 only
#if QWT_VERSION >= 0x060000

#include <qwt_plot_histogram.h>
#include <qwt_painter.h>
#include <qwt_scale_map.h>
#include <qwt_text.h>

#include <QString>
#include <QPainter>

#include <iostream>


#include "QwtHistogramm.hh"

class Histogram::PrivateData {

public:

  PrivateData() :
          attributes(Histogram::Auto),
          data(0),
          reference(0.0)
  {}

  int attributes;
  QwtIntervalSeriesData* data;
  std::vector<QColor> colors_;
  double reference;

};

Histogram::Histogram(const QwtText &title) :
        QwtPlotItem(title)
{
  init();
}

Histogram::Histogram(const QString &title) :
        QwtPlotItem(QwtText(title))
{
  init();
}

Histogram::~Histogram()
{
  delete d_data;
}

void Histogram::init()
{
  d_data = new PrivateData();

  setItemAttribute(QwtPlotItem::AutoScale, true);
  setItemAttribute(QwtPlotItem::Legend, true);

  setZ(20.0);
}

void Histogram::setBaseline(double reference)
{
  if (d_data->reference != reference) {
    d_data->reference = reference;
    itemChanged();
  }
}

double Histogram::baseline() const
{
  return d_data->reference;
}

void Histogram::setData(QwtIntervalSeriesData* data)
{
  d_data->data = data;
  itemChanged();
}

const QwtIntervalSeriesData* Histogram::data() const
{
  return d_data->data;
}

void Histogram::setColors(std::vector<QColor>& _colors)
{
  d_data->colors_ = _colors;
  itemChanged();
}

QColor Histogram::color(uint i) const
{
  if (i < d_data->colors_.size())
    return d_data->colors_[i];
  else
    return Qt::darkBlue;
}

QRectF Histogram::boundingRect() const
{
  QRectF rect = d_data->data->boundingRect();
  if (!rect.isValid())
    return rect;

  if (d_data->attributes & Xfy) {
    rect = QRectF(rect.y(), rect.x(), rect.height(), rect.width());

    if (rect.left() > d_data->reference)
      rect.setLeft(d_data->reference);
    else if (rect.right() < d_data->reference)
      rect.setRight(d_data->reference);
  } else {
    if (rect.bottom() < d_data->reference)
      rect.setBottom(d_data->reference);
    else if (rect.top() > d_data->reference)
      rect.setTop(d_data->reference);
  }

  return rect;
}


int Histogram::rtti() const
{
  return QwtPlotItem::Rtti_PlotHistogram;
}

void Histogram::setHistogramAttribute(HistogramAttribute attribute, bool on)
{
  if (bool(d_data->attributes & attribute) == on)
    return;

  if (on)
    d_data->attributes |= attribute;
  else
    d_data->attributes &= ~attribute;

  itemChanged();
}

bool Histogram::testHistogramAttribute(HistogramAttribute attribute) const
{
  return d_data->attributes & attribute;
}

void Histogram::draw(QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &) const
{
  const QwtIntervalSeriesData &iData = *(d_data->data);

  const int x0 = xMap.transform(baseline());
  const int y0 = yMap.transform(baseline());

  for (int i = 0; i < (int) iData.size(); i++) {
    if (d_data->attributes & Histogram::Xfy) {
      const int x2 = xMap.transform(iData.sample(i).value);
      if (x2 == x0)
        continue;

      int y1 = yMap.transform(iData.sample(i).interval.minValue());
      int y2 = yMap.transform(iData.sample(i).interval.maxValue());
      if (y1 > y2)
        qSwap(y1, y2);

      if (i < (int) iData.size() - 2) {
        const int yy1 = yMap.transform(iData.sample(i + 1).interval.minValue());
        const int yy2 = yMap.transform(iData.sample(i + 1).interval.maxValue());

        if (y2 == qMin(yy1, yy2)) {
          const int xx2 = xMap.transform(iData.sample(i + 1).interval.minValue());
          if (xx2 != x0 && ((xx2 < x0 && x2 < x0) || (xx2 > x0 && x2 > x0))) {
            // One pixel distance between neighbored bars
            y2++;
          }
        }
      }

      painter->setPen(QPen(color(i)));

      drawBar(painter, Qt::Horizontal, QRect(x0, y1, x2 - x0, y2 - y1));
    } else {
      const int y2 = yMap.transform(iData.sample(i).value);
      if (y2 == y0)
        continue;

      int x1 = xMap.transform(iData.sample(i).interval.minValue());
      int x2 = xMap.transform(iData.sample(i).interval.maxValue());
      if (x1 > x2)
        qSwap(x1, x2);

      if (i < (int) iData.size() - 2) {
        const int xx1 = xMap.transform(iData.sample(i + 1).interval.minValue());
        const int xx2 = xMap.transform(iData.sample(i + 1).interval.maxValue());

        if (x2 == qMin(xx1, xx2)) {
          const int yy2 = yMap.transform(iData.sample(i + 1).value);
          if (yy2 != y0 && ((yy2 < y0 && y2 < y0) || (yy2 > y0 && y2 > y0))) {
            // One pixel distance between neighbored bars
            x2--;
          }
        }
      }

      painter->setPen(QPen(color(i)));

      drawBar(painter, Qt::Vertical, QRect(x1, y0, x2 - x1, y2 - y0));
    }
  }
}

void Histogram::drawBar(QPainter *painter, Qt::Orientation, const QRect& rect) const
{
  painter->save();

  const QColor color(painter->pen().color());
  const QRect r = rect.normalized();

  const int factor = 125;
  const QColor light(color.lighter(factor));
  const QColor dark(color.darker(factor));

  painter->setBrush(color);
  painter->setPen(Qt::NoPen);
  QwtPainter::drawRect(painter, r.x() + 1, r.y() + 1, r.width() - 2, r.height() - 2);
  painter->setBrush(Qt::NoBrush);

  painter->setPen(QPen(light, 2));

  QwtPainter::drawLine(painter, r.left() + 1, r.top() + 2, r.right() + 1, r.top() + 2);

  painter->setPen(QPen(dark, 2));

  painter->setPen(QPen(light, 1));

  QwtPainter::drawLine(painter, r.left(), r.top() + 1, r.left(), r.bottom());
  QwtPainter::drawLine(painter, r.left() + 1, r.top() + 2, r.left() + 1, r.bottom() - 1);

  painter->setPen(QPen(dark, 1));

  QwtPainter::drawLine(painter, r.right() + 1, r.top() + 1, r.right() + 1, r.bottom());
  QwtPainter::drawLine(painter, r.right(), r.top() + 2, r.right(), r.bottom() - 1);

  painter->restore();
}

#endif

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



#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <qglobal.h>
#include <QColor>
#include <ACG/Config/ACGDefines.hh>

#include "qwt_plot_item.h" 

class QwtIntervalData;
class QString;

/** \brief Histogram plot
 *
 * This plot can be used to render a plot. It provides the plot management and
 * drawing functions for histograms. The data is supplied as QwtIntervalData
 * via HistogramItem::setData(). Additionally you can set colors for each bar,
 * which are provided via HistogramItem::setColors()
 */
class ACGDLLEXPORT Histogram: public QwtPlotItem
{
public:
    /// Constructor
    explicit Histogram(const QString &title = QString());
    /// Constructor
    explicit Histogram(const QwtText &title);
    /// Destructor
    virtual ~Histogram();

    /** \brief set data to render
     *
     * @param data data provided as QwtIntervalData
     */
    void setData( QwtIntervalSeriesData* data);
    const QwtIntervalSeriesData* data() const;

    /** \brief Set colors
     *
     * @param _colors A vector of colors. One for each interval in the same order.
     */
    void setColors( std::vector< QColor >& _colors);

    /// Function hat will return the datas bounding rectangle (for rendering)
    virtual QRectF boundingRect() const;

    virtual int rtti() const;

    /// The actual draw function, drawing the bars inside the plot widget.
    virtual void draw(QPainter *, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &) const;

    void   setBaseline(double reference);
    double baseline() const;

    enum HistogramAttribute
    {
        Auto = 0,
        Xfy = 1
    };

    void setHistogramAttribute(HistogramAttribute, bool on = true);
    bool testHistogramAttribute(HistogramAttribute) const;

protected:

    /// Draws a single bar
    virtual void drawBar(QPainter *, Qt::Orientation o, const QRect &) const;

private:
    QColor color(uint i) const;

    void init();

    class PrivateData;
    PrivateData *d_data;
};

#endif

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

#pragma once

#include <vector>
#include <cassert>
#include <memory>
#include <exception>
#include <stdexcept>
#include <algorithm>
#include <type_traits>
#include <map>

#include <QString>

#include "SmartPointer.hh"
#include "../Config/ACGDefines.hh"

namespace ACG {
class ACGDLLEXPORT Histogram {
public:
    enum class LabelType {
        PerBin,
        PerBoundary,
    };

    Histogram() = default;
    Histogram(std::vector<size_t> &&bins,
              std::vector<double> &&bin_widths)
        : bins_(std::move(bins)),
          bin_widths_(std::move(bin_widths))
    {}

    virtual ~Histogram() = default;
    const std::vector<size_t> &getBins() const { return bins_; }
    const std::vector<double> &getBinWidths() const { return bin_widths_; }
    virtual double getTotalWidth() const = 0;

    virtual LabelType getLabelType() const = 0;
    virtual QString getBoundaryLabel(size_t /*idx*/) const { assert(false); return QString();}
    virtual QString getBinLabel     (size_t /*idx*/) const { assert(false); return QString();}

protected:
    std::vector<size_t> bins_;
    std::vector<double> bin_widths_;
};


inline QString formatValue (int val) {
    return QString::number(val);
}
inline QString formatValue (unsigned int val) {
    return QString::number(val);
}
inline QString formatValue (double val) {
    return QString::number(val, 'g', 3);
}

template<typename T>
class UnbinnedHistogram : public Histogram
{
public:
    UnbinnedHistogram(std::vector<size_t> &&bin_counts,
              std::vector<T> &&bin_values)
        : Histogram(std::move(bin_counts),
                    std::vector<double>(bin_counts.size(), 1.)),
          bin_values_(std::move(bin_values))
    {
    }
    double getTotalWidth() const override { return bins_.size();};
    LabelType getLabelType() const override { return LabelType::PerBin; };
    QString getBinLabel     (size_t idx) const override { return formatValue(bin_values_[idx]);}
private:
    std::vector<T> bin_values_;
};

template<typename T>
class HistogramT : public Histogram {
public:
    HistogramT() = default;

    HistogramT(std::vector<size_t> &&histogram,
               std::vector<T> &&bin_boundaries,
               std::vector<double> &&bin_widths
               )
        : Histogram(std::move(histogram), std::move(bin_widths)),
          bin_boundaries_(std::move(bin_boundaries))
    {
        if (bins_.size() != bin_widths_.size()
            || bins_.size() + 1 != bin_boundaries_.size()) {
            throw std::runtime_error("Histogram constructor sizes don't match.");
        }
    }

    const std::vector<T> &getBinBoundaries() const {
        return bin_boundaries_;
    }

    double getTotalWidth() const override
    {
        return bin_boundaries_.back() - bin_boundaries_.front();
    }

    LabelType getLabelType() const override
    {
        return LabelType::PerBoundary;
    }

    QString getBoundaryLabel (size_t idx) const override
    {
        // TODO: for floating point types, choose accuracy depending on bin size
        return formatValue(bin_boundaries_[idx]);
    };


private:
    std::vector<T> bin_boundaries_;
};


template<typename T>
std::unique_ptr<Histogram> create_histogram_unbinned(const std::map<T, size_t> &counts)
{
    std::vector<T> values;
    std::vector<size_t> histogram;
    values.reserve(counts.size());
    histogram.reserve(counts.size());
    for (const auto &entry: counts)
    {
        values.push_back(entry.first);
        histogram.push_back(entry.second);
    }
    return ptr::make_unique<UnbinnedHistogram<T>>(std::move(histogram), std::move(values));
}

template<typename T, typename Iterable>
std::unique_ptr<Histogram> create_histogram_autorange(const Iterable &range, size_t max_bins = 50)
{
// we need to be careful with ranges, some sums (e.g. INT_MAX - INT_MIN) do not fit into a signed int,
// so we store bin sizes as doubles. With specialization or some tricks we
// could probably use the next-biggest integer type, but if we're using
// the biggest integer type already, we should to fall back to double anyways.

    using std::begin;
    using std::end;

    std::vector<T> bin_boundaries;
    std::vector<size_t> bins;
    std::vector<double> bin_widths;

    const size_t n = std::distance(begin(range), end(range));
    if (n == 0) return {};
    const auto minmax = std::minmax_element(begin(range), end(range));
    const T min = *minmax.first;
    const T max = *minmax.second;

    const double min_dbl = static_cast<double>(min);
    const double val_range = static_cast<double>(max) - min_dbl;

    const size_t n_bins_max = std::min(max_bins, n);
    bin_boundaries.reserve(n_bins_max + 1);

    T last_boundary = min;
    bin_boundaries.push_back(min);
    for (size_t i = 1; i < n_bins_max; ++i) {
        // Adding val_range/n_bins to a accumulator might seem more efficient/elegant,
        // but might cause numeric issues.

        // This multiplication order is bad for huge ranges that cause overflows,
        // however I assume tiny ranges are more common than huge values and more
        // important to get right. If you disagree, add a case distinction or something better.

        T boundary = static_cast<T>(min + (i * val_range) / n_bins_max);
        // avoid zero-sized bins (happens for many ints with values in a small range)
        if (boundary != last_boundary || i == 0) {
            bin_boundaries.push_back(boundary);
            bin_widths.push_back(boundary - last_boundary);
        }
        last_boundary = boundary;
    }
    bin_boundaries.push_back(max); // avoid rounding issues etc by explicitly picking max.
    bin_widths.push_back(max - last_boundary);

    bin_boundaries.shrink_to_fit();
    size_t n_bins = bin_boundaries.size() - 1;
    bins.resize(n_bins);

    // note that due to rounding, our bins may have differing sizes, which matters
    // if we handle integral types (relative size difference worst case: bin width 1 vs 2).
    // Be careful to select the right bin.
    std::for_each(begin(range), end(range), [&](const T &val) {
        auto it = std::upper_bound(bin_boundaries.begin(), bin_boundaries.end(), val);
        if (it == bin_boundaries.end()) --it; // the last value is exactly max!
        size_t idx = std::distance(bin_boundaries.begin(), it);
        assert(idx > 0);
        ++bins[idx - 1];
    });
    return ptr::make_unique<HistogramT<T>>(std::move(bins), std::move(bin_boundaries), std::move(bin_widths));
}

template<typename Iterable>
std::unique_ptr<Histogram> create_histogram_auto(const Iterable &range, size_t max_bins = 50)
{
    using std::begin;
    using std::end;
    using T = typename std::remove_cv<
                            typename std::remove_reference<
                                decltype(*begin(range))
                            >::type
                        >::type;
    const size_t n = std::distance(begin(range), end(range));
    if (n == 0) return {};

    std::map<T, size_t> elem_counts;
    bool too_many_unique = false;
    for (const auto &v: range)
    {
        ++elem_counts[v];
        if (elem_counts.size() > max_bins)
        {
            too_many_unique = true;
            break;
        }
    }

    if (too_many_unique) {
        return create_histogram_autorange<T>(range, max_bins);
    } else {
        return create_histogram_unbinned(elem_counts);
    }
}

} // namespace ACG


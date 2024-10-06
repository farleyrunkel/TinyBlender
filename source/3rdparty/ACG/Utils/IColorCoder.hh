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

#ifndef ACG_ICOLORCODER_HH
#define ACG_ICOLORCODER_HH

#include <ACG/Math/VectorT.hh>
#include <ACG/Config/ACGDefines.hh>
#include <QColor>
#include <algorithm>

namespace {
// With C++17, we can use std::clamp() instead
template<class T>
const T clamp( const T v, const T lo, const T hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}
}
namespace ACG {

class ACGDLLEXPORT IColorCoder {
public:
    virtual ~IColorCoder() = default;

    // "raw" refers to directly using the value,
    // assuming it to be inside [min(),max()] without any checks,
    // those are implemented in color4() and color_float4()

    virtual ACG::Vec4uc color4_raw(float _v) const = 0;
    virtual ACG::Vec4f  color_float4_raw(float _v) const = 0;

    /// min scalar value
    virtual float min() const = 0;

    /// max scalar value
    virtual float max() const = 0;

    virtual ACG::Vec4uc color4(float _v) const
    {
        auto clamped = clamp(_v, min(), max());
        auto col = color4_raw(clamped);
        if (mapOutsideRangeToAlpha0_ && clamped != _v) {
            col[3] = 0;
        }
        return col;
    }

    virtual ACG::Vec4f  color_float4(float _v) const {
        auto clamped = clamp(_v, min(), max());
        auto col = color_float4_raw(clamped);
        if (mapOutsideRangeToAlpha0_ && clamped != _v) {
            col[3] = 0.0f;
        }
        return col;
    }

    inline ACG::Vec3uc color(float _v) const {
        ACG::Vec4uc c = color4(_v);
        return ACG::Vec3uc(c[0], c[1], c[2]);
    }

    inline ACG::Vec3f color_float(float _v) const
    {
        ACG::Vec4f c = color_float4(_v);
        return ACG::Vec3f(c[0], c[1], c[2]);
    }

    inline QColor color_qcolor(float _v) const
    {
        ACG::Vec4uc c = color4(_v);
        return QColor(c[0], c[1], c[2], c[3]);
    }

    // Make the color coder usable as a function operator.
    inline ACG::Vec4f operator() (float _v) const {
        return color_float4(_v);
    }

    void setMapOutsideRangeToAlpha0(bool value)
    {
        mapOutsideRangeToAlpha0_ = value;
    }

    bool getMapOutsideRangeToAlpha0() const
    {
        return mapOutsideRangeToAlpha0_;
    }

protected:
    bool mapOutsideRangeToAlpha0_ = false;
};





} // namespace ACG

#endif // ICOLORCODER_HH

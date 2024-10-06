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


#include <gtest/gtest.h>

#include <ACG/Utils/Histogram.hh>

#include <numeric>

using ACG::Histogram;
using ACG::HistogramT;

class HistogramTest : public testing::Test {

public:
  HistogramTest()
  {
  }

protected:
  // This function is called before each test is run
  virtual void SetUp() {
  }

  // This function is called after all tests are through
  virtual void TearDown() {
  }
};

TEST_F(HistogramTest, simpleDouble ) {
    std::vector<double> v {1.0, 1.1, 2.0};
    auto hist =  ACG::create_histogram_auto(v,2);

    std::vector<size_t> correct_bins {2, 1};
    EXPECT_EQ(hist->getBins(), correct_bins);


    auto widths = hist->getBinWidths();
    ASSERT_EQ(widths.size(), 2u);
    EXPECT_DOUBLE_EQ(widths[0], 0.5);
    EXPECT_DOUBLE_EQ(widths[1], 0.5);

    double width_sum = std::accumulate(widths.begin(), widths.end(), 0.0);
    double total_width = hist->getTotalWidth();
    EXPECT_DOUBLE_EQ(total_width, 1.0);
    EXPECT_DOUBLE_EQ(total_width, width_sum);

}

#if 0 // not yet implemented
TEST_F(HistogramTest, simpleInt ) {
    std::vector<int> v {0, 1, 1, 2, 5};
    HistogramT<int> hist(v.begin(), v.end(), 3);
    // should be: 0-1, 2-3, 4-5
    //    counts: 3    1    1

    std::vector<size_t> correct_bins {2, 1, 1};
    EXPECT_EQ(hist.getBins(), correct_bins);


    auto widths = hist.getBinWidths();
    ASSERT_EQ(widths.size(), 3);
    EXPECT_EQ(widths[0], 2);
    EXPECT_EQ(widths[1], 2);
    EXPECT_EQ(widths[2], 2);

    int width_sum = std::accumulate(widths.begin(), widths.end(), 0.0);
    int total_width = hist.getTotalWidth();
    EXPECT_EQ(total_width, 6);
    EXPECT_EQ(total_width, width_sum);

    std::vector<int> correct_boundaries {0, 2, 4, 5};
    auto boundaries = hist.getBinBoundaries();
    ASSERT_EQ(boundaries, correct_boundaries);
}
#endif

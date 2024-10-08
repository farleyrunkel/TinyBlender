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

#include <ACG/Math/GLMatrixT.hh>


class MathTest : public testing::Test {

public:
  MathTest()
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

TEST_F(MathTest, GLMatrixT_extract_planes ) {
  
  const double cmp_eps = 1e-7;

  const double near = 10.0;
  const double far = 25.0;

  // test clipping plane extraction on perspective and orthographic matrix
  ACG::GLMatrixd P;
  P.identity();

  EXPECT_FALSE(P.isPerspective());
  EXPECT_FALSE(P.isOrtho());

  P.perspective(45.0, 4.0/3.0, near, far);

  ACG::Vec2d planes = P.extract_planes_perspective();

  EXPECT_LE(fabs(planes[0] - near), cmp_eps);
  EXPECT_LE(fabs(planes[1] - far), cmp_eps);

  EXPECT_TRUE(P.isPerspective());
  EXPECT_FALSE(P.isOrtho());

  // test on orthographic matrix

  P.identity();
  P.ortho(-1.0, 1.0, -1.0, 1.0, near, far);

  planes = P.extract_planes_ortho();

  EXPECT_LE(fabs(planes[0] - near), cmp_eps);
  EXPECT_LE(fabs(planes[1] - far), cmp_eps);

  EXPECT_FALSE(P.isPerspective());
  EXPECT_TRUE(P.isOrtho());
}

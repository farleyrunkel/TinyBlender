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

#include <ACG/Math/VectorT.hh>
#include <ACG/Geometry/Algorithms.hh>

class ALGORITHM_TEST_BASE : public testing::Test {

    protected:

        // This function is called before each test is run
        virtual void SetUp() {

        }

        // This function is called after all tests are through
        virtual void TearDown() {

            // Do some final stuff with the member data here...
        }

};

TEST_F(ALGORITHM_TEST_BASE, triangleIntersection ) {


  // ==============================================
  // Triangle intersection algorithm
  // ==============================================

  /* All in z = 0 plane :

   (0,1)
    p2
    | \
    |  \
    |   \
    |    \
    |     \
    |      \
    p0 === p1
   (0,0)   (1,0)
   */

  // Triangle points
  ACG::Vec3f p0 ( 0.0,0.0,0.0);
  ACG::Vec3f p1 ( 1.0,0.0,0.0);
  ACG::Vec3f p2 ( 0.0,1.0,0.0);

  // Shooting ray origin and direction
  ACG::Vec3f origin( 0.1f, 0.1f, -1.0f);
  ACG::Vec3f direction( 0.0f, 0.0f, 1.0f);

  float distance,u,v;
  bool result = ACG::Geometry::triangleIntersection(origin, direction,
                                                    p0, p1, p2,
                                                    distance,u,v);

  EXPECT_TRUE( result )       << "Intersection failed!";

  EXPECT_EQ( 1.0f, distance ) << "Wrong distance!" << std::endl;
  EXPECT_EQ( 0.1f, u )        << "Wrong u!" << std::endl;
  EXPECT_EQ( 0.1f, v )        << "Wrong v!" << std::endl;

}

TEST_F(ALGORITHM_TEST_BASE, triangleIntersection_FlippedTriangleOrientation ) {


  // ==============================================
  // Triangle intersection algorithm
  // ==============================================

  /* All in z = 0 plane :

   (0,1)
    p2
    | \
    |  \
    |   \
    |    \
    |     \
    |      \
    p0 === p1
   (0,0)   (1,0)
   */

  // Triangle points
  ACG::Vec3f p0 ( 0.0,0.0,0.0);
  ACG::Vec3f p1 ( 1.0,0.0,0.0);
  ACG::Vec3f p2 ( 0.0,1.0,0.0);

  // Shooting ray origin and direction
  ACG::Vec3f origin( 0.1f, 0.1f, -1.0f);
  ACG::Vec3f direction( 0.0, 0.0, 1.0);

  float distance,u,v;
  bool result = ACG::Geometry::triangleIntersection(origin, direction,
                                                    p0, p2, p1,
                                                    distance,u,v);

  EXPECT_TRUE( result )       << "Intersection failed!";

  EXPECT_EQ( 1.0f, distance ) << "Wrong distance!" << std::endl;
  EXPECT_EQ( 0.1f, u )        << "Wrong u!" << std::endl;
  EXPECT_EQ( 0.1f, v )        << "Wrong v!" << std::endl;

}

TEST_F(ALGORITHM_TEST_BASE, triangleIntersection_NegativeShootingDirection ) {


  // ==============================================
  // Triangle intersection algorithm
  // ==============================================

  /* All in z = 0 plane :

   (0,1)
    p2
    | \
    |  \
    |   \
    |    \
    |     \
    |      \
    p0 === p1
   (0,0)   (1,0)
   */

  // Triangle points
  ACG::Vec3f p0 ( 0.0,0.0,0.0);
  ACG::Vec3f p1 ( 1.0,0.0,0.0);
  ACG::Vec3f p2 ( 0.0,1.0,0.0);

  // Shooting ray origin and direction
  ACG::Vec3f origin( 0.1f, 0.1f, -1.0f);
  ACG::Vec3f direction( 0.0f, 0.0f, -1.0f);

  float distance,u,v;
  bool result = ACG::Geometry::triangleIntersection(origin, direction,
                                                    p0, p1, p2,
                                                    distance,u,v);

  EXPECT_TRUE( result )        << "Intersection failed!";

  EXPECT_EQ( -1.0f, distance ) << "Wrong distance!" << std::endl;
  EXPECT_EQ( 0.1f, u )         << "Wrong u!" << std::endl;
  EXPECT_EQ( 0.1f, v )         << "Wrong v!" << std::endl;

}

TEST_F(ALGORITHM_TEST_BASE, triangleIntersection_NegativeShootingDirection_FlippedTriangleOrientation ) {


  // ==============================================
  // Triangle intersection algorithm
  // ==============================================

  /* All in z = 0 plane :

   (0,1)
    p2
    | \
    |  \
    |   \
    |    \
    |     \
    |      \
    p0 === p1
   (0,0)   (1,0)
   */

  // Triangle points
  ACG::Vec3f p0 ( 0.0,0.0,0.0);
  ACG::Vec3f p1 ( 1.0,0.0,0.0);
  ACG::Vec3f p2 ( 0.0,1.0,0.0);

  // Shooting ray origin and direction
  ACG::Vec3f origin( 0.1f, 0.1f, -1.0f);
  ACG::Vec3f direction( 0.0f, 0.0f, -1.0f);

  float distance,u,v;
  bool result = ACG::Geometry::triangleIntersection(origin, direction,
                                                    p0, p2, p1,
                                                    distance,u,v);

  EXPECT_TRUE( result )        << "Intersection failed!";

  EXPECT_EQ( -1.0f, distance ) << "Wrong distance!" << std::endl;
  EXPECT_EQ( 0.1f, u )         << "Wrong u!" << std::endl;
  EXPECT_EQ( 0.1f, v )         << "Wrong v!" << std::endl;

}





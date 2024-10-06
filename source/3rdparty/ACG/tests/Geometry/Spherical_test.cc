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
#include <ACG/Math/GLMatrixT.hh>
#include <ACG/Geometry/Spherical.hh>

namespace {

using ACG::Vec3d;

inline Vec3d rot(const Vec3d &ref, const Vec3d &normal, double angle) {
    ACG::GLMatrixT<double> rmat;
    rmat.identity();
    rmat.rotate(angle * M_1_PI * 180.0, normal, ACG::MULT_FROM_LEFT);
    return rmat.transform_vector(ref);
}

class Spherical : public testing::Test {

    protected:

        virtual void SetUp() {
        }

        virtual void TearDown() {
        }

};

TEST_F(Spherical, sphericalInnerAngleSum_zeroTriangle) {
    {
        const Vec3d n1(1, 0, 0);
        EXPECT_NEAR(M_PI, ACG::Geometry::sphericalInnerAngleSum(n1, n1, n1), 1e-6);
    }

    /*
     * Jitter along one axis.
     */
    {
        const Vec3d n1(1, 0, 0);
        const Vec3d axis = (n1 % Vec3d(3, 1, 2).normalized()).normalized();
        EXPECT_NEAR(M_PI, ACG::Geometry::sphericalInnerAngleSum(
                rot(n1, axis, .1), n1, n1), 1e-6);
        EXPECT_NEAR(M_PI, ACG::Geometry::sphericalInnerAngleSum(
                n1, rot(n1, axis, .1), n1), 1e-6);
        EXPECT_NEAR(M_PI, ACG::Geometry::sphericalInnerAngleSum(
                n1, n1, rot(n1, axis, .1)), 1e-6);
        EXPECT_NEAR(M_PI, ACG::Geometry::sphericalInnerAngleSum(
                rot(n1, axis, .1), rot(n1, axis, -.05), rot(n1, axis, .07)), 1e-6);
    }

    {
        const Vec3d n1 = Vec3d(4, 5, 6).normalized();
        const Vec3d axis = (n1 % Vec3d(3, 1, 2).normalized()).normalized();
        EXPECT_NEAR(M_PI, ACG::Geometry::sphericalInnerAngleSum(
                rot(n1, axis, .1), rot(n1, axis, -.05), rot(n1, axis, .07)), 1e-6);
    }
}

TEST_F(Spherical, sphericalPolyhedralGaussCurv_pointPolyhedral) {
    std::vector<Vec3d> normals;
    normals.push_back(Vec3d(1, 0, 0));
    normals.push_back(Vec3d(1, 0, 0));
    normals.push_back(Vec3d(1, 0, 0));
    normals.push_back(Vec3d(1, 0, 0));

    EXPECT_NEAR(0, ACG::Geometry::sphericalPolyhedralGaussCurv<Vec3d>(normals.begin(), normals.end()), 1e-6);

    const Vec3d v = Vec3d(3, 2, 7).normalized();
    normals.clear();
    for (int i = 0; i < 7; ++i) normals.push_back(v);

    EXPECT_NEAR(0, ACG::Geometry::sphericalPolyhedralGaussCurv<Vec3d>(normals.begin(), normals.end()), 1e-6);
}

TEST_F(Spherical, sphericalPolyhedralGaussCurv_linePolyhedral) {
    /*
     * Jitter along one axis.
     */
    const Vec3d n1 = Vec3d(4, 5, 6).normalized();
    const Vec3d axis = (n1 % Vec3d(3, 1, 2)).normalized();

    std::vector<Vec3d> normals;
    normals.push_back(rot(n1, axis, .1));
    normals.push_back(rot(n1, axis, .2));
    normals.push_back(rot(n1, axis, .05));
    normals.push_back(rot(n1, axis, .09));
    normals.push_back(rot(n1, axis, -.2));
    normals.push_back(rot(n1, axis, .01));
    normals.push_back(rot(n1, axis, -.1));
    normals.push_back(rot(n1, axis, -.2));

    EXPECT_NEAR(0, ACG::Geometry::sphericalPolyhedralGaussCurv<Vec3d>(normals.begin(), normals.end()), 1e-6);
}

TEST_F(Spherical, sphericalPolyhedralGaussCurv_cubeCorner) {
    {
        std::vector<Vec3d> normals;
        normals.push_back(Vec3d(1, 0, 0));
        normals.push_back(Vec3d(0, 1, 0));
        normals.push_back(Vec3d(0, 0, 1));

        EXPECT_NEAR(M_PI_2, ACG::Geometry::sphericalPolyhedralGaussCurv<Vec3d>(normals.begin(), normals.end()), 1e-6);
    }
}


TEST_F(Spherical, sphericalPolyhedralGaussCurv_houseCorner) {
    std::vector<Vec3d> normals;
    normals.push_back(Vec3d(0, 0, 1));
    normals.push_back(Vec3d(0, 1, 0));
    normals.push_back(Vec3d(0, 1, 0));
    normals.push_back(Vec3d(0, 1, 0));
    normals.push_back(Vec3d(1, 0, 0));

    EXPECT_NEAR(-M_PI_2, ACG::Geometry::sphericalPolyhedralGaussCurv<Vec3d>(normals.begin(), normals.end()), 1e-6);
}

} /* namespace */

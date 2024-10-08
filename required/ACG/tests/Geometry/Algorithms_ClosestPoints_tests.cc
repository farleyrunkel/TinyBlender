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

#include "../Math/MatrixTestHelper.hh"

namespace {

TEST(Algorithms_ClosestPoints, closestPointTri_inside) {
    // Closest point inside
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(0.180189728737, 0.284434795380, 0.685222446918),
                ACG::Vec3d(0.780881822109, -0.711405277252, 0.000000000000),
                ACG::Vec3d(-1.080165147781, 0.470909714699, 0.000000000000),
                ACG::Vec3d(0.663306236267, 1.208429455757, 0.000000000000)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(0.180189728737, 0.284434795380, 0.000000000000)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(0.180189728737, 0.284434795380, 0.685222446918),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(0.379620872542, 0.129005603666, 0.039932640059)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(0.383401751518, -0.044320285320, -0.729106009007),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(0.165861088939, 0.125222789570, -0.025220099004)));
    }
}

TEST(Algorithms_ClosestPoints, closestPointTri_onEdge) {
    // Closest point on edges.
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(2.194746255875, 0.654435634613, 1.605020284653),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(0.826052171821, 0.158427751505, 0.170818395715)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(-1.639497995377, -1.687432765961, 0.645231306553),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-0.312445889897, -0.008722876212, -0.140780953244)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(-0.954960584641, 3.141639709473, -0.399959266186),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-0.091698979114, 0.882223932862, -0.287157561834)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(-0.326786577702, -0.595147013664, -0.012022850104),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-0.062953975601, -0.153517634035, -0.028797485905)));
    }
}

TEST(Algorithms_ClosestPoints, closestPointTri_onCorner) {
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(-1.840421676636, 0.351687341928, -1.691396117210),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(1.548038601875, 1.783731102943, -0.995657980442),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(1.674694657326, -1.574745774269, -0.140889823437),
                ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407),
                ACG::Vec3d(-1.138887882233, 0.470909774303, -0.511726200581),
                ACG::Vec3d(0.738806843758, 1.208429455757, -0.109056398273)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(0.898327171803, -0.711405336857, 0.402669787407)));
    }
}

TEST(Algorithms_ClosestPoints, closestPointTri_degenerate) {
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(0.379400968552, -0.866840600967, -1.375681400299),
                ACG::Vec3d(-2.827600002289, -1.685648560524, -0.758169531822),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070630312),
                ACG::Vec3d(-0.661723732948, 1.627503752708, -0.181971758604)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-1.568279320469, 0.240740866643, -0.423146806300)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(0.842265009880, 3.628707885742, -0.395066857338),
                ACG::Vec3d(-2.827600002289, -1.685648560524, -0.758169531822),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070630312),
                ACG::Vec3d(-0.661723732948, 1.627503752708, -0.181971758604)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-0.661723732948, 1.627503752708, -0.181971758604)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(0.842265009880, 3.628707885742, -0.395066857338),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070600510),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070630312),
                ACG::Vec3d(-0.661723732948, 1.627503752708, -0.181971758604)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-0.661723732948, 1.627503752708, -0.181971758604)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(-3.731231689453, -2.141752481461, -0.150896072388),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070600510),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070630312),
                ACG::Vec3d(-0.661723732948, 1.627503752708, -0.181971758604)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070600510)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(-0.966588973999, 0.534245491028, 0.345366060734),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070600510),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070630312),
                ACG::Vec3d(-0.661723732948, 1.627503752708, -0.181971758604)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-1.200293249127, 0.803651109932, -0.325249806970)));
    }
    {
        ACG::Vec3d closest_point = ACG::Geometry::closestPointTri(
                ACG::Vec3d(-0.966588973999, 0.534245491028, 0.345366060734),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070600510),
                ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070630312),
                ACG::Vec3d(-1.744661808014, -0.029072403908, -0.470070600510)
        );
        EXPECT_TRUE(areClose(closest_point, ACG::Vec3d(-1.744661808014, -0.029072359204, -0.470070600510)));
    }
}

} /* namespace */

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

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <ACG/Geometry/bsp/TriangleBSPT.hh>
#include <ACG/Geometry/Algorithms.hh>


struct CustomTraits : public OpenMesh::DefaultTraits {
};

typedef OpenMesh::TriMesh_ArrayKernelT<CustomTraits> Mesh;

typedef OpenMeshTriangleBSPT< Mesh > BSP;

class BSP_CUBE_BASE : public testing::Test {

    protected:

        // This function is called before each test is run
        virtual void SetUp() {

          mesh_.clear();

            // Add some vertices
            Mesh::VertexHandle vhandle[8];
            vhandle[0] = mesh_.add_vertex(Mesh::Point(-1, -1,  1));
            vhandle[1] = mesh_.add_vertex(Mesh::Point( 1, -1,  1));
            vhandle[2] = mesh_.add_vertex(Mesh::Point( 1,  1,  1));
            vhandle[3] = mesh_.add_vertex(Mesh::Point(-1,  1,  1));
            vhandle[4] = mesh_.add_vertex(Mesh::Point(-1, -1, -1));
            vhandle[5] = mesh_.add_vertex(Mesh::Point( 1, -1, -1));
            vhandle[6] = mesh_.add_vertex(Mesh::Point( 1,  1, -1));
            vhandle[7] = mesh_.add_vertex(Mesh::Point(-1,  1, -1));

            // Add six faces to form a cube
            std::vector<Mesh::VertexHandle> face_vhandles;

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[0]);
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[3]);
            mesh_.add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[2]);
            face_vhandles.push_back(vhandle[3]);
            mesh_.add_face(face_vhandles);

            //=======================

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[7]);
            face_vhandles.push_back(vhandle[6]);
            face_vhandles.push_back(vhandle[5]);
            mesh_.add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[7]);
            face_vhandles.push_back(vhandle[5]);
            face_vhandles.push_back(vhandle[4]);
            mesh_.add_face(face_vhandles);

            //=======================

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[0]);
            face_vhandles.push_back(vhandle[4]);
            mesh_.add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[4]);
            face_vhandles.push_back(vhandle[5]);
            mesh_.add_face(face_vhandles);

            //=======================

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[2]);
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[5]);
            mesh_.add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[2]);
            face_vhandles.push_back(vhandle[5]);
            face_vhandles.push_back(vhandle[6]);
            mesh_.add_face(face_vhandles);


            //=======================

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[3]);
            face_vhandles.push_back(vhandle[2]);
            face_vhandles.push_back(vhandle[6]);
            mesh_.add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[3]);
            face_vhandles.push_back(vhandle[6]);
            face_vhandles.push_back(vhandle[7]);
            mesh_.add_face(face_vhandles);

            //=======================

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[0]);
            face_vhandles.push_back(vhandle[3]);
            face_vhandles.push_back(vhandle[7]);
            mesh_.add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[0]);
            face_vhandles.push_back(vhandle[7]);
            face_vhandles.push_back(vhandle[4]);
            mesh_.add_face(face_vhandles);


            // Test setup:
            //
            //
            //      3 ======== 2
            //     / \        /
            //    /   \      / |
            //   /     \    /  |
            //  /       \  /   |
            // 0 ======== 1    |
            // |       /  |    |
            // |      /   |    6    z
            // |     /    |   /     |   y
            // |    /     |  /      |  /
            // |   /      | /       | /
            // |  /       |/        |/
            // 4 ======== 5         -------> x
            //


            //      x ======== x
            //     / \        /
            //    /   \  1   / |
            //   /  0  \    /  |
            //  /       \  /   |
            // x ======== x    |
            // |       /  |    |
            // |  4   /   |    x    z
            // |     /    |   /     |   y
            // |    /  5  |  /      |  /
            // |   /      | /       | /
            // |  /       |/        |/
            // x ======== x         -------> x
            //

            // create Triangle BSP
            bsp_ = new BSP( mesh_ );

            // build Triangle BSP
            bsp_->reserve(mesh_.n_faces());

            Mesh::FIter f_it  = mesh_.faces_begin();
            Mesh::FIter f_end = mesh_.faces_end();

            for (; f_it!=f_end; ++f_it)
              bsp_->push_back(*f_it);

            bsp_->build(10, 100); //max vertices per leaf 10, max depth 100

        }

        // This function is called after all tests are through
        virtual void TearDown() {

            // Do some final stuff with the member data here...
        }

    // This member will be accessible in all tests
    Mesh mesh_;

    //This will be the tree
    BSP* bsp_;
};


/* Basic check if the test mesh is setup correctly
 */
TEST_F(BSP_CUBE_BASE, CheckCubeConstruction) {

  // Check setup of the mesh
  EXPECT_EQ(8u, mesh_.n_vertices() ) << "Wrong number of vertices";
  EXPECT_EQ(12u, mesh_.n_faces() )   << "Wrong number of faces";

}

/* Basic check if the bsp can be setup
 */
TEST_F(BSP_CUBE_BASE, SetupBSPfromCube ) {

  EXPECT_FALSE(bsp_->empty()) << "BSP should not be empty !";
  EXPECT_EQ(12u, bsp_->size()) << "Wrong number of triangles in BSP after construction";

}

/* Nearest neighbor check
 */
TEST_F(BSP_CUBE_BASE, NearestNeighbour_OnSurface_1 ) {

  // (-1,1)      (1,1)
  //    x ======== x
  //    |       /  |             y = -1 plane
  //    |  4   /   |        z
  //    |     /    |        |
  //    |    /  5  |        |
  //    |   /      |        |
  //    |  /       |        |
  //    x ======== x         -------> x
  // (-1,-1)     (1,-1)


  // Point close to face 5
  // (-1,1)      (1,1)
  //    x ======== x
  //    |       /  |
  //    |      /   |
  //    |     /    |
  //    |    /  p  |
  //    |   /      |
  //    |  /       |
  //    x ======== x
  // (-1,-1)     (1,-1)
  Mesh::Point p1(0.75,-1.0,0.0);
  BSP::NearestNeighbor nn = bsp_->nearest(p1);

  EXPECT_EQ(5, nn.handle.idx()) << "Wrong Handle on closest face";
}

TEST_F(BSP_CUBE_BASE, NearestNeighbour_OnSurface_2 ) {
  // Point close to face 4
  // (-1,1)      (1,1)
  //    x ======== x
  //    |       /  |
  //    |      /   |
  //    |     /    |
  //    | p  /     |
  //    |   /      |
  //    |  /       |
  //    x ======== x
  // (-1,-1)     (1,-1)
  Mesh::Point p1(-0.75,-1.0,0.0);
  BSP::NearestNeighbor nn = bsp_->nearest(p1);
  EXPECT_EQ(4, nn.handle.idx()) << "Wrong Handle on closest face";
}

TEST_F(BSP_CUBE_BASE, NearestNeighbour_OnSurface_3 ) {
  // Point close to face 4
  // (-1,1)      (1,1)
  //    x ======== x
  //    | p     /  |
  //    |      /   |
  //    |     /    |
  //    |    /     |
  //    |   /      |
  //    |  /       |
  //    x ======== x
  // (-1,-1)     (1,-1)
  Mesh::Point p1(-0.75,-1.0,0.75);
  BSP::NearestNeighbor nn = bsp_->nearest(p1);
  EXPECT_EQ(4, nn.handle.idx()) << "Wrong Handle on closest face";
}

TEST_F(BSP_CUBE_BASE, NearestNeighbour_OnSurface_4 ) {

  // Point close to face 5
  // (-1,1)      (1,1)
  //    x ======== x
  //    |       /  |
  //    |      /   |
  //    |     /    |
  //    |    /     |
  //    |   /      |
  //    |  /     p |
  //    x ======== x
  // (-1,-1)     (1,-1)
  Mesh::Point p1(0.75,-1.0,-0.75);
  BSP::NearestNeighbor nn = bsp_->nearest(p1);
  EXPECT_EQ(5, nn.handle.idx()) << "Wrong Handle on closest face";

}

TEST_F(BSP_CUBE_BASE, RayIntersectionAboveSurface_NonDirectionalFunction_1 ) {


  // ==============================================
  // Test some Rays
  // ==============================================

  // Shoot through face 4 in y direction
  // Start point is not on face 4
  //      x ======== x
  //     / \        /
  //    /   \  1   / |
  //   /  0  \    /  |
  //  /       \  /   |
  // x ======== x    |
  // |       /  |    |
  // |  4   /   |    x    z
  // |     /    |   /     |   y
  // | p  /  5  |  /      |  /
  // |   /      | /       | /
  // |  /       |/        |/
  // x ======== x         -------> x

  Mesh::Point yDirection(0.0,1.0,0.0);
  Mesh::Point p1(-0.5,-2.0,0.0);
  BSP::RayCollision rc;
  rc = bsp_->raycollision(p1,yDirection);

  EXPECT_EQ(2u, rc.size() ) << "Wrong number of hit faces in ray collision test 1";
  if ( rc.size() == 2u ) { // Don't crash on wrong size
    EXPECT_EQ(4, rc[0].first.idx() )         << "Wrong handle of first face in ray collision test 1";
    EXPECT_EQ(9, rc[1].first.idx() )         << "Wrong handle of second face in ray collision test 1";
  }

}

TEST_F(BSP_CUBE_BASE, RayIntersectionAboveSurface_NonDirectionalFunction_NegativeDirection_1 ) {
  // ==============================================
  // Test some Rays
  // ==============================================

  // Shoot through face 4 in negative y direction
  // Start point is not on face 4
  //      x ======== x
  //     / \        /
  //    /   \  1   / |
  //   /  0  \    /  |
  //  /       \  /   |
  // x ======== x    |
  // |       /  |    |
  // |  4   /   |    x    z
  // |     /    |   /     |   y
  // | p  /  5  |  /      |  /
  // |   /      | /       | /
  // |  /       |/        |/
  // x ======== x         -------> x

  Mesh::Point nyDirection(0.0,-1.0,0.0);
  Mesh::Point p1(-0.5,-2.0,0.0);
  BSP::RayCollision rc;
  rc = bsp_->raycollision(p1,nyDirection);

  EXPECT_EQ(2u, rc.size() )       << "Wrong number of hit faces in ray collision test 1";
  if ( rc.size() == 2u ) { // Don't crash on wrong size
    EXPECT_EQ(4, rc[0].first.idx() )    << "Wrong handle of first face in ray collision test 1";
    EXPECT_EQ(9, rc[1].first.idx() )    << "Wrong handle of second face in ray collision test 1";
  }
}

TEST_F(BSP_CUBE_BASE, RayIntersectionAboveSurface_NonDirectionalFunction_2 ) {
  // Shoot through face 5 in y direction
  // Start point is not on face 5
  //      x ======== x
  //     / \        /
  //    /   \  1   / |
  //   /  0  \    /  |
  //  /       \  /   |
  // x ======== x    |
  // |       /  |    |
  // |  4   /   |    x    z
  // |     /    |   /     |   y
  // | p  /  5  |  /      |  /
  // |   /      | /       | /
  // |  /       |/        |/
  // x ======== x         -------> x

  Mesh::Point yDirection(0.0,1.0,0.0);
  Mesh::Point p1(0.5,-2.0,0.0);
  BSP::RayCollision rc;
  rc = bsp_->raycollision(p1,yDirection);

  EXPECT_EQ(2u, rc.size() )     << "Wrong number of hit faces in ray collision test 2";
  if ( rc.size() == 2u ) { // Don't crash on wrong size
    EXPECT_EQ(5, rc[0].first.idx() )  << "Wrong handle of first face in ray collision test 2";
    EXPECT_EQ(8, rc[1].first.idx() )  << "Wrong handle of second face in ray collision test 2";
  }

}

TEST_F(BSP_CUBE_BASE, RayIntersectionAboveSurface_NonDirectionalFunction_NegativeDirection_2 ) {

  // Shoot through face 5 in negative y direction
  // Start point is not on face 5
  //      x ======== x
  //     / \        /
  //    /   \  1   / |
  //   /  0  \    /  |
  //  /       \  /   |
  // x ======== x    |
  // |       /  |    |
  // |  4   /   |    x    z
  // |     /    |   /     |   y
  // | p  /  5  |  /      |  /
  // |   /      | /       | /
  // |  /       |/        |/
  // x ======== x         -------> x

  Mesh::Point nyDirection(0.0,-1.0,0.0);
  Mesh::Point p1(0.5,-2.0,0.0);
  BSP::RayCollision rc;
  rc = bsp_->raycollision(p1,nyDirection);

  EXPECT_EQ(2u, rc.size() )      << "Wrong number of hit faces in ray collision test 2";
  if ( rc.size() == 2u ) { // Don't crash on wrong size
    EXPECT_EQ(5, rc[0].first.idx() )   << "Wrong handle of first face in ray collision test 2";
    EXPECT_EQ(8, rc[1].first.idx() )   << "Wrong handle of second face in ray collision test 2";
  }

}



TEST_F(BSP_CUBE_BASE, RayIntersectionAboveSurface_DirectionalFunction_1 ) {


  // ==============================================
  // Test some Rays
  // ==============================================

  // Shoot through face 4 in y direction
  // Start point is not on face 4
  //      x ======== x
  //     / \        /
  //    /   \  1   / |
  //   /  0  \    /  |
  //  /       \  /   |
  // x ======== x    |
  // |       /  |    |
  // |  4   /   |    x    z
  // |     /    |   /     |   y
  // | p  /  5  |  /      |  /
  // |   /      | /       | /
  // |  /       |/        |/
  // x ======== x         -------> x

  Mesh::Point yDirection(0.0,1.0,0.0);
  Mesh::Point origin(-0.5,-2.0,0.0);
  BSP::RayCollision rc;
  rc = bsp_->directionalRaycollision(origin,yDirection);

  EXPECT_EQ(2u, rc.size() )      << "Wrong number of hit faces in ray collision test 1";
  if ( rc.size() == 2u ) { // Don't crash on wrong size
    EXPECT_EQ(4, rc[0].first.idx() )   << "Wrong handle of first face in ray collision test 1";
    EXPECT_EQ(9, rc[1].first.idx() )   << "Wrong handle of second face in ray collision test 1";


    // Some intersection test to see, if we really have something usefull here:
    Mesh::FaceVertexIter fv_it =  Mesh::FaceVertexIter(mesh_, rc[0].first);

    float distance,u,v;
    Mesh::Point p1 = mesh_.point(*fv_it);
    Mesh::Point p2 = mesh_.point(*(++fv_it));
    Mesh::Point p3 = mesh_.point(*(++fv_it));

    ACG::Geometry::triangleIntersection(origin,
                                        yDirection,
                                        p1,
                                        p2,
                                        p3,
                                        distance,
                                        u,
                                        v);

    EXPECT_EQ(1.0f, distance ) << "Wrong distance";
    EXPECT_EQ(0.25f, u )       << "Wrong u";
    EXPECT_EQ(0.5f , v )       << "Wrong v";

  }

}

TEST_F(BSP_CUBE_BASE, RayIntersectionAboveSurface_DirectionalFunction_NegativeDirection_1 ) {
  // ==============================================
  // Test some Rays
  // ==============================================

  // Shoot through face 4 in negative y direction
  // Start point is not on face 4
  //      x ======== x
  //     / \        /
  //    /   \  1   / |
  //   /  0  \    /  |
  //  /       \  /   |
  // x ======== x    |
  // |       /  |    |
  // |  4   /   |    x    z
  // |     /    |   /     |   y
  // | p  /  5  |  /      |  /
  // |   /      | /       | /
  // |  /       |/        |/
  // x ======== x         -------> x

  Mesh::Point nyDirection(0.0,-1.0,0.0);
  Mesh::Point p1(-0.5,-2.0,0.0);
  BSP::RayCollision rc;
  rc = bsp_->directionalRaycollision(p1,nyDirection);

  EXPECT_EQ(0u, rc.size() ) << "Wrong number of hit faces in ray collision test 1";

}


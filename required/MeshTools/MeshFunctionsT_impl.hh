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






//=============================================================================
//
//  IMPLEMENTATION
//
//=============================================================================

#define MESHFUNCTIONS_C

//== INCLUDES =================================================================

#include "MeshFunctions.hh"

#include <ACG/Geometry/Algorithms.hh>
#include "Math_Tools/Math_Tools.hh"

#include <set>

#include <iostream>

#include <OpenMesh/Core/Geometry/MathDefs.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>

//== NAMESPACES ===============================================================

namespace MeshFunctions {

//== IMPLEMENTATION ==========================================================

template < typename MeshT , typename VectorT >
bool get_boundary(MeshT& _mesh, 
                               typename MeshT::VertexHandle _vh, 
                               std::vector< std::pair< VectorT , typename MeshT::VertexHandle > >& _boundary) 
{
   _boundary.clear();
    typename MeshT::VertexHandle last  = _vh;
    const typename MeshT::VertexHandle begin = _vh;
    
    std::set< typename MeshT::VertexHandle > set;
    
    //walk along boundary
    do  {
        //insert current Vertex (which is on boundary to set (needed to detect loops)
        set.insert( _vh );
        
        // store the current vertex in the boundary list
        _boundary.push_back(std::pair< VectorT , typename MeshT::VertexHandle > ( (VectorT)_mesh.point(_vh) , _vh )  );
        
        // iterate over all outgoing halfedges of the current vertex to find next one
        for (typename MeshT::VertexOHalfedgeIter vohe_it(_mesh,_vh); vohe_it.is_valid() ; ++vohe_it) {
        
          //if vertex is on the boundary use it as the next one (if its not the one, we are comming from)
          if ( _mesh.is_boundary(*vohe_it) && ( _mesh.to_vertex_handle(*vohe_it) != last ) ) {
            last = _vh;
            _vh = _mesh.to_vertex_handle(*vohe_it);
            break;
           }
        }
     // stop if we find a vertex which is already in the list (can also detect partial loops)
     } while ( set.count( _vh ) == 0 );
     
     if ( begin != _vh ) {
        std::cout << "Warning in ( GeometryFunctions.cc get_boundary ) : boundary loop may be partial ( start- != endpoint ) " << std::endl;
     }
     
     return true;
} 

template < typename MeshT , typename VectorT >
bool get_boundary(MeshT& _mesh, 
                               std::vector< std::pair< VectorT , typename MeshT::VertexHandle > >& _boundary) 
{
   typename MeshT::VertexHandle vh;

   // Search for one Vertex on boundary
   bool found = false;
   for (typename MeshT::HalfedgeIter he_it=_mesh.halfedges_begin() ; he_it != _mesh.halfedges_end() ; ++he_it) {
      if ( _mesh.is_boundary(he_it) ) {
         vh = _mesh.from_vertex_handle(he_it);
         found = true;
         break;
      }
   }

   if ( found ) 
      return get_boundary(_mesh , vh , _boundary);
   else {
      std::cerr << "Did not find Mesh boundary!! ( GeometryFunctions.cc get_boundary )" << std::endl;
      return false;
   }
}

template < typename MeshT , typename VectorT >
void smooth_boundary(MeshT& _mesh, 
                                      typename MeshT::VertexHandle _vh)
{
     std::vector< std::pair< VectorT , typename MeshT::VertexHandle > > boundary;
     
     //get the boundary
     get_boundary (_mesh , _vh , boundary); 

     for (uint i = 1 ; i <= boundary.size() ; ++i ) {
        _mesh.point( boundary[ i % boundary.size() ].second ) = ( boundary[ ( i - 1) % boundary.size() ].first  
                                                                + boundary[ ( i ) % boundary.size() ].first   * 2.0
                                                                + boundary[ ( i + 1) % boundary.size() ].first  ) * 0.25; 
        
     }
}

template < typename MeshT >
bool neighbour(const MeshT& _mesh , 
               const typename MeshT::FaceHandle& _fh1 ,
               const typename MeshT::FaceHandle& _fh2 )
{
  for ( typename MeshT::FaceFaceIter ff_it(_mesh,_fh1) ; ff_it ; ++ff_it)
     if (*ff_it == _fh2)
         return true;
  return false;
}


template <class MeshT , typename VectorT >
bool
cut_face(const VectorT& _porigin, 
               const VectorT& _pnormal, 
               const MeshT& _mesh, 
               const typename MeshT::FaceHandle& _fh)
{
  typename MeshT::ConstFaceVertexIter fv_it = _mesh.cfv_iter(_fh);
  const VectorT p0 = _mesh.point(  fv_it);
  const VectorT p1 = _mesh.point(++fv_it);
  const VectorT p2 = _mesh.point(++fv_it);

  unsigned int npos(0), nneg(0);

  if( ACG::Geometry::distPointPlane< VectorT , double >( _porigin, _pnormal, p0) > 0 ) ++npos; else ++nneg;
  if( ACG::Geometry::distPointPlane< VectorT , double >( _porigin, _pnormal, p1) > 0 ) ++npos; else ++nneg;
  if( ACG::Geometry::distPointPlane< VectorT , double >( _porigin, _pnormal, p2) > 0 ) ++npos; else ++nneg;


  if( npos && nneg ) return true;
  else               return false;
}

template < typename MeshT >
double 
calc_area( const MeshT& _mesh) 
{
  double area = 0.0;
  for ( typename MeshT::ConstFaceIter f_it = _mesh.faces_begin() ; f_it != _mesh.faces_end() ; ++f_it) {
     typename MeshT::ConstFaceVertexIter fv_it = _mesh.cfv_iter(f_it);  
     const ACG::Vec3d vertex1 = (ACG::Vec3d)_mesh.point( fv_it );
     const ACG::Vec3d vertex2 = (ACG::Vec3d)_mesh.point( ++fv_it );
     const ACG::Vec3d vertex3 = (ACG::Vec3d)_mesh.point( ++fv_it );
     
     area += ((vertex1 - vertex2) % (vertex3-vertex2)).norm();
   }
   
  return (0.5 * area);
  
}

template < typename MeshT >
double 
calc_angle_around( const MeshT& _mesh , const typename MeshT::VertexHandle& _vh)
{
    double angle = 0.0;
    
    const typename MeshT::Point p0 = _mesh.point(_vh);
    
    typename MeshT::ConstVertexOHalfedgeIter voh_it(_mesh,_vh);
    typename MeshT::ConstVertexOHalfedgeIter nx_voh_it = voh_it;
    
    ++nx_voh_it;
   
    for ( ; voh_it; ++voh_it , ++nx_voh_it) {
       const typename MeshT::Point edge_1 = MathTools::sane_normalized( _mesh.point(_mesh.to_vertex_handle(voh_it)) - p0);
       const typename MeshT::Point edge_2 = MathTools::sane_normalized( (_mesh.point(_mesh.to_vertex_handle(nx_voh_it))) - p0);
       angle += acos(OpenMesh::sane_aarg(edge_1 | edge_2));
    }
    
    return angle;
}

template< typename MeshT >
void transformMesh(ACG::Matrix4x4d _matrix , MeshT& _mesh) {

    // Get the inverse matrix of the transformation for the normals
    ACG::Matrix4x4d invTranspMat = _matrix;

    // Build inverse transposed matrix of _matrix
    invTranspMat.invert();
    invTranspMat.transpose();

    typename MeshT::VertexIter v_it = _mesh.vertices_begin();
    typename MeshT::VertexIter v_end = _mesh.vertices_end();
    for (; v_it != v_end; ++v_it) {

        // Transform the mesh vertex
        _mesh.set_point(*v_it, _matrix.transform_point(_mesh.point(*v_it)));

        // Transform the vertex normal
        if(_mesh.has_vertex_normals()) {
            typename MeshT::Normal n = invTranspMat.transform_vector(_mesh.normal(*v_it));
            n.normalize();
            _mesh.set_normal(*v_it, n);
        }
    }

    typename MeshT::FaceIter f_it = _mesh.faces_begin();
    typename MeshT::FaceIter f_end = _mesh.faces_end();
    for (; f_it != f_end; ++f_it) {

        // Transform the face normal
        if(_mesh.has_face_normals()) {
            typename MeshT::Normal n = invTranspMat.transform_vector(_mesh.normal(*f_it));
            n.normalize();
            _mesh.set_normal(*f_it, n);
        }
    }
}

template< typename MeshT >
void transformHandleVertices(ACG::Matrix4x4d _matrix , MeshT& _mesh) {

    // Get the inverse matrix of the transformation for the normals
    ACG::Matrix4x4d invTranspMat = _matrix;

    // Build inverse transposed matrix of _matrix
    invTranspMat.invert();
    invTranspMat.transpose();

    typename MeshT::VertexIter v_it = _mesh.vertices_begin();
    typename MeshT::VertexIter v_end = _mesh.vertices_end();
    for (; v_it != v_end; ++v_it) {

        if(!_mesh.status(*v_it).is_bit_set(HANDLEAREA)) continue;

        // Transform the mesh vertex
        _mesh.set_point(*v_it, _matrix.transform_point(_mesh.point(*v_it)));

        // Transform the vertex normal
        if(_mesh.has_vertex_normals()) {
            typename MeshT::Normal n = invTranspMat.transform_vector(_mesh.normal(*v_it));
            n.normalize();
            _mesh.set_normal(*v_it, n);
        }
    }

    // Transform the face normal
    if(_mesh.has_face_normals()) {
        _mesh.update_face_normals();
    }
}

//=============================================================================
} // MeshFunctions Namespace 
//=============================================================================

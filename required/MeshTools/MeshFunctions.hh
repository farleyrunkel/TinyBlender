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
//
//=============================================================================


#ifndef MESHFUNCTIONS_HH
#define MESHFUNCTIONS_HH


/*! \file MeshFunctions.hh
    \brief Functions for modifying a Mesh
    
    General file with template functions doing modifications on a given Mesh 
    (e.g smooth, reposition,... )
*/

//== INCLUDES =================================================================

#include <vector>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

/// Namespace providing different Mesh editing functions
namespace MeshFunctions {

//== DEFINITIONS ==============================================================

/**
    get one boundary at the vertex
    @param _mesh Mesh
    @param _vh   Vertex handle of one vertex of the boundary
    @param _boundary Coords and vertex handles of the boundary
  */
template < typename MeshT , typename VectorT >
bool get_boundary(MeshT& _mesh, 
                               typename MeshT::VertexHandle _vh, 
                               std::vector< std::pair< VectorT , typename MeshT::VertexHandle > >& _boundary);  

/**
    get boundary of a mesh (assumes that there is only one boundary!!
    @param _mesh Mesh
    @param _boundary Coords and vertex handles of the boundary
    @return Found a boundary?
  */
template < typename MeshT , typename VectorT >
bool get_boundary(MeshT& _mesh, 
                               std::vector< std::pair< VectorT , 
                               typename MeshT::VertexHandle > >& _boundary);  

/**
    function to smooth a boundary of a given mesh by moving each vertex to the 
    mean Position of its adjazent verticies  
    @param _mesh Mesh to work on
    @param _vh Vertex handle on the boundary 
*/ 
template < typename MeshT , typename VectorT >
void smooth_boundary(MeshT& _mesh , 
                     typename MeshT::VertexHandle _vh);

/**
    Checks for two faces if they are adjazent
    @param _mesh Mesh
    @param _fh1  First Face 
    @param _fh2  Second Face
*/
template < typename MeshT >
bool neighbour(const MeshT& _mesh , 
               const typename MeshT::FaceHandle& _fh1 ,
               const typename MeshT::FaceHandle& _fh2 );

/**
   Checks if plane cuts the face
   @param _porigin Planes origin
   @param _pnormal Plane normal
   @param _mesh    Mesh
   @param _fh      Facehandle of face
   @return triangle cut by plane?
*/
template < typename MeshT , typename VectorT >
bool
cut_face(const VectorT& _porigin, 
               const VectorT& _pnormal, 
               const MeshT& _mesh, 
               const typename MeshT::FaceHandle& _fh);
         
/** Get the area of a mesh ( sum over all triangle areas)
    @param _mesh Mesh to calculate area 
    @return Area spanned by the mesh
    */
template < typename MeshT >
double 
calc_area( const MeshT& _mesh);

/** Get the sum of the angles around a vertex
    @param _mesh Mesh to work on
    @param _vh Vertex
    @return Sum of angles around the vertex
    */
template < typename MeshT >
double 
calc_angle_around( const MeshT& _mesh , const typename MeshT::VertexHandle& _vh);

/**
 * Transform geometry of the mesh using the specified
 * transformation matrix.
 * @param _matrix The transformation matrix
 * @param _mesh The mesh that is to be transformed
 */
template< typename MeshT >
void transformMesh(ACG::Matrix4x4d _matrix , MeshT& _mesh);

/**
 * Transform handle vertices only
 * @param _matrix The transformation matrix
 * @param _mesh The mesh that is to be transformed
 */
template< typename MeshT >
void transformHandleVertices(ACG::Matrix4x4d _matrix , MeshT& _mesh);

//=============================================================================
} // MeshFunctions Namespace 
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(MESHFUNCTIONS_C)
#define MESHFUNCTIONS_TEMPLATES
#include "MeshFunctionsT_impl.hh"
#endif
//=============================================================================
#endif // MESHFUNCTIONS_HH defined
//=============================================================================


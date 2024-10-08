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


#ifndef MESHSELECTION_HH
#define MESHSELECTION_HH


/*! \file MeshSelectionT.hh
    \brief Functions for selection on a mesh

*/

//== INCLUDES =================================================================

#include <OpenMesh/Core/Mesh/Handles.hh>
#include <vector>

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace MeshSelection {

//== DEFINITIONS ==============================================================


//===========================================================================
/** @name Vertex Selection
* @{ */
//===========================================================================

/** \brief Select given vertices of a mesh
 *   @param _mesh      Mesh to work on
 *   @param _vertices  Vertex indices to select
 */
template< typename MeshT >
inline
void selectVertices(MeshT* _mesh, const std::vector< int >& _vertices);

/** \brief Select given vertices of a mesh
 *   Same as selectVertices
 *   @param _mesh      Mesh to work on
 *   @param _vertices  Vertex index to select
 */
template< typename MeshT >
inline
void selectElements(MeshT* _mesh, const std::vector< int >& _vertices, OpenMesh::VertexHandle /*_tag*/)
{
  selectVertices(_mesh, _vertices);
}


/** \brief Unselect given vertices of a mesh
 *   @param _mesh      Mesh to work on
 *   @param _vertices  Vertex index to unselect
 */
template< typename MeshT >
inline
void unselectVertices(MeshT* _mesh, const std::vector< int >& _vertices);

/** \brief Select all vertices of a mesh
 *   @param _mesh      Mesh to work on
 */
template< typename MeshT >
inline
void selectAllVertices(MeshT* _mesh);

/** \brief Set all vertices to unselected
 *   @param _mesh      Mesh to work on
 */
template< typename MeshT >
inline
void clearVertexSelection(MeshT* _mesh);

/** \brief invert vertex selection
 *   @param _mesh      Mesh to work on
 */
template< typename MeshT >
inline
void invertVertexSelection(MeshT* _mesh);

/** \brief Select all vertices of the mesh which are boundary vertices
 *
 *   @param _mesh      Mesh to work on
 */
template< typename MeshT >
inline
void selectBoundaryVertices(MeshT* _mesh);

/** \brief Shrink  vertex selection
 *   @param _mesh Mesh to work on
 */
template< typename MeshT >
inline
void shrinkVertexSelection(MeshT* _mesh) ;

/** \brief Grow vertex selection
 *   @param _mesh Mesh to work on
 */
template< typename MeshT >
inline
void growVertexSelection(MeshT* _mesh);

/** \brief Get the current vertex selection
 *
 *   @param _mesh Mesh to work on
 *   @return vector of selected vertex indices
 */
template< typename MeshT >
inline
std::vector< int > getVertexSelection(MeshT* _mesh);

/** Get the current vertex selection
 *
 * @param _mesh Mesh to work on
 * @param _invert if true : vector has been inverted to save mem
 */
template< typename MeshT >
inline
std::vector< int > getVertexSelection(MeshT* _mesh, bool& _invert);

/** \brief Select all vertices belonging to the same boundary as _vh
 *
 * This function gets one vertex that has to be on a boundary.
 * Than the function will follow the boundary and mark all vertices
 * at it as selected.
 *
 * @param _mesh Mesh to work on
 * @param _vh handle of a boundary vertex
 */
template< typename MeshT >
inline
void selectBoundaryVertices(MeshT* _mesh, const typename MeshT::VertexHandle& _vh);

/** \brief Select for each vertex in _vertices all incident edges
 *
 * @param _mesh Mesh to work on
 * @param _vertices List of vertices to be converted
 */
template< typename MeshT >
inline
void convertVertexToEdgeSelection(MeshT* _mesh, const std::vector< int >& _vertices);

/** \brief For each selected vertex select all incident edges
 *
 * @param _mesh Mesh to work on
 */
template< typename MeshT >
inline
void convertVertexToEdgeSelection(MeshT* _mesh);

/** \brief Select for each vertex in _vertices all incident halfedges
 *
 * @param _mesh Mesh to work on
 * @param _vertices List of vertices to be converted
 */
template< typename MeshT >
inline
void convertVertexToHalfedgeSelection(MeshT* _mesh, const std::vector< int >& _vertices);

/** \brief For each selected vertex select all incident halfedges
 *
 * @param _mesh Mesh to work on
 */
template< typename MeshT >
inline
void convertVertexToHalfedgeSelection(MeshT* _mesh);

/** \brief  Select for each vertex in _vertices all adjacent faces
 *
 * @param _mesh Mesh to work on
 * @param _vertices List of vertices to be converted
 */
template< typename MeshT >
inline
void convertVertexToFaceSelection(MeshT* _mesh, const std::vector< int >& _vertices);

/** \brief For each selected vertex select all adjacent faces
 *
 * @param _mesh Mesh to work on
 */
template< typename MeshT >
inline
void convertVertexToFaceSelection(MeshT* _mesh);

/** \brief Convert vertex selection to feature selection
 *
 * @param _mesh Mesh to work on
 */
template< typename MeshT >
inline
void convertVertexSelectionToFeatureVertices(MeshT* _mesh);

/** \brief  Convert feature selection to vertex selection
 *
 * @param _mesh Mesh to work on
 */
template< typename MeshT >
inline
void convertFeatureVerticesToVertexSelection(MeshT* _mesh);

/** \brief Clear all features
 *
 * @param _mesh Mesh to work on
 */
template< typename MeshT >
inline
void clearFeatureVertices(MeshT* _mesh);

/** @} */

//===========================================================================
/** @name Modeling Regions
* @{ */
//===========================================================================

/** \brief Set the area bit for all defined vertices
 *
 * @param _mesh Mesh to work on
 * @param _vertices The vertices belonging to the modeling area
 * @param _type Bit to be changed ( Normally Modeling area is OpenMesh::Attributes::UNUSED << 1 )
 * @param _state New state to be set
 * */
template< typename MeshT >
inline
void setArea(MeshT* _mesh, const std::vector< int >& _vertices , unsigned int _type, bool _state);

/** \brief Reset Modeling Status for vertices
 *
 * @param _mesh Mesh to work on
 * @param _type Bit to be changed ( Normally Modeling area is OpenMesh::Attributes::UNUSED << 1 )
 * @param _state New state to be set
 * */
template< typename MeshT >
inline
void setArea(MeshT* _mesh , unsigned int _type, bool _state);

/** Get the current vertex selection
 */
template< typename MeshT >
inline
std::vector< int > getArea(MeshT* _mesh, unsigned int _type);

/** Get the current vertex selection
 *
 * @param _mesh Mesh to work on
 * @param _type Bit to be changed ( Normally Modeling area is OpenMesh::Attributes::UNUSED << 1 )
 * @param _invert if true : vector has been inverted to save mem
 */
template< typename MeshT >
inline
std::vector< int > getArea(MeshT* _mesh, unsigned int _type , bool& _invert);

/** @} */

//===========================================================================
/** @name Edge Selection
* @{ */
//===========================================================================

/** \brief Select given edges of a mesh
 * @param _mesh Mesh to work on
 * @param _edges Edge indices to select
 * @param _dihedral_angle_threshold If mesh has normals, dihedral angle must be greator than or equal to this threshold in order to be selected
 */
template< typename MeshT >
inline
void selectEdges(MeshT* _mesh, const std::vector< int >& _edges, const double _dihedral_angle_threshold = 0.0);


/** \brief Select given edges of a mesh
 *   Same as selectEdges
 * @param _mesh Mesh to work on
 * @param _edges Edge indices to select
 * @param _dihedral_angle_threshold If mesh has normals, dihedral angle must be greator than or equal to this threshold in order to be selected
 */
template< typename MeshT >
inline
void selectElements(MeshT* _mesh, const std::vector< int >& _edges, OpenMesh::EdgeHandle /*_tag*/)
{
  selectEdges(_mesh, _edges);
}

/// Unselect given edges of a mesh
template< typename MeshT >
inline
void unselectEdges(MeshT* _mesh, const std::vector< int >& _edges);

/// Select all edges of a mesh
template< typename MeshT >
inline
void selectAllEdges(MeshT* _mesh);

///  Set all edges to unselected
template< typename MeshT >
inline
void clearEdgeSelection(MeshT* _mesh);

/// Invert Edge selection
template< typename MeshT >
inline
void invertEdgeSelection(MeshT* _mesh);

/// Invert Edge selection
template< typename MeshT >
inline
void growEdgeSelection(MeshT* _mesh);

/// Select all boundary edges of a mesh
template< typename MeshT >
void selectBoundaryEdges(MeshT* _mesh);

/** Get the current edge selection
 */
template< typename MeshT >
inline
std::vector< int > getEdgeSelection(MeshT* _mesh);

/** Get the current edge selection
 * @param _mesh Mesh to work on
 * @param _invert if true : vector has been inverted to save mem
 */
template< typename MeshT >
inline
std::vector< int > getEdgeSelection(MeshT* _mesh, bool& _invert);

/**
 * Select for each edge in _edges all incident vertices
 * @param _mesh Mesh to work on
 * @param _edges List of edges to be converted
 */
template< typename MeshT >
inline
void convertEdgeToVertexSelection(MeshT* _mesh, const std::vector< int >& _edges);

/**
 * For each selected edge select all incident vertices
 */
template< typename MeshT >
inline
void convertEdgeToVertexSelection(MeshT* _mesh);

/**
 * Select for each edge in _edges all adjacent faces
 * @param _mesh Mesh to work on
 * @param _edges List of edges to be converted
 */
template< typename MeshT >
inline
void convertEdgeToFaceSelection(MeshT* _mesh, const std::vector< int >& _edges);

/**
 * For each selected edge select all adjacent faces
 */
template< typename MeshT >
inline
void convertEdgeToFaceSelection(MeshT* _mesh);

/**
 * For each selected edge select all halfedges
 */
template< typename MeshT >
inline
void convertEdgeToHalfedgeSelection(MeshT* _mesh);

/**
 * Convert edge selection to feature selection
 */
template< typename MeshT >
inline
void convertEdgeSelectionToFeatureEdges(MeshT* _mesh);

/**
 * Convert feature selection to edge selection
 */
template< typename MeshT >
inline
void convertFeatureEdgesToEdgeSelection(MeshT* _mesh);

/**
 * Clear all features
 */
template< typename MeshT >
inline
void clearFeatureEdges(MeshT* _mesh);


/** @} */

//===========================================================================
/** @name Halfedge Selection
* @{ */
//===========================================================================

/** \brief Select given halfedges of a mesh
 * @param _mesh Mesh to work on
 * @param _halfedges face indices to select
 */
template< typename MeshT >
inline
void selectHalfedges(MeshT* _mesh, const std::vector< int >& _halfedges);

/** \brief Select given halfedges of a mesh
 *   Same as selectHalfedges
 * @param _mesh Mesh to work on
 * @param _halfedges halfedge indices to select
 */
template< typename MeshT >
inline
void selectElements(MeshT* _mesh, const std::vector< int >& _halfedges, OpenMesh::HalfedgeHandle /*_tag*/)
{
  selectHalfedges(_mesh, _halfedges);
}

/// Unselect given edges of a mesh
template< typename MeshT >
inline
void unselectHalfedges(MeshT* _mesh, const std::vector< int >& _halfedges);

/// Select all edges of a mesh
template< typename MeshT >
inline
void selectAllHalfedges(MeshT* _mesh);

///  Set all edges to unselected
template< typename MeshT >
inline
void clearHalfedgeSelection(MeshT* _mesh);

/// Invert Edge selection
template< typename MeshT >
inline
void invertHalfedgeSelection(MeshT* _mesh);

/// Select all boundary edges of a mesh
template< typename MeshT >
void selectBoundaryHalfedges(MeshT* _mesh);

/** Get the current edge selection
 */
template< typename MeshT >
inline
std::vector< int > getHalfedgeSelection(MeshT* _mesh);

/**
 * For each halfedge select all incident vertices
 */
template< typename MeshT >
inline
void convertHalfedgeToVertexSelection(MeshT* _mesh);

/**
 * For each halfedge select all edges
 */
template< typename MeshT >
inline
void convertHalfedgeToEdgeSelection(MeshT* _mesh);

/**
 * For each halfedge select all incident faces
 */
template< typename MeshT >
inline
void convertHalfedgeToFaceSelection(MeshT* _mesh);

//===========================================================================
/** @name Face Selection
* @{ */
//===========================================================================

/** \brief Select given faces of a mesh
 * @param _mesh Mesh to work on
 * @param _faces face indices to select
 */
template< typename MeshT >
inline
void selectFaces(MeshT* _mesh, const std::vector< int >& _faces );

/** \brief Select given faces of a mesh
 *   Same as selectFaces
 * @param _mesh Mesh to work on
 * @param _faces face indices to select
 */
template< typename MeshT >
inline
void selectElements(MeshT* _mesh, const std::vector< int >& _faces, OpenMesh::FaceHandle /*_tag*/)
{
  selectFaces(_mesh, _faces);
}

/// Unselect given faces of a mesh
template< typename MeshT >
inline
void unselectFaces(MeshT* _mesh, const std::vector< int >& _faces );

/// Select all faces of a mesh
template< typename MeshT >
inline
void selectAllFaces(MeshT* _mesh);

/// Set all faces to unselected
template< typename MeshT >
inline
void clearFaceSelection(MeshT* _mesh);

/// Invert face selection
template< typename MeshT >
inline
void invertFaceSelection(MeshT* _mesh);

/// Select all boundary faces of a mesh
template< typename MeshT >
void selectBoundaryFaces(MeshT* _mesh);

/** \brief Shrink Face selection
 *
 * Deselects all faces which are adjacent to a boundary vertex of the original selection
 */
template< typename MeshT >
inline
void shrinkFaceSelection(MeshT* _mesh);

/** \brief Grow Face selection
 *
 * Selects all faces which are adjacent to a vertex of a already selected face.
 */
template< typename MeshT >
inline
void growFaceSelection(MeshT* _mesh);

/** Get the current face selection
 */
template< typename MeshT >
inline
std::vector< int > getFaceSelection(MeshT* _mesh);

/** Get the current face selection
 *
 * @param _mesh Mesh to work on
 * @param _invert if true : vector has been inverted to save mem
 */
template< typename MeshT >
inline
std::vector< int > getFaceSelection(MeshT* _mesh, bool& _invert);

/**
 * Select for each face in _faces all adjacent vertices
 *
 * @param _mesh Mesh to work on
 * @param _faces List of faces to be converted
 */
template< typename MeshT >
inline
void convertFaceToVertexSelection(MeshT* _mesh, const std::vector< int >& _faces);

/**
 * For each selected face select all adjacent vertices
 */
template< typename MeshT >
inline
void convertFaceToVertexSelection(MeshT* _mesh);

/**
 * For each selected face select all adjacent edges
 */
template< typename MeshT >
inline
void convertFaceToEdgeSelection(MeshT* _mesh);

/**
 * For each selected face select all incident halfedges
 */
template< typename MeshT >
inline
void convertFaceToHalfedgeSelection(MeshT* _mesh);

/**
 * Convert face selection to feature selection
 */
template< typename MeshT >
inline
void convertFaceSelectionToFeatureFaces(MeshT* _mesh);

/**
 * Convert feature selection to edge selection
 */
template< typename MeshT >
inline
void convertFeatureFacesToFaceSelection(MeshT* _mesh);

/**
 * Clear all features
 */
template< typename MeshT >
inline
void clearFeatureFaces(MeshT* _mesh);

/** @} */

//=============================================================================
} // MeshSelection Namespace
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(MESHSELECTION_C)
#define MESHSELECTION_TEMPLATES
#include "MeshSelectionT_impl.hh"
#endif
//=============================================================================
#endif // MESHSELECTION_HH defined
//=============================================================================


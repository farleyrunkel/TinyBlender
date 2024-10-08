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






//=============================================================================
//
//  CLASS GridNode
//
//=============================================================================


#ifndef ACG_GRIDNODE_HH
#define ACG_GRIDNODE_HH


//== INCLUDES =================================================================


#include "MaterialNode.hh"
#include <vector>


//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== CLASS DEFINITION =========================================================



/** \class GridNode GridNode.hh <ACG/.../GridNode.hh>

    Scenegraph Node.

    A more elaborate description follows.
**/
class ACGDLLEXPORT GridNode : public MaterialNode
{
public:

  /// Default constructor
  GridNode(BaseNode*  _parent=0,
	     const std::string&  _name="<GridNode>" );

  /// Destructor
  ~GridNode() {}
  
  /// Enum for Orientation
  enum Orientation { NONE     = 0,
                     XY_PLANE = 1 , 
                     XZ_PLANE = 2 , 
                     YZ_PLANE = 4 };

  /// implement className()
  ACG_CLASSNAME(GridNode);
  
  /// return available draw modes
  ACG::SceneGraph::DrawModes::DrawMode  availableDrawModes() const override;
  
  /// update bounding box
  void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax) override;
  
  /// drawing the primitive
  void draw(GLState& _state, const DrawModes::DrawMode& _drawMode) override;
  
  /// don't pick me
  void pick(GLState& _state, PickTarget _target) override;
  
public:

  /// Get GridSize
  float gridSize();

  /// Set Grid Size
  void gridSize(float _size);
  
  
  /** \brief Set the minimum refinement distance
  *
  * This is the distance the viewer needs to the grid, when refinement is started.
  * At a distance greater then the given distance, no refinement takes places.
  */
  void minRefinementDistance( double _distance );
  
  /** \brief returns the minimal refinement distance
  */
  double minRefinementDistance();
  
  /** \brief Set the plane orientation
  *
  */
  void setOrientation( unsigned int _orientation ) ;
  
  void autoResize(bool _auto);

private:

  /// initial number of baseLines
  int horizontalLines_;
  int verticalLines_;
  int maxRefinement_;
  
  
  /** This is the distance, which the viewer has to the grid at which the grid is
   *  shown with minimal refinement ( lowest split count )
   */
  double minRefinementDistance_;

  /// dimensions of the grid
  float gridSize_;

  /// bounding box
  Vec3d bb_min_;
  Vec3d bb_max_;

  /// colors for the grid
  Vec3f baseLineColor_;
  Vec3f midLineColor_;
  
  bool autoResize_;
  
  /// Contains all orientations to draw
  unsigned int orientation_;
  
};


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#endif // ACG_GRIDNODE_HH defined
//=============================================================================


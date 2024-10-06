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
//  CLASS GridNode - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================


#include "GridNode.hh"
#include "SceneGraph.hh"
#include <cstdio>

//== NAMESPACES ===============================================================


namespace ACG {
namespace SceneGraph {


//== IMPLEMENTATION ==========================================================


GridNode::
GridNode(BaseNode* _parent, const std::string& _name)
  : MaterialNode(_parent, _name),
    horizontalLines_(7),
    verticalLines_(7),
    maxRefinement_(49),
    minRefinementDistance_(10),
    gridSize_(10.0),
    baseLineColor_( Vec3f(0.5f, 0.5f, 0.5f) ),
    midLineColor_( Vec3f(0.3f, 0.3f, 0.3f) ),
    autoResize_(false),
    orientation_(XZ_PLANE)
{

}


//-----------------------------------------------------------------------------


DrawModes::DrawMode
GridNode::availableDrawModes() const
{
  return ( DrawModes::WIREFRAME           |
	        DrawModes::SOLID_FLAT_SHADED   );
}


//-----------------------------------------------------------------------------


void
GridNode::boundingBox(Vec3d& _bbMin, Vec3d& _bbMax)
{
  // If we do not autoresize, we set our bounding box here
  // otherwise we set the size of the grid according to the 
  // real scene geometry from the state
//   if ( !autoResize_ ) {

  // Only return a bounding box, if something will be drawn
  if ( orientation_ != NONE) {
    bb_min_ = Vec3f(-0.5*gridSize_, 0.0, -0.5*gridSize_);
    bb_max_ = Vec3f( 0.5*gridSize_, 0.0,  0.5*gridSize_);

    _bbMin.minimize(bb_min_);
    _bbMax.maximize(bb_max_);
   }
  
}


//----------------------------------------------------------------

void
GridNode::pick(GLState& /*_state*/, PickTarget /*_target*/)
{


}

//-----------------------------------------------------------------------------


void
GridNode::draw(GLState&  _state  , const DrawModes::DrawMode& /* _drawMode */ )
{

  glPushAttrib( GL_LIGHTING_BIT ); // STACK_ATTRIBUTES <- LIGHTING_ATTRIBUTE
  ACG::GLState::disable(GL_LIGHTING);

  glPushAttrib( GL_DEPTH_TEST );
  ACG::GLState::enable( GL_DEPTH_TEST );

  glLineWidth(0.1f);

  ACG::Vec3d direction1,direction2;
  
  // For all orientations :
  for ( unsigned int i = 0 ; i < 3 ; ++i ) {
    
    Vec3d viewDirection(0.0, 1.0, 0.0); // = _state.viewing_direction();
    Vec3d eye = _state.eye();
    
    //first we calculate the hitpoint of the camera direction with the grid
    GLMatrixd modelview = _state.modelview();
    
    // Distance from eye point to the plane
    double distance = 0.0;
    
    if ( i == 0 && ((orientation_ ) & XZ_PLANE)) {
      direction1 = ACG::Vec3f( 1.0 , 0.0 , 0.0 );
      direction2 = ACG::Vec3f( 0.0 , 0.0 , 1.0 );
      distance = fabs( eye | ACG::Vec3d(0.0,1.0,0.0 ) );
    } else if ( i == 1 && (orientation_  & XY_PLANE)) {
      direction1 = ACG::Vec3f( 1.0 , 0.0 , 0.0 );
      direction2 = ACG::Vec3f( 0.0 , 1.0 , 0.0 );
      distance = fabs( eye | ACG::Vec3d(0.0,0.0,1.0 ) );
    } else if ( i == 2 && (orientation_  & YZ_PLANE)) {
      direction1 = ACG::Vec3f( 0.0 , 1.0 , 0.0 );
      direction2 = ACG::Vec3f( 0.0 , 0.0 , 1.0 );
      distance = fabs( eye | ACG::Vec3d(1.0,0.0,0.0 ) );
    } else {
      continue; 
    }
    
    // Remember original Modelview Matrix
    _state.push_modelview_matrix();
    
    // The factor is means the following
    // If the viewer is farther away than the minRefinementDistance_, the factor is 0 and no refinement will take place
    // If the viewer goes closer to the grid, the grid will be refined. 
    // Block negative distances (grid behind us)
    int factor = floor( minRefinementDistance_ / distance) - 1;
    
    int vertical   = verticalLines_;
    int horizontal = horizontalLines_;
    
    if (factor > 20){
      vertical   = maxRefinement_;
      horizontal = maxRefinement_;
      
    } else if (factor > 0){
      // 'factor' times split the grid (split every cell into 4 new cells)
      int rest = 0;
      
      for (int j=0; j < factor; j++)
        rest -= floor( pow(double(2.0), j));
      
      vertical   = vertical   * floor( pow(double(2.0),factor)) + rest;
      horizontal = horizontal * floor( pow(double(2.0),factor)) + rest;
      
      vertical   = std::min(vertical, maxRefinement_ );
      horizontal = std::min(vertical, maxRefinement_ );
    }
    
    
    
//     if ( autoResize_ ) {
//       
//       // Get the current scene size from the glstate
//       ACG::Vec3d bb_min,bb_max;
//       _state.get_bounding_box(bb_min,bb_max);
//       
//       double radius = std::max( std::max( fabs(bb_max | direction1) , fabs(bb_min | direction1) ),
//                       std::max( fabs(bb_max | direction2) , fabs(bb_min | direction2) ) ) * 2.0;
//       
//       // compute the minimal scene radius from the bounding box
// //       double radius = std::min(  fabs(bb_max[0]-bb_min[0]) * 2,fabs(bb_max[2]-bb_min[2]) * 2);
//       
//       // set grid size to the given radius if we autoadjust to the scene size
//       if ( radius > 10.0 )
//         gridSize_ = radius;
//       
//       
//     /*  
//       _state.set_line_width(3);
//       glBegin(GL_LINES);
//       glVertex(bb_min);
//       glVertex(bb_max);
//       glEnd();*/
//       
//       //     // update the bounding box
//       //     bb_min_ = Vec3f(-0.5*gridSize_, 0.0, -0.5*gridSize_);
//       //     bb_max_ = Vec3f( 0.5*gridSize_, 0.0,  0.5*gridSize_);
//       
//       std::cerr << "Draw " << bb_min << " "  << bb_max << std::endl;
//       
//       
//       
//     }
    
    //now start drawing
    _state.translate( direction1 * -0.5 * gridSize_ + direction2 * -0.5 * gridSize_ );
    
    glBegin(GL_LINES);

      //red line (x-axis)
      glColor3f( 0.7f, 0.0f, 0.0f );
      glVertex(  direction2 * gridSize_ * 0.5 );
      glVertex(  direction1 * gridSize_ + direction2 * gridSize_ * 0.5 );

      //blue line (z-axis)
      glColor3f( 0.0f, 0.0f, 0.7f );
      glVertex( direction1 * gridSize_ * 0.5 );
      glVertex(  direction1 * 0.5 * gridSize_ + direction2 * gridSize_);

      //remaining vertical lines
      for ( int j = 0 ; j <  vertical ; ++j ) {

        //first draw a baseLine
        glColor3fv( &baseLineColor_[0] );

        double big  = gridSize_ / (vertical-1) * j;
        double next = gridSize_ / (vertical-1) * (j+1);

        glVertex( direction1 * big );
        glVertex( direction1 * big + direction2 * gridSize_ );

        if ( j+1 < vertical)
          for (int k=1; k < 10; k++){

            //then draw 9 darker lines in between
            glColor3fv( &midLineColor_[0] );
    
            double smallPos = big + (next - big) / 10.0 * k;
            glVertex( direction1 * smallPos);
            glVertex( direction1 * smallPos + direction2 * gridSize_);
          }
      }

      //remaining horizontal lines
      for ( int j = 0 ; j <  horizontal; ++j ) {

        //first draw a baseline
        glColor3fv( &baseLineColor_[0] );

        double big  = gridSize_ / (vertical-1) * j;
        double next = gridSize_ / (vertical-1) * (j+1);

        glVertex( direction2 * gridSize_ / (horizontal-1) * j);
        glVertex( direction1 * gridSize_ + direction2 * gridSize_ / (horizontal-1) * j);

        if ( j+1 < vertical)
          for (int k=1; k < 10; k++){

            //then draw 9 darker lines in between
            glColor3fv( &midLineColor_[0] );
    
            double smallPos = big + (next - big) / 10.0 * k;
            glVertex( direction2 * smallPos );
            glVertex( direction1 * gridSize_ + direction2 * smallPos );
          }   
      }
    glEnd();

    _state.pop_modelview_matrix();
    
  }

  glLineWidth(1.0);

  glPopAttrib( ); // ->Depth test
  glPopAttrib( ); // ->Lighting
}

//-----------------------------------------------------------------------------

void
GridNode::gridSize(float _size){
  gridSize_ = _size;

  bb_min_ = Vec3f(-0.5*gridSize_, 0.0, -0.5*gridSize_);
  bb_max_ = Vec3f( 0.5*gridSize_, 0.0,  0.5*gridSize_);
}

//-----------------------------------------------------------------------------

float
GridNode::gridSize(){
  return gridSize_;
}

//-----------------------------------------------------------------------------

void 
GridNode::minRefinementDistance( double _distance ) {
  minRefinementDistance_ = _distance;
}

//-----------------------------------------------------------------------------

double 
GridNode::minRefinementDistance()
{
 return minRefinementDistance_;
}

//-----------------------------------------------------------------------------

void GridNode::setOrientation( unsigned int _orientation)
{
  orientation_ = _orientation;
}

//-----------------------------------------------------------------------------

void GridNode::autoResize(bool _auto)
{
  autoResize_ = _auto;
}


//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================

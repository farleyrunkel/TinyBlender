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
//  CLASS ColorStack
//
//=============================================================================

#ifndef ACG_COLORSTACK_HH
#define ACG_COLORSTACK_HH


//== INCLUDES =================================================================

#include <vector>

#include <ACG/GL/gl.hh>
#include <ACG/Math/VectorT.hh>
#include "ColorTranslator.hh"


//== NAMESPACES ===============================================================


namespace ACG {

class GLState;

//== CLASS DEFINITION =========================================================


/** This class can be used to implement a gl picking stack based on colors
*/
class ACGDLLEXPORT ColorStack
{
public:
   
  /// Default constructor.
  ColorStack();
   /// Destructor.
  ~ColorStack();
  
  /// init (takes current GL context/ like glInitNames (); glPushName (0))
  void initialize(ACG::GLState*);
  /// has it been initialized?
  bool initialized() const { return initialized_; }

  /// sets the maximum index number used in current node 
  bool setMaximumIndex (size_t _idx);

  /// sets the current color the given index (like glLoadName)
  void setIndex (size_t _idx);

  /// gets the color instead of setting it directly
  Vec4uc getIndexColor (size_t _idx);

  /// creates a new node the stack (like glPushName)
  void pushIndex (size_t _idx);

  /// pops the current node from the stack (like glPopName)
  void popIndex ();

  /// converts the given color to index values on the stack
  std::vector<size_t> colorToStack (Vec4uc _rgba) const;

  /// returns maximal available index count
  size_t freeIndicies () const;

  /// Did an error occur during picking
  bool error () const { return error_ && initialized_; };

  /// returns the current color index
  size_t currentIndex () const;

private:

  // Internal class used realize the color stack

  class Node {
    public:
      Node (size_t _idx, Node *_parent, ColorTranslator *_ct);
      ~Node ();

      /// sets the maximum index number used in current node 
      bool setMaximumIndex (size_t _idx);

      /// sets the current color the given index (like glLoadName)
      bool setIndex (size_t _idx) const;

      /// gets the color instead of setting it directly
      bool getIndexColor (size_t _idx, Vec4uc &_rgba) const;

      /// creates a new node the stack (like glPushName)
      Node * pushIndex (size_t _idx);

      /// pops the current node from the stack (like glPopName)
      Node * popIndex ();

      void colorToStack (std::vector<size_t> &_stack, size_t size_t);

      size_t startIndex () const { return startIdx_; };
      size_t endIndex ()   const { return endIdx_; };
      size_t colorIndex () const { return colorStartIdx_; };

    private:
      Node                *parent_;
      size_t              index_;
      ColorTranslator     *translator_;
      std::vector<Node *> nodes_;

      size_t startIdx_;
      size_t endIdx_;
      size_t colorStartIdx_;
      size_t colorEndIdx_;

  };

  bool            initialized_;
  ColorTranslator translator_;
  Node            *root_;
  Node            *currentNode_;
  bool            error_;
};


//=============================================================================
} // namespace ACG
//=============================================================================
#endif // ACG_COLORSTACK_HH defined
//=============================================================================


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
//  CLASS QtLasso
//
//=============================================================================


#ifndef ACG_QTLASSO_HH
#define ACG_QTLASSO_HH


//== INCLUDES =================================================================

// ACG
#include "../GL/GLState.hh"
#include "../Math/VectorT.hh"

// Qt
//#include <QGl>
//#include <QNameSpace>
#include <QMouseEvent>


//== FORWARDDECLARATIONS ======================================================


//== NAMESPACES ===============================================================

namespace ACG {


//== CLASS DEFINITION =========================================================


class ACGDLLEXPORT QtLasso : public QObject
{
  Q_OBJECT

public:

  explicit QtLasso(GLState& _glstate);
  ~QtLasso();

  void reset() { is_active_ = false; free_mask(); }
  bool is_active() const { return is_active_; }

  bool is_vertex_selected(const Vec3d& _v);

  enum SelectionMode {
    NewSelection,
    AddToSelection,
    DelFromSelection
  };


public slots:

  void slotMouseEvent(QMouseEvent* _event);


signals:

  void signalLassoSelection(ACG::QtLasso::SelectionMode);


private:

  // hide copy & assignement
  QtLasso(const QtLasso& _rhs);
  QtLasso& operator=(const QtLasso& _rhs);

  void create_mask();
  void free_mask();


  GLState&                glstate_;
  Vec2i                   first_point_, last_point_, rubberband_point_;
  unsigned char          *mask_;
  unsigned int            mask_width_, mask_height_;
  bool                    is_active_;
};


//=============================================================================
} // namespace ACG
//=============================================================================
#endif // ACG_QTLASSO_HH defined
//=============================================================================


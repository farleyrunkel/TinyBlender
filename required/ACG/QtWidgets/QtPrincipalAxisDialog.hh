/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2016, RWTH-Aachen University                 *
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
//  CLASS PrincipalAxisDialog
//
//=============================================================================


#ifndef ACG_PRINCIPALAXISDIALOG_HH
#define ACG_PRINCIPALAXISDIALOG_HH


//== INCLUDES =================================================================


// qmake users have to include
#include "ui_QtPrincipalAxisDialogBaseUi.h"

// ACGMake users have to include
// #include "QtPrincipalAxisDialogBaseUi.hh"


#include <ACG/Scenegraph/PrincipalAxisNode.hh>

//== FORWARDDECLARATIONS ======================================================

namespace SceneGraph
{
class PrincipalAxisNode;
}

//== NAMESPACES ===============================================================

namespace ACG
{

//== CLASS DEFINITION =========================================================



/** \class PrincipalAxisDialog PrincipalAxisDialog.hh <ACG/.../PrincipalAxisDialog.hh>

    Brief Description.

    A more elaborate description follows.
*/
class QtPrincipalAxisDialog
         : public QDialog, Ui::QtPrincipalAxisDialogBaseUi
{
   Q_OBJECT
public:

   typedef SceneGraph::PrincipalAxisNode PrincipalAxisNode;

   /// Default constructor
   QtPrincipalAxisDialog( SceneGraph::PrincipalAxisNode& _pnode,
                        QWidget*    _parent = 0 ):
         QDialog( _parent ),
         Ui::QtPrincipalAxisDialogBaseUi(),
         pnode_( _pnode )
   {
      setupUi( this );
      get_parameters();

      connect( OkButton, SIGNAL( clicked() ), this, SLOT( slotOkButton() ) );
      connect( CancelButton, SIGNAL( clicked() ), this, SLOT( slotCancelButton() ) );
   }

   /// Destructor
   ~QtPrincipalAxisDialog() {}

   void get_parameters();
   void set_parameters();

   typedef Vec4f ColorScheme[3];
   const ColorScheme &getSelectedColorScheme();
   void setColorScheme(ColorScheme cs);


public slots:
   virtual void slotOkButton();
   virtual void slotCancelButton();

private:

   PrincipalAxisNode& pnode_;
   static const ColorScheme color_schemes_[3];
};


//=============================================================================
} // namespace ACG
//=============================================================================
#endif // ACG_PRINCIPALAXISDIALOG_HH defined
//=============================================================================


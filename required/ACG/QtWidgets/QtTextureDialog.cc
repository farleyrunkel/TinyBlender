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
//  CLASS QtTextureDialog - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "QtTextureDialog.hh"
#include "../Scenegraph/TextureNode.hh"


//== NAMESPACES ==============================================================


namespace ACG {
namespace QtWidgets {


//== IMPLEMENTATION ==========================================================


QtTextureDialog::QtTextureDialog( QWidget                  * _parent,
				                        SceneGraph::TextureNode * _node )
  : QDialog( _parent ),
    node_(_node) {
  
    ui_.setupUi( this );
  
    // get initial values from node
    mipmapping_   = bak_mipmapping_   = node_->mipmapping();
    
    ui_.mipmapping->setChecked(mipmapping_);

    connect( ui_.mipmapping, SIGNAL( toggled(bool) ),
            this, SLOT( changeMipmapping(bool) ) );

    connect( ui_.okButton, SIGNAL( clicked() ),
            this, SLOT( accept() ) );
    connect( ui_.cancelButton, SIGNAL( clicked() ),
            this, SLOT( reject() ) );
}

//-----------------------------------------------------------------------------

void QtTextureDialog::reject() {
    
    undoChanges();
    QDialog::reject();
}


//-----------------------------------------------------------------------------


void QtTextureDialog::applyChanges() {

    if ( mipmapping_ )
        node_->enable_mipmapping();
    else
        node_->disable_mipmapping();

    emit signalNodeChanged(node_);
}


//-----------------------------------------------------------------------------


void QtTextureDialog::undoChanges() {

    if ( bak_mipmapping_ )
        node_->enable_mipmapping();
    else
        node_->disable_mipmapping();

    emit signalNodeChanged(node_);
}


//-----------------------------------------------------------------------------


void
QtTextureDialog::changeMipmapping(bool _b) {
  
    mipmapping_ = _b;
    applyChanges();
}

//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================

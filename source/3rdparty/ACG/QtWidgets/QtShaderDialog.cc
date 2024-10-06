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
//  CLASS QtShaderDialog - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "QtShaderDialog.hh"
#include "../Scenegraph/ShaderNode.hh"
#include "../Scenegraph/DrawModes.hh"
#include <QFileInfo>
#include <QDir>

//== NAMESPACES ============================================================== 


namespace ACG {
namespace QtWidgets {


//== IMPLEMENTATION ========================================================== 


QtShaderDialog::QtShaderDialog( QWidget                * _parent,
				SceneGraph::ShaderNode * _node )
  : QDialog( _parent ),
    node_(_node)
{
  ui_.setupUi( this );
  
  ACG::SceneGraph::DrawModes::DrawMode drawmode = ACG::SceneGraph::DrawModes::DEFAULT;
  
  while ( drawmode != ACG::SceneGraph::DrawModes::UNUSED )
  {
    ui_.drawModeBox->addItem( QString( drawmode.description().c_str() ) );
    ++drawmode ;
  }
  
  ui_.shaderDir->setText( QString(_node->shaderDir().c_str()) );
  ui_.vertexShader->setText( QString(_node->vertexShaderName(ACG::SceneGraph::DrawModes::DEFAULT).c_str()) );
  ui_.fragmentShader->setText( QString(_node->fragmentShaderName(ACG::SceneGraph::DrawModes::DEFAULT).c_str()) ); 
  
  connect( ui_.okButton, SIGNAL( clicked() ),
           this, SLOT( applyChanges() ) );
  connect( ui_.cancelButton, SIGNAL( clicked() ),
           this, SLOT( reject() ) );
  
  connect( ui_.drawModeBox, SIGNAL( currentIndexChanged( int ) ),
           this, SLOT( comboChanged( int ) ) );
}


//-----------------------------------------------------------------------------


void QtShaderDialog::reject()
{
  std::cerr << "reject" << std::endl;
  undoChanges();
  QDialog::reject();
}


//-----------------------------------------------------------------------------


void QtShaderDialog::applyChanges()
{
  // Get and test shader directory
  std::string shaderDirectory("");
  
  QString shaderDir = ui_.shaderDir->text();
  QDir dir(shaderDir);
  
  if ( dir.exists() ) {
    if ( ! shaderDir.endsWith('/' ) && ! shaderDir.endsWith( '\\' ) ) {
      shaderDir += "/"; 
    }
    
    shaderDirectory = std::string( shaderDir.toUtf8() );
    node_->setShaderDir( shaderDirectory );
    
  } else {
    std::cerr << "Shader directory does not exist" << std::string( shaderDir.toUtf8() ) << std::endl; 
    return;
  }
  
  SceneGraph::DrawModes::DrawMode drawMode = SceneGraph::DrawModes::DrawMode(1);
  
  for ( int i = 0 ; i < ui_.drawModeBox->currentIndex() ; ++i )
    ++drawMode;
  
  node_->setShader(drawMode, 
                   std::string(  ui_.vertexShader->text().toUtf8() ),
                   std::string(  ui_.fragmentShader->text().toUtf8() ) );
          
  emit signalNodeChanged(node_);
  
  accept();
}


//-----------------------------------------------------------------------------


void QtShaderDialog::undoChanges()
{
  std::cerr << "undo" << std::endl;
  emit signalNodeChanged(node_);
}

void QtShaderDialog::comboChanged ( int index ) {
  ACG::SceneGraph::DrawModes::DrawMode drawMode = ACG::SceneGraph::DrawModes::DrawMode(1);
  
  for ( int i = 0 ; i < index; ++i )
    ++drawMode;
  
  QString vertexShader(node_->vertexShaderName(drawMode).c_str());
  QString fragmentShader(node_->fragmentShaderName(drawMode).c_str());
  
  
  QString shaderDir( node_->shaderDir().c_str() );
  vertexShader   = vertexShader.remove( shaderDir );
  fragmentShader = fragmentShader.remove( shaderDir );
  
  ui_.vertexShader->setText( vertexShader );
  ui_.fragmentShader->setText( fragmentShader );
}


//=============================================================================
} // namespace QtWidgets
} // namespace ACG
//=============================================================================

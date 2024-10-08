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
//  CLASS QtFileDialog - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include "QtFileDialog.hh"

#include <OpenMesh/Core/IO/IOManager.hh>

#include <QFileDialog>
#include <QMessageBox>


//== NAMESPACES ===============================================================

namespace ACG {

//== IMPLEMENTATION ========================================================== 


QString
getOpenFileName(QWidget*        _parent, 
		const QString&  _caption,
		const QString&  _filter,
		const QString&  _start)
{

#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
  return  
    QFileDialog::getOpenFileName( _parent,  // parent
				  _caption, // caption
				  _start,   // dir
				  _filter,  // filter
				  0,        // selected filter
				  0        // options
				  );
#else
    return
      QFileDialog::getOpenFileName( _parent,  // parent
                    _caption, // caption
                    _start,   // dir
                    _filter,  // filter
                    0,        // selected filter
                    QFileDialog::Options()        // options
                    );
#endif

}


QString
getOpenMeshName(QWidget*        _parent, 
		const QString&  _caption,
		const QString&  _start)
{
  return  
    ACG::getOpenFileName(_parent, 
			 _caption,
			 OpenMesh::IO::IOManager().qt_read_filters().c_str(),
			 _start);
}


//-----------------------------------------------------------------------------


QStringList
getOpenFileNames(QWidget*        _parent,
		 const QString&  _caption,
		 const QString&  _filter,
		 const QString&  _start)
{
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
  return
    QFileDialog::getOpenFileNames( _parent, // parent
				   _caption, // caption
				   _start, // dir
				   _filter, //_filter
				   0, // selected filter
				   0 // options
				   );
#else
    return
      QFileDialog::getOpenFileNames( _parent, // parent
                     _caption, // caption
                     _start, // dir
                     _filter, //_filter
                     0, // selected filter
                      QFileDialog::Options() // options
                     );
#endif
}


QStringList
getOpenMeshNames(QWidget*        _parent,
		 const QString&  _caption,
		 const QString&  _start)
{
  return
    ACG::getOpenFileNames(_parent, 
			  _caption,
			  OpenMesh::IO::IOManager().qt_read_filters().c_str(),
			  _start);
}


//-----------------------------------------------------------------------------


QString
getSaveFileName(QWidget*        _parent, 
		const QString&  _caption, 
		const QString&  _filter, 
		bool            _askOW,
		const QString&  _start)
{
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
  QString filename = 
    QFileDialog::getSaveFileName ( _parent, // parent
				   _caption, // caption
				   _start, // dir
				   _filter, // filter,
				   0, // selected filter
                   0 // options
				   );
#else
    QString filename =
      QFileDialog::getSaveFileName ( _parent, // parent
                     _caption, // caption
                     _start, // dir
                     _filter, // filter,
                     0, // selected filter
                     QFileDialog::Options() // options
                     );
#endif

  if (_askOW && !filename.isEmpty() && QFile(filename).exists())
  {
    QString s;
    s += QString("The file\n  ");
    s += filename;
    s += QString("\nalready exists.\n\n");
    s += QString("Do you want to overwrite it?");

    if (QMessageBox::warning(_parent, "Overwrite", s, QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes) != QMessageBox::Yes)
      return QString();
  }

  return filename;
}


QString
getSaveMeshName(QWidget*        _parent, 
		const QString&  _caption, 
		bool            _askOW,
		const QString&  _start)
{
  return
    ACG::getSaveFileName(_parent, 
			 _caption,
			 OpenMesh::IO::IOManager().
			 qt_write_filters().c_str(),
			 _askOW,
			 _start);
}


//=============================================================================
} // namespace ACG
//=============================================================================

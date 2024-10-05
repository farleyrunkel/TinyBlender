#ifndef MENUINTERFACE_H
#define MENUINTERFACE_H

#include <QAction>
#include <QMenuBar>

#include <boost/signals2/signal.hpp>

#include "SARibbonBar.h"

/// The Menu will be added inside the File Menu
#define FILEMENU tr("File")

/// The Menu will be added inside the View Menu
#define VIEWMENU tr("View")

/// The Menu will be added inside the Tools Menu
#define TOOLSMENU tr("Tools")

/// The Menu will be added inside the Algorithms Menu
#define ALGORITHMMENU tr("Algorithms")

/// The Menu will be added inside the Python Menu
#define PYTHONMENU tr("Python")


class MenuInterface 
{
    
public:

    /// Destructor
    virtual ~MenuInterface() {};


public:

	/// Signal to get the Ribbon Category
	boost::signals2::signal<void(QString, SARibbonCategory*&, bool)> signalGetRibbonCategory;

};


#define MenuInterface_iid "TinyBlender.MenuInterface"

Q_DECLARE_INTERFACE(MenuInterface, MenuInterface_iid)


#endif // MENUINTERFACE_H
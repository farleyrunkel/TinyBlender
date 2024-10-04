#ifndef MENUINTERFACE_H
#define MENUINTERFACE_H

#include <QAction>
#include <QMenuBar>


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


class MenuInterface {

public:

    /// Destructor
    virtual ~MenuInterface() {};

signals:

    /**  \brief Get a existing top level menu pointer or create a new one
     *
     *  Checks if a top level menu is present and creates one if needed \n
     *
     * @param _name   Menu name (see FILEMENU/VIEWMENU/TOOLSMENU example defines or use other QStrings )
     * @param _menu   The returned top level menu
     * @param _create Should a new menu be created if id doesn't exist
   */
    virtual void getMenubarMenu(QString _name, QMenu*& _menu, bool _create) {};

    /**  \brief Adds an action to the menubar
      *
      *  Add an action to one of the menubar top level menus \n
      * \n
      *   Example : \n
      * \code
      *   QMenu *colorMenu = new QMenu(tr("&Colors"));
      *   emit addMenubarAction( colorMenu->menuAction(), TOOLSMENU );
      * \endcode
      *
      * All actions or sub actions can be freely controlled by yourself. You have
      * to connect the required signals and slots to your plugin.
      *
      * @param _action Pointer to the new action
      * @param _name   Name of the menu
    */
    virtual void addMenubarAction(QAction* _action, QString _name) {};

    /** \brief Add multiple actions to the menu bar.
     *
     * Does the same as multiple calls to addMenubarAction except
     * it doesn't insert a separator between the individual items.
     *
     * @param _actions Vector of pointers to the new actions.
     * @param _name Name of the menu.
     */
    virtual void addMenubarActions(std::vector<QAction*>& _actions, QString _name) {};
};


#define MenuInterface_iid "TinyBlender.MenuInterface"

Q_DECLARE_INTERFACE(MenuInterface, MenuInterface_iid)


#endif // MENUINTERFACE_H
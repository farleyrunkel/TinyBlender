#ifndef COREWIDGET_H
#define COREWIDGET_H

#include <QMainWindow>
#include <QMenu>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabWidget>
#include <QOpenGLWidget>
#include <QGraphicsScene>

#include "SARibbonMainWindow.h"
#include "SARibbonBar.h"

#include "graphicsview.h"

class MainWindow : public SARibbonMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

signals:

private:
    void setupMainUi();
    void setupApplicationButton();
    void setupGraphicView();

    QAction *createAction(const QString &text, const QString &iconurl);

private:
    QMenu*          myAppButton;
    SARibbonBar*    myRibbonBar;
    QSplitter*      myMainSplitter;


    QStackedWidget* myStackedWidget;

    QOpenGLWidget*  myOpenGLWidget;

    GraphicsView*   myView;
    QGraphicsScene* myScene;

    QTabWidget*     mySideWidget;

    void setupCategories();
};

#endif // COREWIDGET_H

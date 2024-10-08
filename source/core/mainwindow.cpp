#include "core/mainwindow.h"

#include <QStatusBar>
#include <QTabBar>

#include "SARibbonMenu.h"
#include "SARibbonBar.h"
#include "SARibbonCategory.h"
#include "SARibbonPannel.h"
#include "SARibbonApplicationButton.h"

MainWindow::MainWindow(QWidget *parent)
    : SARibbonMainWindow(parent),
    myAppButton(nullptr),
    myMainSplitter(nullptr),
    myStackedWidget(nullptr),
    mySideWidget(nullptr)
{
    setupMainUi();

    setupApplicationButton();

    setupCategories();

    setupGraphicView();

}

void MainWindow::setupMainUi()
{
    resize(1260, 800);

    setWindowTitle(("TinyBlender"));
    setStatusBar(new QStatusBar());
    setWindowIcon(QIcon(":/icons/tinyblender.png"));

    // set ribbonbar
    myRibbonBar = ribbonBar();
    //! 通过setContentsMargins设置ribbon四周的间距
    myRibbonBar->setContentsMargins(5, 0, 5, 0);

    // set central widget
    myMainSplitter = new QSplitter(Qt::Horizontal,this);
    myMainSplitter->setHandleWidth(0);
    setCentralWidget(myMainSplitter);

    mySideWidget = new QTabWidget;
    mySideWidget->setTabPosition(QTabWidget::West);
    mySideWidget->addTab(new QWidget, QIcon(":/icon/save.svg"), "");
    mySideWidget->addTab(new QWidget, QIcon(":/icon/save.svg"), "");
    mySideWidget->addTab(new QWidget, QIcon(":/icon/save.svg"), "");
    mySideWidget->setIconSize(QSize(30, 30));

    myStackedWidget = new QStackedWidget;

    myMainSplitter->addWidget(mySideWidget);
    myMainSplitter->addWidget(myStackedWidget);
    myMainSplitter->setChildrenCollapsible(false);
    myMainSplitter->setSizes({1, 150});

}

void MainWindow::setupGraphicView()
{
    myView = new GraphicsView();

    myScene = new QGraphicsScene;

    myScene->addPixmap(QPixmap(":/icons/tinyblender.png"));

    myOpenGLWidget = new QOpenGLWidget(myView);

    myView->setViewport(myOpenGLWidget);
    myView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    myView->setScene(myScene);
    myView->setFrameStyle(QFrame::NoFrame);

    myStackedWidget->addWidget(myView);
}

void MainWindow::setupApplicationButton()
{
    if (!myRibbonBar) {
        return;
    }
    QAbstractButton* btn = myRibbonBar->applicationButton();
    if (!btn) {
        btn = new SARibbonApplicationButton(this);
        myRibbonBar->setApplicationButton(btn);
    }
    myRibbonBar->applicationButton()->setText(("  &File  "));  // 文字两边留有间距，好看一点

    if (!myAppButton) {
        myAppButton = new SARibbonMenu(this);
        myAppButton->addAction(createAction("test1", "://icon/action.svg"));
        myAppButton->addAction(createAction("test2", "://icon/action2.svg"));
        myAppButton->addAction(createAction("test3", "://icon/action3.svg"));
        myAppButton->addAction(createAction("test4", "://icon/action4.svg"));
    }
    SARibbonApplicationButton* appBtn = qobject_cast< SARibbonApplicationButton* >(btn);
    if (!appBtn) {
        return;
    }
    appBtn->setMenu(myAppButton);
}

void MainWindow::setupCategories()
{

    {
        //Add main tab - The main tab is added through the addcategorypage factory function.
        SARibbonCategory* categoryEdit = myRibbonBar->addCategoryPage(tr("Edit"));
            //Using the addpannel function to create saribponpannel. The effect is the same as that of new saribponpannel, and then call SARibbonCategory:: addpannel.

        {
            SARibbonPannel* aPannel = categoryEdit->addPannel(("Panel 1"));

            {
                QAction* aAction = new QAction;
                aAction->setText("save");
                aAction->setIcon(QIcon("://icon/save.svg"));
                aAction->setObjectName("actSave");
                aAction->setShortcut(QKeySequence(QLatin1String("Ctrl+S")));
                aPannel->addLargeAction(aAction);
            }
        }
    }

}


QAction* MainWindow::createAction(const QString& text, const QString& iconurl)
{
    QAction* act = new QAction(this);
    act->setText(text);
    act->setIcon(QIcon(iconurl));
    act->setObjectName(text);
    return act;
}

void MainWindow::slotGetRibbonCategory(QString name, SARibbonCategory*& menu, bool create)
{
    // if menu with name already exists, return it
    SARibbonCategory* category = myRibbonBar->categoryByName(name);
    if (category) {
        menu = category;
        return;
    }
    else if (create) {
        menu = myRibbonBar->addCategoryPage(name);
    }
    else {
        menu = nullptr;
    }
}

#include "corewidget.h"

#include <QStatusBar>
#include <QTabBar>

#include "SARibbonMenu.h"
#include "SARibbonBar.h"
#include "SARibbonCategory.h"
#include "SARibbonPannel.h"
#include "SARibbonApplicationButton.h"

CoreWidget::CoreWidget(QWidget *parent)
    : SARibbonMainWindow(parent),
    myAppButton(nullptr),
    myMainSplitter(nullptr),
    myStackedWidget(nullptr),
    mySideWidget(nullptr)
{
    resize(1260, 800);

    setWindowTitle(("TinyBlender"));
    setStatusBar(new QStatusBar());
    setWindowIcon(QIcon("://icons/tinyblender.png"));

    myMainSplitter = new QSplitter(Qt::Horizontal,this);
    myMainSplitter->setHandleWidth(0);
    setCentralWidget(myMainSplitter);

    mySideWidget = new QTabWidget;
    mySideWidget->setTabPosition(QTabWidget::West);
    mySideWidget->addTab(new QWidget, QIcon("://icon/save.svg"), "");
    mySideWidget->addTab(new QWidget, QIcon("://icon/save.svg"), "");
    mySideWidget->addTab(new QWidget, QIcon("://icon/save.svg"), "");
    mySideWidget->setIconSize(QSize(30, 30));

    myStackedWidget = new QStackedWidget;
    myMainSplitter->addWidget(mySideWidget);
    myMainSplitter->addWidget(myStackedWidget);
    myMainSplitter->setSizes({1, 150});
    myMainSplitter->setChildrenCollapsible(false);

    myRibbonBar = ribbonBar();
    //! 通过setContentsMargins设置ribbon四周的间距
    myRibbonBar->setContentsMargins(5, 0, 5, 0);

    createRibbonApplicationButton();

    {
        //Add main tab - The main tab is added through the addcategorypage factory function.
        SARibbonCategory* categoryEdit = myRibbonBar->addCategoryPage(tr("Edit"));
            //Using the addpannel function to create saribponpannel. The effect is the same as that of new saribponpannel, and then call SARibbonCategory:: addpannel.
        SARibbonPannel* pannel1 = categoryEdit->addPannel(("Panel 1"));
        QAction* actSave = new QAction(this);
        actSave->setText("save");
        actSave->setIcon(QIcon("://icon/save.svg"));
        actSave->setObjectName("actSave");
        actSave->setShortcut(QKeySequence(QLatin1String("Ctrl+S")));
        pannel1->addLargeAction(actSave);
    }

    {
        //Add main tab - The main tab is added through the addcategorypage factory function.
        SARibbonCategory* categorySelect = myRibbonBar->addCategoryPage(tr("Select"));
            //Using the addpannel function to create saribponpannel. The effect is the same as that of new saribponpannel, and then call SARibbonCategory:: addpannel.
        SARibbonPannel* pannel1 = categorySelect->addPannel(("Panel 1"));
        QAction* actSave = new QAction(this);
        actSave->setText("save");
        actSave->setIcon(QIcon("://icon/save.svg"));
        actSave->setObjectName("actSave");
        actSave->setShortcut(QKeySequence(QLatin1String("Ctrl+S")));
        pannel1->addLargeAction(actSave);
    }

}


void CoreWidget::createRibbonApplicationButton()
{
    if (!myRibbonBar) {
        return;
    }
    QAbstractButton* btn = myRibbonBar->applicationButton();
    if (!btn) {
        // cn: SARibbonBar默认就会创建一个SARibbonApplicationButton，因此，在正常情况下，这个位置不会进入
        // en: SARibbonBar creates a SARibbonApplicationButton by default. Therefore, under normal circumstances, this location will not enter
        btn = new SARibbonApplicationButton(this);
        myRibbonBar->setApplicationButton(btn);
    }
    myRibbonBar->applicationButton()->setText(("  &File  "));  // 文字两边留有间距，好看一点

    // cn: SARibbonMenu和QMenu的操作是一样的
    // en: The operations of SARibbonMenu and QMenu are the same
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

QAction* CoreWidget::createAction(const QString& text, const QString& iconurl)
{
    QAction* act = new QAction(this);
    act->setText(text);
    act->setIcon(QIcon(iconurl));
    act->setObjectName(text);
    return act;
}


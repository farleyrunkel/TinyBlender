#ifndef COREWIDGET_H
#define COREWIDGET_H

#include <QMainWindow>
#include <QMenu>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabWidget>

#include "SARibbonMainWindow.h"
#include "SARibbonBar.h"

class MainWindow : public SARibbonMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

signals:

private:
    void setupMainUi();
    void setupApplicationButton();

    QAction *createAction(const QString &text, const QString &iconurl);

private:
    QMenu*          myAppButton;
    SARibbonBar*    myRibbonBar;
    QSplitter*      myMainSplitter;
    QStackedWidget* myStackedWidget;
    QTabWidget*     mySideWidget;

};

#endif // COREWIDGET_H

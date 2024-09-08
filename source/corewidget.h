#ifndef COREWIDGET_H
#define COREWIDGET_H

#include <QMainWindow>
#include <QMenu>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabWidget>

#include "SARibbonMainWindow.h"
#include "SARibbonBar.h"

class CoreWidget : public SARibbonMainWindow
{
    Q_OBJECT
public:
    explicit CoreWidget(QWidget *parent = nullptr);

signals:
private:
    void createRibbonApplicationButton();

    QMenu* myAppButton;
    SARibbonBar* myRibbonBar;
    QSplitter* myMainSplitter;
    QStackedWidget* myStackedWidget;
    QTabWidget* mySideWidget;

    QAction *createAction(const QString &text, const QString &iconurl);
};

#endif // COREWIDGET_H

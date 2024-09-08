#ifndef COREWIDGET_H
#define COREWIDGET_H

#include <QMainWindow>
#include <QMenu>
#include <QSplitter>

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
    SARibbonBar* myRibbon;
    QSplitter* mySplitter;
    QAction *createAction(const QString &text, const QString &iconurl);
};

#endif // COREWIDGET_H

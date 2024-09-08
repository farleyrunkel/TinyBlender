#ifndef COREWIDGET_H
#define COREWIDGET_H

#include <QMainWindow>
#include <QMenu>

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

    QMenu* mMenuApplicationBtn;
    SARibbonBar* ribbon;
    QAction *createAction(const QString &text, const QString &iconurl);
};

#endif // COREWIDGET_H

#ifndef COREWIDGET_H
#define COREWIDGET_H

#include <QMainWindow>

class CoreWidget : public QMainWindow
{
    Q_OBJECT
public:
    explicit CoreWidget(QWidget *parent = nullptr);

signals:
};

#endif // COREWIDGET_H

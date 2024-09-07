#ifndef CORE_H
#define CORE_H

#include <QObject>
#include <QSplashScreen>

#include "corewidget.h"

class Core : public QObject
{
    Q_OBJECT
public:
    explicit Core(QObject *parent = nullptr);

    void init();

    void finishSplash();

signals:
private:
    CoreWidget *coreWidget_;
    QSplashScreen* splash_;

};

#endif // CORE_H

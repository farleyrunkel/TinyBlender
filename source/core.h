#ifndef CORE_H
#define CORE_H

#include <QObject>
#include <QSplashScreen>

#include "mainwindow.h"

class Core : public QObject
{
    Q_OBJECT
public:
    explicit Core(QObject *parent = nullptr);

    void init();

    void finishSplash();

signals:
private:
    MainWindow *myMainWindow;
    QSplashScreen* mySplashScreen;

};

#endif // CORE_H

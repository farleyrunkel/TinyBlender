#ifndef CORE_H
#define CORE_H

#include <QObject>
#include <QSplashScreen>
#include <QPluginLoader>

#include "mainwindow.h"
#include "baseinterface.h"


class Core : public QObject
{
    Q_OBJECT
public:
    explicit Core(QObject *parent = nullptr);

    void init();

    void finishSplash();

private:
    void loadPlugins();

	void loadPlugin(BaseInterface* plugin);

	void setupConnections();

signals:
private:
    MainWindow *myMainWindow;
    QSplashScreen* mySplashScreen;

};

#endif // CORE_H

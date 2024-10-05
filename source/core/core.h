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
    explicit Core(QObject* theParent = nullptr);

    void init();

    void finishSplash();

private:
    void loadPlugins();

    void loadPlugin(QObject* thePlugin);


	void setupConnections();

    bool checkSlot(QObject* thePlugin, const char* theSlotSignature);

    bool checkSignal(QObject* thePlugin, const char* theSlotSignature);


signals:

    void pluginsInitialized();


private:
    MainWindow *myMainWindow;
    QSplashScreen* mySplashScreen;

};

#endif // CORE_H

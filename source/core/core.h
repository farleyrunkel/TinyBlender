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

    void loadPlugin(QObject* plugin);


	void setupConnections();

    bool checkSlot(QObject* _plugin, const char* _slotSignature);

    bool checkSignal(QObject* _plugin, const char* _signalSignature);


signals:

    void pluginsInitialized();


private:
    MainWindow *myMainWindow;
    QSplashScreen* mySplashScreen;

};

#endif // CORE_H

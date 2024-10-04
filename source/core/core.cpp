#include "core.h"

#include <QEventLoop>
#include <QTimer>
#include <QApplication>

#include "settings.h"
#include "options.h"
#include "baseinterface.h"
#include "menuinterface.h"


Core::Core(QObject *parent)
    : QObject(parent),
    myMainWindow(nullptr),
    mySplashScreen(nullptr)
{}

void Core::init() {

    if (Settings::value("core/Gui/splash", true).toBool() ) {
       // QPixmap splashPixmap(TinyBlenderOptions::IconDirStr() + TinyBlenderOptions::dirSeparator() + "splash.png");
        QPixmap splashPixmap("://icons/splash.png");
        splashPixmap = splashPixmap.scaledToWidth(680);
        mySplashScreen = new QSplashScreen(splashPixmap, Qt::SplashScreen | Qt::WindowStaysOnTopHint);
        mySplashScreen->show();

        mySplashScreen->showMessage(tr("Initializing TinyBlender") ,
                             Qt::AlignTop | Qt::AlignLeft , Qt::white);

        QEventLoop* eventloop = new QEventLoop;
        QTimer::singleShot(2000, eventloop, &QEventLoop::quit);
        eventloop->exec();
    }

    myMainWindow = new MainWindow();
    myMainWindow->show();

    finishSplash();

    loadPlugins();

}

void Core::finishSplash() {
    if (mySplashScreen) {
        mySplashScreen->finish(myMainWindow);
        mySplashScreen->deleteLater();
        mySplashScreen = 0;
    }
}

void Core::loadPlugins() {
    const auto staticInstances = QPluginLoader::staticInstances();
    for (QObject* plugin : staticInstances) {

         loadPlugin(plugin);       
    }
}


void Core::loadPlugin(QObject* plugin) {

    BaseInterface* basePlugin = qobject_cast<BaseInterface*>(plugin);

    // Initialize Plugin
    if (basePlugin) {
        if (checkSlot(plugin, "initializePlugin()"))
            basePlugin->initializePlugin();
    }


    //Check if the plugin supports Menubar-Interface
    MenuInterface* menubarPlugin = qobject_cast<MenuInterface*>(plugin);

    if (menubarPlugin) {
        if (checkSignal(plugin, "getRibbonCategory(QString, SARibbonCategory*&, bool)"))
            connect(plugin, SIGNAL(getRibbonCategory(QString, SARibbonCategory*&, bool)), myMainWindow, SLOT(slotGetRibbonCategory(QString, SARibbonCategory*&, bool)), Qt::AutoConnection);
    }

    if (checkSlot(plugin, "pluginsInitialized()")) {
        connect(this, &Core::pluginsInitialized, [basePlugin]() {basePlugin->pluginsInitialized(); });
    }


    emit pluginsInitialized();
 }

void Core::setupConnections() {
      connect(myMainWindow, &MainWindow::destroyed, this, &Core::finishSplash);
  }

bool Core::checkSlot(QObject* _plugin, const char* _slotSignature) {
    const QMetaObject* meta = _plugin->metaObject();
    int id = meta->indexOfSlot(QMetaObject::normalizedSignature(_slotSignature));
    return (id != -1);
}


bool Core::checkSignal(QObject* _plugin, const char* _signalSignature) {
    const QMetaObject* meta = _plugin->metaObject();
    int id = meta->indexOfSignal(QMetaObject::normalizedSignature(_signalSignature));
    return (id != -1);
}
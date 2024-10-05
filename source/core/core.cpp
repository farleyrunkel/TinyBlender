#include "core.h"

#include <QEventLoop>
#include <QTimer>
#include <QApplication>

#include <boost/bind.hpp>

#include "settings.h"
#include "options.h"
#include "baseinterface.h"
#include "menuinterface.h"


Core::Core(QObject* theParent)
    : QObject(theParent),
    myMainWindow(nullptr),
    mySplashScreen(nullptr)
{}

void Core::init() {

    if (Settings::value("core/Gui/splash", true).toBool() ) {
       // QPixmap splashPixmap(TinyBlenderOptions::IconDirStr() + TinyBlenderOptions::dirSeparator() + "splash.png");
        QPixmap aSplashPixmap("://icons/splash.png");
        aSplashPixmap = aSplashPixmap.scaledToWidth(680);
        mySplashScreen = new QSplashScreen(aSplashPixmap, Qt::SplashScreen | Qt::WindowStaysOnTopHint);
        mySplashScreen->show();

        mySplashScreen->showMessage(tr("Initializing TinyBlender") ,
                             Qt::AlignTop | Qt::AlignLeft , Qt::white);

        QEventLoop* aEventloop = new QEventLoop;
        QTimer::singleShot(2000, aEventloop, &QEventLoop::quit);
        aEventloop->exec();
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
    const auto aStaticInstances = QPluginLoader::staticInstances();
    for (QObject* aPlugin : aStaticInstances) {
         loadPlugin(aPlugin);
    }
}


void Core::loadPlugin(QObject* thePlugin) {

    BaseInterface* aBasePlugin = qobject_cast<BaseInterface*>(thePlugin);

    // Initialize Plugin
    if (aBasePlugin) {
        aBasePlugin->initializePlugin();
        connect(this, &Core::pluginsInitialized, [aBasePlugin]() {aBasePlugin->pluginsInitialized();});
    }

    //Check if the plugin supports Menubar-Interface
    MenuInterface* aMenuPlugin = qobject_cast<MenuInterface*>(thePlugin);
    if (aMenuPlugin) {
        aMenuPlugin->signalGetRibbonCategory.connect(boost::bind(&MainWindow::slotGetRibbonCategory, myMainWindow, _1, _2, _3));
    }

    emit pluginsInitialized();
 }

void Core::setupConnections() {
      connect(myMainWindow, &MainWindow::destroyed, this, &Core::finishSplash);
  }

bool Core::checkSlot(QObject* thePlugin, const char* theSlotSignature) {
    const QMetaObject* meta = thePlugin->metaObject();
    int aIndex = meta->indexOfSlot(QMetaObject::normalizedSignature(theSlotSignature));
    return (aIndex != -1);
}


bool Core::checkSignal(QObject* thePlugin, const char* theSlotSignature) {
    const QMetaObject* aMeta = thePlugin->metaObject();
    int aIndex = aMeta->indexOfSignal(QMetaObject::normalizedSignature(theSlotSignature));
    return (aIndex != -1);
}
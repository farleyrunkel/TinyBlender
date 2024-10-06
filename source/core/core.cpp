#include "core.h"
#include "core.h"
#include "core.h"

#include <QEventLoop>
#include <QTimer>
#include <QApplication>
#include <QThread>

#include <boost/bind.hpp>

#include "settings.h"
#include "options.h"

#include "baseinterface.h"
#include "menuinterface.h"
#include "TypeInterface.h"


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


    TypeInterface* LoadSavePlugin = qobject_cast<TypeInterface*>(thePlugin);
    if (LoadSavePlugin) {
        LoadSavePlugin->signalAddEmptyObject.connect(boost::bind(&Core::slotAddEmptyObject, this, _1, _2));
    }

    emit pluginsInitialized();
 }

void Core::slotAddEmptyObject(int theType, int& theId) {

    if (QThread::currentThread() != QApplication::instance()->thread())
    {
        //execute method in main thread
        QMetaObject::invokeMethod(this, "slotAddEmptyObject", Qt::BlockingQueuedConnection, Q_ARG(DataType, theType), Q_ARG(int*, &theId));
    }
    else
    {
        theId = addEmptyObject(theType);
    }
}


int Core::addEmptyObject(DataType _type) {
    //// Iterate over all plugins. The first plugin supporting the addEmpty function for the
    //// specified type will be used to create the new object. If adding the object failed,
    //// we iterate over the remaining plugins.

    //// Iterate over type plugins
    //for (int i = 0; i < (int)supportedDataTypes_.size(); i++)
    //    if (supportedDataTypes_[i].type & _type) {
    //        int retCode = supportedDataTypes_[i].plugin->addEmpty();
    //        if (retCode != -1)
    //            return retCode;
    //    }

    return -1; // no plugin found
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
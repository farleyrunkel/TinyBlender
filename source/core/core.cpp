#include "core.h"

#include <QEventLoop>
#include <QTimer>

#include "settings.h"
#include "options.h"
#include "baseinterface.h"


Core::Core(QObject *parent)
    : QObject(parent),
    myMainWindow(nullptr),
    mySplashScreen(nullptr)
{}

void Core::init() {
	loadPlugins();

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
        BaseInterface* basePlugin = qobject_cast<BaseInterface*>(plugin);
        if (basePlugin) {
            loadPlugin(basePlugin);
        }
    }
}

void Core::loadPlugin(BaseInterface* plugin) {
     qDebug() << "Loading plugin: " << plugin->name();
 }

void Core::setupConnections() {
      connect(myMainWindow, &MainWindow::destroyed, this, &Core::finishSplash);
  }

#include "core.h"

#include <QEventLoop>
#include <QTimer>

#include "settings.h"
#include "options.h"

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
}

void Core::finishSplash() {
    if (mySplashScreen) {
        mySplashScreen->finish(myMainWindow);
        mySplashScreen->deleteLater();
        mySplashScreen = 0;
    }
}

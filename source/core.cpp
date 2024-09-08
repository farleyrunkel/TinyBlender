#include "core.h"

#include <QEventLoop>
#include <QTimer>

#include "tinyblendersettings.h"
#include "tinyblenderoptions.h"

Core::Core(QObject *parent)
    : QObject(parent),
    coreWidget_(nullptr),
    splash_(nullptr)
{}

void Core::init() {


    if (TinyBlenderSettings::value("core/Gui/splash", true).toBool() ) {
       // QPixmap splashPixmap(TinyBlenderOptions::IconDirStr() + TinyBlenderOptions::dirSeparator() + "splash.png");
        QPixmap splashPixmap("://icons/splash.png");
        splashPixmap = splashPixmap.scaledToWidth(680);
        splash_ = new QSplashScreen(splashPixmap, Qt::SplashScreen | Qt::WindowStaysOnTopHint);
        splash_->show();

        splash_->showMessage(tr("Initializing TinyBlender") ,
                             Qt::AlignTop | Qt::AlignLeft , Qt::white);

        QEventLoop* eventloop = new QEventLoop;
        QTimer::singleShot(2000, eventloop, &QEventLoop::quit);
        eventloop->exec();
    }

    coreWidget_ = new CoreWidget();
    coreWidget_->show();
    finishSplash();
}

void Core::finishSplash() {
    if (splash_) {
        splash_->finish(coreWidget_);
        splash_->deleteLater();
        splash_ = 0;
    }
}

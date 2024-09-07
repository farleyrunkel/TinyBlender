#include "core.h"
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

        splash_ = new QSplashScreen(splashPixmap, Qt::SplashScreen | Qt::WindowStaysOnTopHint);
        splash_->show();

        splash_->showMessage(tr("Initializing mainwindow") ,
                             Qt::AlignBottom | Qt::AlignLeft , Qt::white);
    }

    coreWidget_ = new CoreWidget();
    coreWidget_->show();


    // finishSplash();

}

void Core::finishSplash() {
    if (splash_) {
        splash_->finish(coreWidget_);
        splash_->deleteLater();
        splash_ = 0;
    }
}

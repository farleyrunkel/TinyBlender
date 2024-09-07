#include "tinyblenderoptions.h"


// Initialize the static member variable
QDir    TinyBlenderOptions::iconDir_      = QDir("://icons/splash.png");
QString TinyBlenderOptions::dirSeparator_ = "/";


QDir TinyBlenderOptions::iconDir()
{
    return iconDir_;
}

void TinyBlenderOptions::setIconDir(const QDir &newIconDir)
{
    iconDir_ = newIconDir;
}

QString TinyBlenderOptions::IconDirStr() {
    return iconDir_.absolutePath();
}

QString TinyBlenderOptions::dirSeparator()
{
    return dirSeparator_;
}

void TinyBlenderOptions::setDirSeparator(const QString &newDirSeparator)
{
    dirSeparator_ = newDirSeparator;
}

bool TinyBlenderOptions::initializeSettings() {
    return true;
}

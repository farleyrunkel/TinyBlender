#include "options.h"


// Initialize the static member variable
QDir    Options::myIconDir      = QDir("://icons/splash.png");
QString Options::myDirSeparator = "/";


QDir Options::iconDir()
{
    return myIconDir;
}

void Options::setIconDir(const QDir &newIconDir)
{
    myIconDir = newIconDir;
}

QString Options::IconDirStr() {
    return myIconDir.absolutePath();
}

QString Options::dirSeparator()
{
    return myDirSeparator;
}

void Options::setDirSeparator(const QString &newDirSeparator)
{
    myDirSeparator = newDirSeparator;
}

bool Options::initializeSettings() {
    return true;
}

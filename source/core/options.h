#ifndef TINYBLENDEROPTIONS_H
#define TINYBLENDEROPTIONS_H

#include <QDir>

class Options
{
public:

    static QDir iconDir();
    static void setIconDir(const QDir &newIconDir);
    static QString IconDirStr();

    static QString dirSeparator();
    static void setDirSeparator(const QString &newDirSeparator);

private:
    Options() = default;

    bool initializeSettings();

private:
    static QDir myIconDir;
    static QString myDirSeparator;
};

#endif // TINYBLENDEROPTIONS_H

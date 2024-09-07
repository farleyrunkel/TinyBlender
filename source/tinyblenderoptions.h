#ifndef TINYBLENDEROPTIONS_H
#define TINYBLENDEROPTIONS_H

#include <QDir>

class TinyBlenderOptions
{
public:

    static QDir iconDir();
    static void setIconDir(const QDir &newIconDir);
    static QString IconDirStr();

    static QString dirSeparator();
    static void setDirSeparator(const QString &newDirSeparator);


private:
    TinyBlenderOptions() = default;

    bool initializeSettings();

private:
    static QDir iconDir_;
    static QString dirSeparator_;
};

#endif // TINYBLENDEROPTIONS_H

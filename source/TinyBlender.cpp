#include <gl/glew.h>
#include <QApplication>
#include <QLocale>
#include <QTranslator>

#include "core.h"
#include "mainwindow.h"

#include "primitivesGenerator.h"

Q_IMPORT_PLUGIN(PrimitivesGenerator)

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "TinyBlender_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            a.installTranslator(&translator);
            break;
        }
    }

    Core w;
    w.init();

    return a.exec();
}

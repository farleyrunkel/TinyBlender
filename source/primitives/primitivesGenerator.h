#ifndef PRIMITIVESGENERATOR_H
#define PRIMITIVESGENERATOR_H

#include <QObject>
#include <QString>
#include <QPluginLoader>
#include <QStaticPlugin>

#include "baseinterface.h"
#include "menuinterface.h"

class PrimitivesGenerator : public QObject, public BaseInterface, public MenuInterface
{
	Q_OBJECT

	Q_PLUGIN_METADATA(IID "TinyBlender.Plugins.PrimitiveGenerator")

	Q_INTERFACES(BaseInterface)

public:
	QString name() const {
		return QString("PrimitivesGenerator"); 
	};

signals:


public slots:

	void initializePlugin() override { qDebug() << "initializePlugin PrimitivesGenerator;"; };

    void pluginsInitialized() override { qDebug() << "PrimitivesGenerator pluginsInitialized;"; };

};

#endif // !PRIMITIVESGENERATOR_H

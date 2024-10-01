#ifndef PRIMITIVESGENERATOR_H
#define PRIMITIVESGENERATOR_H

#include <QObject>
#include <QString>
#include <QPluginLoader>
#include <QStaticPlugin>

#include "baseinterface.h"


class PrimitivesGenerator : public QObject, public BaseInterface 
{
	Q_OBJECT

	Q_PLUGIN_METADATA(IID "TinyBlender.Plugins.PrimitiveGenerator")

	Q_INTERFACES(BaseInterface)

public:
	QString name() const {
		return QString("PrimitivesGenerator"); 
	};


};

#endif // !PRIMITIVESGENERATOR_H

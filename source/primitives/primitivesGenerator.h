#ifndef PRIMITIVESGENERATOR_H
#define PRIMITIVESGENERATOR_H

#include <QObject>
#include <QString>
#include <QPluginLoader>

#include "baseinterface.h"


class PrimitivesGenerator : public QObject, public BaseInterface {
	Q_OBJECT
	Q_INTERFACES(BaseInterface)
	Q_PLUGIN_METADATA(IID "TinyBlender.Plugins.PrimitiveGenerator")

public:
	QString name() const { 
		return QString("PrimitivesGenerator"); 
	};


};

#endif // !PRIMITIVESGENERATOR_H

// Q_IMPORT_PLUGIN(PrimitivesGenerator)

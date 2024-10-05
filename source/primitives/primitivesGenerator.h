#ifndef PRIMITIVESGENERATOR_H
#define PRIMITIVESGENERATOR_H

#include <QObject>
#include <QString>
#include <QPluginLoader>
#include <QStaticPlugin>

#include "SARibbonBar.h"

#include "baseinterface.h"
#include "menuinterface.h"

class PrimitivesGenerator : public QObject,
                            public BaseInterface,
                            public MenuInterface
{
	Q_OBJECT

	Q_PLUGIN_METADATA(IID "TinyBlender.Plugins.PrimitiveGenerator")

	Q_INTERFACES(BaseInterface MenuInterface)

public:
	QString name() const override;;

signals:

    void getRibbonCategory(QString name, SARibbonCategory*& menu, bool create);

public slots:

	void initializePlugin() override;

    void pluginsInitialized() override;

public slots:

	QString version() { return QString("1.0"); };


	int addSphere(const Vector& _position = Vector(0.0, 0.0, 0.0),
		const double _radius = 1.0);

private:
    SARibbonCategory* myPrimitivesMenu = nullptr;

};

#endif // !PRIMITIVESGENERATOR_H

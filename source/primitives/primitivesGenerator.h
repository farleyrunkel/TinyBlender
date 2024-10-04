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
	QString name() const {
		return QString("PrimitivesGenerator"); 
	};

signals:

    void getRibbonCategory(QString name, SARibbonCategory*& menu, bool create);

public slots:

	void initializePlugin() override { qDebug() << "initializePlugin PrimitivesGenerator;"; };

	void pluginsInitialized() override {

		qDebug() << "pluginsInitialized PrimitivesGenerator;";
		
		emit getRibbonCategory(tr("&Primitives"), primitivesMenu_, true);

        {
            SARibbonPannel* aPrimPannel = primitivesMenu_->addPannel(("Primitives"));
            {
                QAction* aAction = new QAction(QIcon("://icons/plane.svg"), "Plane");
                aPrimPannel->addLargeAction(aAction);
            }
            {
                QAction* aAction = new QAction(QIcon("://icons/cube.svg"), "Cube", aPrimPannel);
                aPrimPannel->addLargeAction(aAction);
            }
            {
                QAction* aAction = new QAction(QIcon("://icons/circle.svg"), "Circle", aPrimPannel);
                aPrimPannel->addLargeAction(aAction);
            }
            {
                QAction* aAction = new QAction(QIcon("://icons/sphere.svg"), "Sphere", aPrimPannel);
                aPrimPannel->addLargeAction(aAction);
            }
            {
                QAction* aAction = new QAction(QIcon("://icons/cylinder.svg"), "Cylinder", aPrimPannel);
                aPrimPannel->addLargeAction(aAction);
            }
            {
                QAction* aAction = new QAction(QIcon("://icons/cone.svg"), "Cone", aPrimPannel);
                aPrimPannel->addLargeAction(aAction);
            }
            {
                QAction* aAction = new QAction(QIcon("://icons/torus.svg"), "Torus", aPrimPannel);
                aPrimPannel->addLargeAction(aAction);
            }
        }

		qDebug() << "PrimitivesGenerator pluginsInitialized;"; 
	};


private:
    SARibbonCategory* primitivesMenu_ = nullptr;

};

#endif // !PRIMITIVESGENERATOR_H

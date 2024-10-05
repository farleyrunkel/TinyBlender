#include "primitivesGenerator.h"


QString PrimitivesGenerator::name() const {
    return QString("PrimitivesGenerator");
}

void PrimitivesGenerator::initializePlugin() {
	qDebug() << "initializePlugin PrimitivesGenerator;";
}

void PrimitivesGenerator::pluginsInitialized() {

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
}


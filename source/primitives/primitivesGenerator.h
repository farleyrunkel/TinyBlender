#ifndef PRIMITIVESGENERATOR_H
#define PRIMITIVESGENERATOR_H

#include <QObject>
#include <QString>
#include <QPluginLoader>
#include <QStaticPlugin>

#include <OpenMesh/Core/Mesh/DefaultPolyMesh.hh>
#include <OpenMesh/Core/Mesh/DefaultTriMesh.hh>

#include "SARibbonBar.h"

#include "interface/baseinterface.h"
#include "interface/menuinterface.h"
#include "interface/fileinterface.h"

#include "math/DataTypes.h"
#include "common/objectmanager.h"

#include "objects/OpenMesh/polymeshobject.h"


class PrimitivesGenerator : public QObject,
                            public BaseInterface,
                            public MenuInterface,
                            public FileInterface
{
	Q_OBJECT

	Q_PLUGIN_METADATA(IID "TinyBlender.Plugins.PrimitiveGenerator")

	Q_INTERFACES(BaseInterface MenuInterface FileInterface)

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
		const double _radius = 1.0) {
		return 1;
	};
    int addPolyMesh() {
        int objectId = -1;

        toAddEmptyObject("PolyMeshObject", objectId);

        PolyMeshObject* object;
        //if (!PluginFunctions::getObject(objectId, object)) {
        //    emit log(LOGERR, "Unable to create new Object");
        //    return -1;
        //}

        return objectId;
    }

    int addCube(const Vector& _position, const double _length) {
        int newObject = addPolyMesh();

        BaseObject* aObject = ObjectManager::getObject(newObject);
        if (aObject == nullptr) {
            return -1;
        }
        PolyMeshObject* object = dynamic_cast<PolyMeshObject*>(aObject);

        return -1;
    }


private:
    SARibbonCategory* myPrimitivesMenu = nullptr;

    OpenMesh::TriMesh* triMesh_;
    OpenMesh::PolyMesh* polyMesh_;

};

#endif // !PRIMITIVESGENERATOR_H

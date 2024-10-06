#ifndef PRIMITIVESGENERATOR_H
#define PRIMITIVESGENERATOR_H

#include <QObject>
#include <QString>
#include <QPluginLoader>
#include <QStaticPlugin>

#include "SARibbonBar.h"

#include "baseinterface.h"
#include "menuinterface.h"
#include "objectinterface.h"

#include "DataTypes.h"

class PrimitivesGenerator : public QObject,
                            public BaseInterface,
                            public MenuInterface,
                            public ObjectInterface
{
	Q_OBJECT

	Q_PLUGIN_METADATA(IID "TinyBlender.Plugins.PrimitiveGenerator")

	Q_INTERFACES(BaseInterface MenuInterface ObjectInterface)

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

        signalAddEmptyObject(1, objectId);

        //PolyMeshObject* object;
        //if (!PluginFunctions::getObject(objectId, object)) {
        //    emit log(LOGERR, "Unable to create new Object");
        //    return -1;
        //}

        return objectId;
    }

    //int addCube(const Vector& _position, const double _length) {
    //    int newObject = addPolyMesh();

    //    PolyMeshObject* object;
    //    if (!PluginFunctions::getObject(newObject, object)) {
    //        emit log(LOGERR, "Unable to create new Object");
    //        return -1;
    //    }
    //    else {

    //        object->setName("Cube " + QString::number(newObject));

    //        polyMesh_ = object->mesh();
    //        polyMesh_->clear();

    //        // Add 8 vertices
    //        vhandles_.resize(8);

    //        const double halfSize = 0.5 * _length;
    //        //   6------5
    //        //  /|     /|
    //        // 2------1 |
    //        // | |    | |
    //        // | 7----|-4
    //        // |/     |/
    //        // 3------0
    //        vhandles_[0] = polyMesh_->add_vertex(PolyMesh::Point(halfSize, -halfSize, halfSize) + _position);
    //        vhandles_[1] = polyMesh_->add_vertex(PolyMesh::Point(halfSize, halfSize, halfSize) + _position);
    //        vhandles_[2] = polyMesh_->add_vertex(PolyMesh::Point(-halfSize, halfSize, halfSize) + _position);
    //        vhandles_[3] = polyMesh_->add_vertex(PolyMesh::Point(-halfSize, -halfSize, halfSize) + _position);
    //        vhandles_[4] = polyMesh_->add_vertex(PolyMesh::Point(halfSize, -halfSize, -halfSize) + _position);
    //        vhandles_[5] = polyMesh_->add_vertex(PolyMesh::Point(halfSize, halfSize, -halfSize) + _position);
    //        vhandles_[6] = polyMesh_->add_vertex(PolyMesh::Point(-halfSize, halfSize, -halfSize) + _position);
    //        vhandles_[7] = polyMesh_->add_vertex(PolyMesh::Point(-halfSize, -halfSize, -halfSize) + _position);

    //        // Add faces
    //        add_face(0, 1, 2, 3);
    //        add_face(0, 4, 5, 1);
    //        add_face(4, 7, 6, 5);
    //        add_face(7, 3, 2, 6);
    //        add_face(1, 5, 6, 2);
    //        add_face(0, 3, 7, 4);

    //        polyMesh_->update_normals();

    //        emit updatedObject(newObject, UPDATE_ALL);
    //        emit createBackup(newObject, "Original Object");

    //        PluginFunctions::viewAll();

    //        return newObject;
    //    }

    //    return -1;
    //}


private:
    SARibbonCategory* myPrimitivesMenu = nullptr;

};

#endif // !PRIMITIVESGENERATOR_H

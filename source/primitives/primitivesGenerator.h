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

#include "common/objectmanager.h"
#include "common/DataTypes.hh"

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

        polyMesh_ = object->mesh();
        polyMesh_->clear();

        // Add 8 vertices
        vhandles_.resize(8);

        const double halfSize = 0.5 * _length;
        //   6------5
        //  /|     /|
        // 2------1 |
        // | |    | |
        // | 7----|-4
        // |/     |/
        // 3------0
        vhandles_[0] = polyMesh_->add_vertex(OpenMesh::PolyMesh::Point(halfSize, -halfSize, halfSize) + _position);
        vhandles_[1] = polyMesh_->add_vertex(OpenMesh::PolyMesh::Point(halfSize, halfSize, halfSize) + _position);
        vhandles_[2] = polyMesh_->add_vertex(OpenMesh::PolyMesh::Point(-halfSize, halfSize, halfSize) + _position);
        vhandles_[3] = polyMesh_->add_vertex(OpenMesh::PolyMesh::Point(-halfSize, -halfSize, halfSize) + _position);
        vhandles_[4] = polyMesh_->add_vertex(OpenMesh::PolyMesh::Point(halfSize, -halfSize, -halfSize) + _position);
        vhandles_[5] = polyMesh_->add_vertex(OpenMesh::PolyMesh::Point(halfSize, halfSize, -halfSize) + _position);
        vhandles_[6] = polyMesh_->add_vertex(OpenMesh::PolyMesh::Point(-halfSize, halfSize, -halfSize) + _position);
        vhandles_[7] = polyMesh_->add_vertex(OpenMesh::PolyMesh::Point(-halfSize, -halfSize, -halfSize) + _position);

        // Add faces
        add_face(0, 1, 2, 3);
        add_face(0, 4, 5, 1);
        add_face(4, 7, 6, 5);
        add_face(7, 3, 2, 6);
        add_face(1, 5, 6, 2);
        add_face(0, 3, 7, 4);

        polyMesh_->update_normals();

        return -1;
    }

    void add_face(int _vh1, int _vh2, int _vh3, int _vh4)
    {
        polyMesh_->add_face(
            static_cast<OpenMesh::PolyMesh::VertexHandle>(_vh1),
            static_cast<OpenMesh::PolyMesh::VertexHandle>(_vh2),
            static_cast<OpenMesh::PolyMesh::VertexHandle>(_vh3),
            static_cast<OpenMesh::PolyMesh::VertexHandle>(_vh4)
        );
    }

private:
    SARibbonCategory* myPrimitivesMenu = nullptr;

    OpenMesh::TriMesh* triMesh_;
    OpenMesh::PolyMesh* polyMesh_;

    std::vector<OpenMesh::TriMesh::VertexHandle> vhandles_;
    std::vector<OpenMesh::PolyMesh::VertexHandle> vphandles_;

};

#endif // !PRIMITIVESGENERATOR_H

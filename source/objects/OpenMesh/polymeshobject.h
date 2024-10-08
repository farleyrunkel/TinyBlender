
#ifndef TB_POLYMESHOBJECT_H
#define TB_POLYMESHOBJECT_H

#include "common/objectfactory.h"
#include "objects/OpenMesh/meshobject.h"

#include <OpenMesh/Core/Mesh/DefaultPolyMesh.hh>


class PolyMeshObject : public MeshObject<OpenMesh::PolyMesh>
{
    
public:
    PolyMeshObject() : MeshObject<OpenMesh::PolyMesh>() {};
    PolyMeshObject(const PolyMeshObject& _object) : MeshObject<OpenMesh::PolyMesh>(_object) {};

    // PolyMeshObject(DataType _typeId);

    /// destructor
    virtual ~PolyMeshObject() {} ;

    virtual std::string name() const override { return autoRegister; }

private:
    static std::string autoRegister;  // Static variable for automatic registration
};


#endif // ! TB_POLYMESHOBJECT_H

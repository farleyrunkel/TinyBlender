
#include "objects/OpenMesh/polymeshobject.h"

// Static variable definition with lambda for automatic registration
std::string PolyMeshObject::autoRegister = [](const std::string& name) {
    ObjectFactory::registerObject(name, []()->BaseObject* {return new PolyMeshObject; });
    return name;
    }("PolyMeshObject");
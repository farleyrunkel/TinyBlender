
#pragma once

#include "common/sceneobject.h"


template < class MeshT >
class MeshObject : public SceneObject 
{
	
public:
	MeshObject() : SceneObject() {};

    MeshObject(const MeshObject& _object) : SceneObject(_object) {};

    /// destructor
    virtual ~MeshObject() {};

    MeshT* mesh() { return mesh_; };

    virtual std::string name() const override {return "MeshObject"; };

private:
    /// pointer to the mesh
    MeshT* mesh_;

};
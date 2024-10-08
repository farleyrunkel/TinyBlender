
#ifndef TINYBLENDER_SCENEOBJECT_H
#define TINYBLENDER_SCENEOBJECT_H

#include "common/baseobject.h"

class SceneObject : public BaseObject
{
    
public:

    SceneObject(const SceneObject& _object) : BaseObject(_object){};

    /// constructor
    SceneObject() : BaseObject() {} ;

    ///destructor
    virtual ~SceneObject() {};

    virtual std::string name() const { return "SceneObject"; };

private:
    /** This function creates the basic scenegraph nodes */
    void initializeScenegraphNodes() {} ;

};

#endif // !TINYBLENDER_SCENEOBJECT_H

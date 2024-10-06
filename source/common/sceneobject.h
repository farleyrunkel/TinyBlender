
#ifndef TINYBLENDER_SCENEOBJECT_H
#define TINYBLENDER_SCENEOBJECT_H

#include "baseobject.h"


class SceneObject : public BaseObject
{
    Q_OBJECT

public:

    SceneObject(const SceneObject& _object) {};

    /// constructor
    SceneObject() {} ;

    ///destructor
    virtual ~SceneObject() {};

private:
    /** This function creates the basic scenegraph nodes */
    void initializeScenegraphNodes() {} ;

};
#endif // !TINYBLENDER_SCENEOBJECT_H

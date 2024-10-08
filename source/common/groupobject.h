#ifndef TINYBLENDER_GROUPOBJECT_H
#define TINYBLENDER_GROUPOBJECT_H

#include <string>

#include "common/baseobject.h"
#include "common/objectfactory.h"

class GroupObject : public BaseObject
{
	
public:

    GroupObject(const GroupObject& _object) :
        BaseObject(_object) {}

    GroupObject(QString _groupName = "Group", GroupObject* _parent = 0) :
        BaseObject(_parent) {}

    /// destructor
    virtual ~GroupObject() {}

    std::string name() const override { return autoRegister; }

private:
    static std::string autoRegister;  // Static variable for automatic registration
};

// Static variable definition with lambda for automatic registration
std::string GroupObject::autoRegister = [](const std::string& name) {
    ObjectFactory::registerObject(name, []()->BaseObject* {return new GroupObject; });
    return name;
    }("GroupObject");

#endif // TINYBLENDER_GROUPOBJECT_H
#include "common/baseobject.h"
#include "common/objectmanager.h"


// Copy constructor: Assign a new unique ID and add object to ObjectManager
BaseObject::BaseObject(const BaseObject& theObject) : myID(myNextID++) {
    ObjectManager::addObjectToMap(id(), this);
}

// Constructor with an optional parent: Assign a new unique ID and add object to ObjectManager
BaseObject::BaseObject(BaseObject* theParent) : myID(myNextID++) {
    ObjectManager::addObjectToMap(id(), this);
}

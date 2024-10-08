#ifndef TB_OBJECTMANAGER_H
#define TB_OBJECTMANAGER_H

#include <vector>
#include <map>
#include <string>
#include <algorithm> // For std::remove

#include "common/baseobject.h"

class ObjectManager {

public:
    // Public static methods to access and modify the static data members
    static void setDataRoot(BaseObject* root);

    static BaseObject* getDataRoot();

    // Static methods to manage object map
    static void addObject(int id, BaseObject* object);

    static void removeObject(int id);

    static BaseObject* getObject(int id);

    static bool getObject(const int _identifier, BaseObject*& _object) {
        _object = getObject(_identifier);
        return _object != nullptr;
    }

    static void addObjectToMap(int _objectId, BaseObject* _object);

private:
    // Inline static member variables with internal initialization
    inline static BaseObject* objectRoot_ = nullptr;       // Pointer to the root of the data tree
    inline static std::map<int, BaseObject*> objectMap_;   // Map of object handles
};

#endif // TB_OBJECTMANAGER_H

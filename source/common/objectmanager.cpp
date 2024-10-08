#include "common/objectmanager.h"

// Set the data root object
void ObjectManager::setDataRoot(BaseObject* root) {
    objectRoot_ = root;
}

// Get the data root object
BaseObject* ObjectManager::getDataRoot() {
    return objectRoot_;
}

// Add an object to the object map
void ObjectManager::addObject(int id, BaseObject* object) {
    objectMap_[id] = object;
}

// Remove an object from the object map
void ObjectManager::removeObject(int id) {
    objectMap_.erase(id);
}

// Get an object by its ID from the object map
BaseObject* ObjectManager::getObject(int id) {
    auto it = objectMap_.find(id);
    if (it != objectMap_.end()) {
        return it->second;
    }
    return nullptr; // Return nullptr if object not found
}

// Add object to map with a check if it already exists
void ObjectManager::addObjectToMap(int _objectId, BaseObject* _object) {
    // Use emplace to only insert if the key doesn't already exist
    auto [it, inserted] = objectMap_.emplace(_objectId, _object);

    // If insertion failed (object with the same ID already exists), return
    if (!inserted) return;
}

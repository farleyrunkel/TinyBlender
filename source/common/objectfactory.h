#pragma once

#include <string>
#include <map>
#include <functional>

#include "common/baseobject.h"


class ObjectFactory {
public:

    virtual ~ObjectFactory() = 0 {};

    // Registers a object with the factory
    static void registerObject(const std::string& type, std::function<BaseObject* ()> creator) {
        objectMap[type] = creator;
    }

    // Creates a object based on the registered type
    static BaseObject* createObject(const std::string& type) {
        auto it = objectMap.find(type);
        if (it != objectMap.end()) {
            return it->second();  // Call the creator function
        }
        return nullptr; // Invalid product type
    }

private:
    // Static map to store product creators
    inline static std::map<std::string, std::function<BaseObject* ()>> objectMap = {};
};

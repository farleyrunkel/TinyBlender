#ifndef TINYBLENDER_BASEOBJECT_H
#define TINYBLENDER_BASEOBJECT_H

#include <string>

class BaseObject {

public:
    // Copy constructor
    BaseObject(const BaseObject& theObject);

    // Constructor with an optional parent
    explicit BaseObject(BaseObject* theParent = nullptr);

    // Virtual destructor
    virtual ~BaseObject() = default;

    // Pure virtual function for returning the object's name
    virtual std::string name() const = 0;
    // Get the object's unique ID
    int id() const { return myID; };


private:
    int myID;  // Object's unique ID

    // Static variable for generating unique IDs
    inline static int myNextID = 1;
};

#endif // TINYBLENDER_BASEOBJECT_H

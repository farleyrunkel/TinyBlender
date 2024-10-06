#ifndef TINYBLENDER_GROUPOBJECT_H
#define TINYBLENDER_GROUPOBJECT_H

#include "BaseObject.h"


class GroupObject : public BaseObject
{
    Q_OBJECT
public:

    GroupObject(const GroupObject& _object) {};


    /** constructor
     *
     * @param _groupName Name of the new Group object
     * @param _parent    The parent object of this object
     */
    GroupObject(QString _groupName = "Group", GroupObject* _parent = 0) {};

    /// destructor
    virtual ~GroupObject() {};

    /** return a full copy of this object. The object will not be a part of the object tree.
     *  This has to be done with the setParent() function.
     */
    BaseObject* copy() {};


};


#endif // TINYBLENDER_GROUPOBJECT_H
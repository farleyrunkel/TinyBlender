#ifndef TINYBLENDER_BASEOBJECT_H
#define TINYBLENDER_BASEOBJECT_H

#include <QObject>


class BaseObject : public QObject
{
    Q_OBJECT
public:

    BaseObject(const BaseObject& theObject) : QObject() {};

    explicit BaseObject(BaseObject* theParent = 0) : QObject() {};

    virtual ~BaseObject() {};


};


#endif // TINYBLENDER_BASEOBJECT_H
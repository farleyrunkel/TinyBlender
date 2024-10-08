#pragma once

#include <QString>
#include <QStringList>
#include "core/signal.h"

typedef unsigned int DataType;

class ObjectInterface 
{
public:
    virtual ~ObjectInterface() {}

public:

    virtual bool registerType() = 0;

    virtual int addEmpty() = 0;

    virtual DataType supportedType() = 0;

    //  virtual void generateBackup(int _id, QString _name, UpdateType _type) {};

public:

    signal<void(DataType, int&)> signalAddEmptyObject;
};

// Interface identifier macro
#define ObjectInterface_iid "TinyBlender.Interface.ObjectInterface"

Q_DECLARE_INTERFACE(ObjectInterface, ObjectInterface_iid)

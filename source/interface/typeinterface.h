#pragma once

#include <QString>
#include <QStringList>
#include "signal.h"

typedef unsigned int DataType;

class TypeInterface 
{
public:
    virtual ~TypeInterface() {}

public:

    virtual bool registerType() = 0;

    virtual int addEmpty() = 0;

    virtual DataType supportedType() = 0;

    //  virtual void generateBackup(int _id, QString _name, UpdateType _type) {};

public:

    signal<void(DataType, int&)> signalAddEmptyObject;
};

// Interface identifier macro
#define TypeInterface_iid "TinyBlender.Interface.TypeInterface"

Q_DECLARE_INTERFACE(TypeInterface, TypeInterface_iid)

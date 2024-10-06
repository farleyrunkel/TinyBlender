#pragma once

#include <QString>
#include <QStringList>
#include "signal.h"

typedef unsigned int DataType;

class ObjectInterface
{
public:
    virtual ~ObjectInterface() {};

public:
    virtual void slotFileOpened(int _id) {};
    virtual void slotAddedEmptyObject(int _id) {};
    virtual void slotObjectDeleted(int _id) {};

public:
    // Signals
    signal <void(int, QString)>             signalSave;
    signal <void(QString, DataType, int&)>  signalLoad;
    signal <void(DataType, int&)>           signalAddEmptyObject;
    signal <void(int, int&)>                signalCopyObject;
    // Deprecated signal (moved to Type Interface)
    signal <void(int)>                      signalEmptyObjectAddedDeprecated;
    signal <void(int)>                      signalDeleteObject;
    signal <void()>                         signalDeleteAllObjects;
    signal <void(QStringList&)>             signalGetAllFileFilters;

};


#define ObjectInterface_iid "TinyBlender.Interface.ObjectInterface"

Q_DECLARE_INTERFACE(ObjectInterface, ObjectInterface_iid)

#pragma once

#include <QString>
#include <QStringList>
#include "core/signal.h"

typedef unsigned int DataType;

class FileInterface
{
public:
    virtual ~FileInterface() {};

public:
    virtual void slotFileOpened(int _id) {};
    virtual void slotAddedEmptyObject(int _id) {};
    virtual void slotObjectDeleted(int _id) {};

public:
    // Signals
    signal <void(int, QString)>             signalSave;
    signal <void(QString, DataType, int&)>  signalLoad;
    signal <void(QString, int&)>            toAddEmptyObject;
    signal <void(int, int&)>                signalCopyObject;
    // Deprecated signal (moved to Type Interface)
    signal <void(int)>                      signalEmptyObjectAddedDeprecated;
    signal <void(int)>                      signalDeleteObject;
    signal <void()>                         signalDeleteAllObjects;
    signal <void(QStringList&)>             signalGetAllFileFilters;

};


#define FileInterface_iid "TinyBlender.Interface.FileInterface"

Q_DECLARE_INTERFACE(FileInterface, FileInterface_iid)

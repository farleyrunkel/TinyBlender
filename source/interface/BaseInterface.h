#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

#include <QObject>
#include <QString>
#include <QtPlugin>

#include "core/signal.h"

class BaseInterface 
{

public:
    virtual ~BaseInterface() {}

public:

    virtual QString name() const = 0;

    virtual void initializePlugin() {};

    virtual void pluginsInitialized() {};

public:
    signal <void()> toUpdateView;
};


#define BaseInterface_iid "TinyBlender.Interface.BaseInterface"

Q_DECLARE_INTERFACE(BaseInterface, BaseInterface_iid)


#endif // BASE_INTERFACE_H


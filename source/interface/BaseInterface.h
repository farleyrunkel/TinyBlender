#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

#include <QObject>
#include <QString>
#include <QtPlugin>

class BaseInterface 
{

public:

    virtual QString name() const = 0;

    virtual void initializePlugin() {};

    virtual void pluginsInitialized() {};

public:
    virtual ~BaseInterface() {}
};


#define BaseInterface_iid "TinyBlender.Interface.BaseInterface"

Q_DECLARE_INTERFACE(BaseInterface, BaseInterface_iid)


#endif // BASE_INTERFACE_H


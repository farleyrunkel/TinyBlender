#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

#include <QObject>
#include <QString>
#include <QtPlugin>

class BaseInterface {

public:
    virtual ~BaseInterface() {}
    virtual QString name() const = 0;

public:

    virtual void initializePlugin() {};

    virtual void pluginsInitialized() {};

};


#define BaseInterface_iid "TinyBlender.BaseInterface"

Q_DECLARE_INTERFACE(BaseInterface, BaseInterface_iid)


#endif // BASE_INTERFACE_H


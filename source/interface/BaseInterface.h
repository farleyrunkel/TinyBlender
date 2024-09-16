#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

#include <QString>

class BaseInterface {
public:
    virtual ~BaseInterface() {}
    virtual QString name() const = 0;
};


#endif // BASE_INTERFACE_H

Q_DECLARE_INTERFACE(BaseInterface, "TinyBlender.BaseInterface")

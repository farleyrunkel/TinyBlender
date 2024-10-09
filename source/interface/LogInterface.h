#ifndef TB_LOGINTERFACE_H
#define TB_LOGINTERFACE_H

#include <QString>
#include "core/signal.h"

class LogInterface
{
public:
    virtual ~LogInterface() {};

public:
    // Signals
    /** Send a log message to the main window of the widget.
     *  This signal can be called from any thread.
     *
     *  @param _type Message type (LOGINFO, LOGOUT, LOGWARN, LOGERR)
     *  @param _message Message to be displayed.
     */
    signal<void(Logtype _type, QString _message)> logMessage;

    /** Send a log message to the main window of the widget.
     *  Defaults to LOGOUT message type.
     *  This signal can be called from any thread.
     *
     *  @param _message Message to be displayed.
     */
    signal<void(QString _message)> logMessageDefault;

private slots:
    /** Through this slot you can receive all logging information emitted by OpenFlipper
     *  or one of its plugins.
     *
     *  @param _type    Message type.
     *  @param _message Message.
     */
    virtual void logOutput(Logtype _type, QString _message) {};

public:
    /// Destructor
    virtual ~LogInterface() {};
};

#define LogInterface_iid "OpenFlipper.LogInterface/1.0"

Q_DECLARE_INTERFACE(LogInterface, LogInterface_iid)

#endif // TB_LOGINTERFACE_H

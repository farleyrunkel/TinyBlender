
#pragma once

#include <iostream>

#include <QObject>

#include "LogType.h"


class Logger : public QObject {
  
  Q_OBJECT
  
  public:
    
    /// Standard Constructor 
    PluginLogger(const QString &_pluginName , Logtype _type = LOGOUT);
    
    /// Destructor
    ~Logger();
  
    /// Provide operator for streaming 
    void operator<< ( const std::string& _s ); 
    
  private slots:
    
    /// Log messages with a given Logtype
    void slotLog(Logtype _type, QString _message);
    
    /// Wrapper which logs at LOGOUT Logtype by default
    void slotLog(QString _message);
    
  signals:
    /// Sends the generated logs to the core
    void log(Logtype , QString);
    
  private:
    /// prepend this name to all output messages
    QString pluginName_;
    
    /// Default logtype ( used for streams )
    Logtype defaultLogType_;
    
};

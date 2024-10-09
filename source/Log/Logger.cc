
#include "Logger.hh"

#include <QStringList>


//== IMPLEMENTATION ==========================================================

/** 
 * @param _pluginName Name of the Plugin which uses this logger (Prepended to all Output)
 * @param _type Default log type ( used for streaming operations )
 */
Logger::PluginLogger(const QString& _pluginName , Logtype _type) :
    pluginName_(_pluginName),
    defaultLogType_(_type)
{
}

Logger::~Logger() {
}

/** Receive log events from plugins and pass them to core
 * @param _type Logtype (defines the color of the output)
 * @param _message The message for output
 */
void Logger::slotLog(Logtype _type, QString _message) {
  QStringList strings;
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
  strings = _message.split("\n",QString::SkipEmptyParts);
#else
  strings = _message.split("\n",Qt::SkipEmptyParts);
#endif

  for ( int i = 0 ; i < strings.size(); ++i )
    emit log(_type,pluginName_ + " : " + strings[i]);
}

/** Receive log events from plugins (defaults to LOGOUT Message type)
 * 
 * @param _message The message for output
 */
void Logger::slotLog(QString _message) {
  QStringList strings;
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
  strings = _message.split("\n",QString::SkipEmptyParts);
#else
  strings = _message.split("\n",Qt::SkipEmptyParts);
#endif

  for ( int i = 0 ; i < strings.size(); ++i )
    emit log(LOGOUT,pluginName_ + " : " + strings[i]);
}

/** This operator is used for streaming ( e.g. catch omerr and omerr streams) 
 *  and redirect them to the logging widget. The default log level passed to
 *  the constructor will be used for the message
 * 
 * @param _s String to display
 */
void Logger::operator<< ( const std::string& _s ) {
  slotLog(defaultLogType_,QString( _s.c_str() ));
}


//=============================================================================

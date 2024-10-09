#ifndef TB_LOGTYPE_H
#define TB_LOGTYPE_H

#include <QMetaType>

enum Logtype {
    LOGSTATUS,  /*!< Status log messages. Will be printed in blue in the logwidget */
    LOGOUT,     /*!< Standard log messages. Will be printed in black in the logwidget */
    LOGINFO,    /*!< Info log messages. Will be printed in green in the logwidget */
    LOGWARN,    /*!< Warning messages. Will be printed in yellow in the logwidget */
    LOGERR      /*!< Error messages. Will be printed in red in the logwidget */
};

Q_DECLARE_METATYPE(Logtype)

#endif // TB_LOGTYPE_H
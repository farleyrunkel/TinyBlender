#ifndef TINYBLENDERSETTINGS_H
#define TINYBLENDERSETTINGS_H

#include <QSettings>

class Settings : public QSettings
{
public:

    // 静态设置值的方法，直接操作单例实例
    static void setValue(const QString& key, const QVariant& value) {
        instance().QSettings::setValue(key, value);  // 直接调用父类 QSettings 的 setValue

    }

    // 静态获取值的方法，直接操作单例实例
    static QVariant value(const QString& key, const QVariant& defaultValue = QVariant()) {
        return instance().QSettings::value(key, defaultValue);
    }

private:

    // Private constructor to prevent direct instantiation
    Settings() = default;

    // Deleted copy constructor and assignment operator to prevent copying
    Settings(const Settings&) = delete;
    Settings& operator=(const Settings&) = delete;

    // Static method to access the singleton instance
    static Settings& instance();

};

#endif // TINYBLENDERSETTINGS_H

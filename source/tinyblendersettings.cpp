#include "tinyblendersettings.h"


TinyBlenderSettings &TinyBlenderSettings::instance()
{
    static TinyBlenderSettings instance;
    return instance;
}

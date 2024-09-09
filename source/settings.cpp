#include "settings.h"


Settings &Settings::instance()
{
    static Settings instance;
    return instance;
}

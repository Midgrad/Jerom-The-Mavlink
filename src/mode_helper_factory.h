#ifndef MODE_HELPER_FACTORY_H
#define MODE_HELPER_FACTORY_H

#include <stdint.h>

#include "i_mode_helper.h"

namespace jerom_mavlink::data_source
{
class ModeHelperFactory
{
public:
    static IModeHelper* create(uint8_t autopilot, uint8_t type);
};
} // namespace jerom_mavlink::data_source

#endif // MODE_HELPER_FACTORY_H

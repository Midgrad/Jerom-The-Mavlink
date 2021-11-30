#ifndef MODE_HELPER_FACTORY_H
#define MODE_HELPER_FACTORY_H

#include <stdint.h>

#include "i_mode_helper.h"

namespace md::data_source
{
class ModeHelperFactory
{
public:
    static IModeHelper* create(uint8_t autopilot, uint8_t type);
};
} // namespace md::data_source

#endif // MODE_HELPER_FACTORY_H

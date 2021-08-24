#ifndef JEROM_MAVLINK_TRANSLATION_H
#define JEROM_MAVLINK_TRANSLATION_H

#include "i_translation.h"

namespace jerom_mavlink::app
{

    class MavlinkTranslation final: public ITranslation
    {
        MavlinkTranslation();

        bytearray_t translate_in(bytearray_t data) override;
        bytearray_t translate_out(bytearray_t data) override;

    };

} // namespace
#endif //JEROM_MAVLINK_TRANSLATION_H

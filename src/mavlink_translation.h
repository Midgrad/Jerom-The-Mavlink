#ifndef MAVLINK_TRANSLATION_H
#define MAVLINK_TRANSLATION_H

#include "i_translation.h"

namespace jerom_mavlink::app
{
class MavlinkTranslation final : public ITranslation
{
    MavlinkTranslation();

    bytearray_t translate_in(bytearray_t data) override;
    bytearray_t translate_out(bytearray_t data) override;
};

} // namespace jerom_mavlink::app
#endif //MAVLINK_TRANSLATION_H

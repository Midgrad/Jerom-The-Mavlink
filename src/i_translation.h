#ifndef I_TRANSLATION_H
#define I_TRANSLATION_H

#include <vector>

namespace jerom_mavlink::app
{
using bytearray_t = std::vector<char>;

class ITranslation
{
public:
    virtual ~ITranslation() = default;

    virtual bytearray_t translate_in(bytearray_t data) = 0;
    virtual bytearray_t translate_out(bytearray_t data) = 0;
};
} // namespace jerom_mavlink::app

#endif //I_TRANSLATION_H

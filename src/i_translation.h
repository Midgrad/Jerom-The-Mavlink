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

    virtual bytearray_t translateIn(bytearray_t data) = 0;
    virtual bytearray_t translateOut(bytearray_t data) = 0;
};
} // namespace jerom_mavlink::app

#endif //I_TRANSLATION_H

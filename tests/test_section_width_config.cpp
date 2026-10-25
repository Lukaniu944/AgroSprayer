#include "section_width_config.h"

#include <cassert>
#include <limits>

int main()
{
    const float defaults[6] = {2.5f, 4.0f, 4.0f, 4.0f, 4.0f, 2.5f};
    assert(section_width_config::valid(defaults, 6));
    assert(!section_width_config::valid(nullptr, 6));
    assert(!section_width_config::valid(defaults, 5));

    const float wrongTotal[6] = {2.4f, 4.0f, 4.0f, 4.0f, 4.0f, 2.5f};
    assert(!section_width_config::valid(wrongTotal, 6));

    float hasNan[6] = {2.5f, 4.0f, 4.0f, 4.0f, 4.0f, 2.5f};
    hasNan[2] = std::numeric_limits<float>::quiet_NaN();
    assert(!section_width_config::valid(hasNan, 6));

    float hasInfinity[6] = {2.5f, 4.0f, 4.0f, 4.0f, 4.0f, 2.5f};
    hasInfinity[4] = std::numeric_limits<float>::infinity();
    assert(!section_width_config::valid(hasInfinity, 6));

    const float tooNarrow[6] = {0.09f, 4.1f, 4.0f, 4.0f, 4.0f, 2.5f};
    assert(!section_width_config::valid(tooNarrow, 6));

    const float tooWide[6] = {20.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
    assert(!section_width_config::valid(tooWide, 6));

    const float roundingOk[6] = {2.55f, 4.0f, 4.0f, 4.0f, 4.0f, 2.45f};
    assert(section_width_config::valid(roundingOk, 6));

    const float roundingBad[6] = {2.55f, 4.0f, 4.0f, 4.0f, 4.0f, 2.502f};
    assert(!section_width_config::valid(roundingBad, 6));
}

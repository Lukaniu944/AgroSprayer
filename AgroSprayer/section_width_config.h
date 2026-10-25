#ifndef AGROSPRAYER_SECTION_WIDTH_CONFIG_H
#define AGROSPRAYER_SECTION_WIDTH_CONFIG_H

#include <cmath>
#include <cstddef>

namespace section_width_config {

constexpr std::size_t kRequiredSectionCount = 6;

inline bool valid(const float *widths, std::size_t count)
{
    if (!widths || count != kRequiredSectionCount) return false;

    float total = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(widths[i]) || widths[i] < 0.1f || widths[i] > 20.0f)
            return false;
        total += widths[i];
    }

    // Widths are edited in 0.1 m steps; this tolerance only covers float rounding.
    return std::isfinite(total) && std::fabs(total - 21.0f) <= 0.051f;
}

} // namespace section_width_config

#endif // AGROSPRAYER_SECTION_WIDTH_CONFIG_H

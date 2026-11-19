#include "configuration_interlock_core.h"
#include <cstdlib>
#include <iostream>

int main() {
    int checks = 0;
    auto expect = [&](bool condition, const char *name) {
        if (!condition) {
            std::cerr << "FAIL: " << name << "\\n";
            std::abort();
        }
        ++checks;
    };

    expect(agroconfig::configurationEditAllowed({false, false, false, false}),
           "idle system permits configuration edits");
    expect(!agroconfig::configurationEditAllowed({true, false, false, false}),
           "bench simulation blocks configuration edits");
    expect(!agroconfig::configurationEditAllowed({false, true, false, false}),
           "any active section blocks configuration edits");
    expect(!agroconfig::configurationEditAllowed({false, false, true, false}),
           "armed regulator blocks configuration edits");
    expect(!agroconfig::configurationEditAllowed({false, false, false, true}),
           "armed or moving valve blocks configuration edits");
    expect(!agroconfig::configurationEditAllowed({true, true, true, true}),
           "multiple active interlocks remain blocked");

    std::cout << "OK: " << checks
              << " configuration interlock assertions passed\\n";
    return 0;
}

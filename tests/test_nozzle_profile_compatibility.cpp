#include "nozzle_profile_compatibility.h"
#include <cassert>

int main()
{
    assert(nozzle_profile_compatible(0, 0));  // Water + 11003
    assert(!nozzle_profile_compatible(1, 0)); // Water cannot select 04RSM
    assert(!nozzle_profile_compatible(0, 1)); // RSM 28 requires 04RSM
    assert(nozzle_profile_compatible(1, 1));
    assert(!nozzle_profile_compatible(0, 2)); // RSM 30 requires 04RSM
    assert(nozzle_profile_compatible(1, 2));
    assert(!nozzle_profile_compatible(0, 3)); // RSM 32 requires 04RSM
    assert(nozzle_profile_compatible(1, 3));
    assert(nozzle_profile_compatible(0, 4));  // Custom profile permits either
    assert(nozzle_profile_compatible(1, 4));
    assert(!nozzle_profile_compatible(2, 4));
    assert(!nozzle_profile_compatible(0, 5));
    return 0;
}

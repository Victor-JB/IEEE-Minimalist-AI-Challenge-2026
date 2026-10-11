// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cerrno>
namespace stepper {
// Shared SLEEP: asleep whenever idle, automatic wake only for an accepted move.
struct Power {
    bool enabled = true, awake = false;
    uint32_t generation = 1, wokeAt = 0;
    bool ready(uint32_t now) const { return awake && uint32_t(now-wokeAt) >= 3; }
    int change(int requested, uint32_t expected) {
        if (requested != 0 && requested != 1) return -EINVAL;
        if (expected != generation) return -ESTALE;
        enabled = requested;
        generation = generation == 0x7fffffff ? 1 : generation+1;
        return 0;
    }
    bool update(bool moving, uint32_t now) {
        const bool next = enabled && moving;
        if (next == awake) return false;
        awake = next; wokeAt = now;
        return true;
    }
    bool permits(uint32_t expected) const { return enabled && expected == generation; }
};
}

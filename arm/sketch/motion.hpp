// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>
#include <cerrno>
#include <array>

namespace stepper {
constexpr unsigned AXES = 3, MAX_POINTS = 8, MAX_HZ = 2400, MIN_HZ = 20;
constexpr uint32_t HOLD_MS = 300, DIR_WAIT_MS = 60;
enum Phase { READY, STARTING, RAMPING, RUNNING, STOPPED, LINK_LOST, ERROR, LIMIT_STOP };
struct Point { uint32_t ms = 0, hz = 0; };
struct Motor {
    uint32_t owner = 0, session = 0, last_id = 0, hz = 0, target = 0, ppr = 0;
    uint32_t started = 0, renewed = 0, directionSet = 0;
    unsigned count = 0;
    bool running = false, direction = false, wanted = false, prepared = false;
    int fault = 0;
    Phase phase = READY;
    std::array<Point, MAX_POINTS> points{};
    void stop(Phase reason = STOPPED) {
        running = false; hz = 0; session = 0; phase = reason;
    }
    int claim(uint32_t token) {
        if (!token) return -EINVAL;
        if (running) return -EBUSY;
        stop(fault ? ERROR : READY); owner = token; last_id = 0;
        return fault;
    }
    int start(uint32_t token, uint32_t id, unsigned dir, uint32_t steps,
              const Point *curve, unsigned n, uint32_t now) {
        if (!token || token != owner) return -EACCES;
        if (fault) return fault;
        if (!id || id <= last_id || dir > 1 || !steps || steps > 100000 ||
            n < 2 || n > MAX_POINTS || curve[0].ms || curve[0].hz) return -EINVAL;
        for (unsigned i=1; i<n; ++i) {
            if (curve[i].ms <= curve[i-1].ms || curve[i].ms-curve[i-1].ms < 20 ||
                curve[i].ms > 60000 || curve[i].hz > MAX_HZ ||
                (curve[i].hz && curve[i].hz < MIN_HZ)) return -EINVAL;
        }
        if (curve[n-1].hz < MIN_HZ) return -EINVAL;
        if (running) return -EBUSY;
        for (unsigned i=0; i<n; ++i) points[i] = curve[i];
        count = n; last_id = session = id; wanted = dir; ppr = steps;
        target = curve[n-1].hz; started = renewed = now;
        hz = 0; running = true; prepared = false; phase = STARTING;
        return 0;
    }
    int hold(uint32_t token, uint32_t id, uint32_t now) {
        if (!token || token != owner) return -EACCES;
        // Test expiry before renewal: queued/delayed HOLD cannot revive a lease.
        if (running && uint32_t(now-renewed) >= HOLD_MS) stop(LINK_LOST);
        if (!running || id != session || !id) return -ESTALE;
        renewed = now; return 0;
    }
    void tick(uint32_t now) {
        if (!running) return;
        if (uint32_t(now-renewed) >= HOLD_MS) { stop(LINK_LOST); return; }
        uint32_t elapsed = now-started;
        if (elapsed < DIR_WAIT_MS) return;
        if (!prepared) { direction = wanted; directionSet = now; prepared = true; return; }
        // Measure setup from the actual DIR update, even after a delayed tick.
        elapsed = now-directionSet;
        if (elapsed < 20) return;
        elapsed -= 20;
        const uint32_t finish = points[count-1].ms;
        if (elapsed >= finish) { hz = target; phase = RUNNING; return; }
        for (unsigned i=1; i<count; ++i) {
            if (elapsed <= points[i].ms) {
                const auto &a = points[i-1], &b = points[i];
                int64_t value = a.hz + (int64_t(b.hz)-a.hz) * (elapsed-a.ms) / (b.ms-a.ms);
                hz = value > 0 && value < MIN_HZ ? MIN_HZ : uint32_t(value);
                phase = RAMPING; return;
            }
        }
    }
};
}

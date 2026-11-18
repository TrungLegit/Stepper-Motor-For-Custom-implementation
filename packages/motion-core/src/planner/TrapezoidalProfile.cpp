#include "planner/TrapezoidalProfile.hpp"
#include <cmath>
#include <algorithm>

namespace motion_core {

void TrapezoidalProfile::calculate(float distance, float v_start, float v_target,
                                   float v_max, float a_max, float /* j_max */) {
    m_distance = std::max(0.0f, distance);
    m_v_start = std::max(0.0f, v_start);
    m_v_target = std::max(0.0f, v_target);
    m_v_max = std::max(0.001f, v_max);
    m_a_max = std::max(0.001f, a_max);

    if (m_distance <= 1e-6f) {
        m_total_duration = 0.0f;
        m_peak_velocity = 0.0f;
        m_phases.fill(PhaseInfo{});
        return;
    }

    m_v_start = std::min(m_v_start, m_v_max);
    m_v_target = std::min(m_v_target, m_v_max);

    // Distance required to accelerate to v_max and decelerate to v_target
    const float sa_max = (m_v_max * m_v_max - m_v_start * m_v_start) / (2.0f * m_a_max);
    const float sd_max = (m_v_max * m_v_max - m_v_target * m_v_target) / (2.0f * m_a_max);
    const float s_needed = sa_max + sd_max;

    float T1 = 0.0f, T2 = 0.0f, T3 = 0.0f;

    if (m_distance >= s_needed) {
        // Full trapezoidal move with cruise phase
        m_peak_velocity = m_v_max;
        T1 = (m_v_max - m_v_start) / m_a_max;
        T2 = (m_distance - s_needed) / m_v_max;
        T3 = (m_v_max - m_v_target) / m_a_max;
    } else {
        // Triangular move (cruise collapses, T2 = 0)
        T2 = 0.0f;
        const float v_pk_sq = (2.0f * m_a_max * m_distance + m_v_start * m_v_start + m_v_target * m_v_target) * 0.5f;
        if (v_pk_sq > 0.0f) {
            m_peak_velocity = std::min(m_v_max, std::sqrt(v_pk_sq));
        } else {
            m_peak_velocity = std::max(m_v_start, m_v_target);
        }
        T1 = std::max(0.0f, (m_peak_velocity - m_v_start) / m_a_max);
        T3 = std::max(0.0f, (m_peak_velocity - m_v_target) / m_a_max);
    }

    // Set up Phase 1 (Acceleration)
    m_phases[0].duration = T1;
    m_phases[0].start_time = 0.0f;
    m_phases[0].end_time = T1;
    m_phases[0].start_pos = 0.0f;
    m_phases[0].start_vel = m_v_start;
    m_phases[0].end_vel = m_peak_velocity;
    m_phases[0].accel = (T1 > 1e-9f) ? m_a_max : 0.0f;
    m_phases[0].end_pos = m_v_start * T1 + 0.5f * m_phases[0].accel * T1 * T1;

    // Set up Phase 2 (Cruise)
    m_phases[1].duration = T2;
    m_phases[1].start_time = T1;
    m_phases[1].end_time = T1 + T2;
    m_phases[1].start_pos = m_phases[0].end_pos;
    m_phases[1].start_vel = m_peak_velocity;
    m_phases[1].end_vel = m_peak_velocity;
    m_phases[1].accel = 0.0f;
    m_phases[1].end_pos = m_phases[1].start_pos + m_peak_velocity * T2;

    // Set up Phase 3 (Deceleration)
    m_phases[2].duration = T3;
    m_phases[2].start_time = T1 + T2;
    m_phases[2].end_time = T1 + T2 + T3;
    m_phases[2].start_pos = m_phases[1].end_pos;
    m_phases[2].start_vel = m_peak_velocity;
    m_phases[2].end_vel = m_v_target;
    m_phases[2].accel = (T3 > 1e-9f) ? -m_a_max : 0.0f;
    m_phases[2].end_pos = m_distance; // Ensure exact final distance

    m_total_duration = T1 + T2 + T3;
}

float TrapezoidalProfile::getVelocity(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f) {
        return m_v_start;
    }
    if (t >= m_total_duration) {
        return m_v_target;
    }

    for (const auto& p : m_phases) {
        if (t <= p.end_time || &p == &m_phases[2]) {
            const float tau = t - p.start_time;
            return p.start_vel + p.accel * tau;
        }
    }
    return m_v_target;
}

float TrapezoidalProfile::getPosition(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f) {
        return 0.0f;
    }
    if (t >= m_total_duration) {
        return m_distance;
    }

    for (const auto& p : m_phases) {
        if (t <= p.end_time || &p == &m_phases[2]) {
            const float tau = t - p.start_time;
            return p.start_pos + p.start_vel * tau + 0.5f * p.accel * tau * tau;
        }
    }
    return m_distance;
}

float TrapezoidalProfile::getAcceleration(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f || t >= m_total_duration) {
        return 0.0f;
    }

    for (const auto& p : m_phases) {
        if (t <= p.end_time || &p == &m_phases[2]) {
            return p.accel;
        }
    }
    return 0.0f;
}

uint8_t TrapezoidalProfile::getPhase(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f) {
        return 1;
    }
    if (t >= m_total_duration) {
        return 3;
    }

    for (size_t i = 0; i < 3; ++i) {
        if (t <= m_phases[i].end_time) {
            return static_cast<uint8_t>(i + 1);
        }
    }
    return 3;
}

void TrapezoidalProfile::generateStepTable(float steps_per_unit, uint32_t timer_freq_hz,
                                           std::vector<StepInterval>& out_table,
                                           uint32_t max_segments) const {
    out_table.clear();
    const uint32_t total_steps = static_cast<uint32_t>(std::round(m_distance * steps_per_unit));
    if (total_steps == 0 || m_total_duration <= 1e-6f) {
        return;
    }

    if (total_steps <= 256) {
        for (uint32_t step = 0; step < total_steps; ++step) {
            const float target_s = (static_cast<float>(step) + 0.5f) / steps_per_unit;
            float t_low = 0.0f;
            float t_high = m_total_duration;
            for (int it = 0; it < 18; ++it) {
                const float t_mid = 0.5f * (t_low + t_high);
                if (getPosition(t_mid) < target_s) {
                    t_low = t_mid;
                } else {
                    t_high = t_mid;
                }
            }
            const float t_step = 0.5f * (t_low + t_high);
            const float vel = std::max(0.001f, getVelocity(t_step));
            const float dt = 1.0f / (vel * steps_per_unit);
            const uint32_t ticks = std::max(1U, static_cast<uint32_t>(std::round(dt * static_cast<float>(timer_freq_hz))));
            out_table.push_back({1, ticks});
        }
    } else {
        const uint32_t num_segments = std::min(max_segments, total_steps);
        const float dt_seg = m_total_duration / static_cast<float>(num_segments);

        uint32_t accumulated_steps = 0;
        for (uint32_t seg = 0; seg < num_segments; ++seg) {
            const float t_a = static_cast<float>(seg) * dt_seg;
            const float t_b = static_cast<float>(seg + 1) * dt_seg;
            const float s_a = getPosition(t_a);
            const float s_b = getPosition(t_b);

            const uint32_t step_a = static_cast<uint32_t>(std::round(s_a * steps_per_unit));
            const uint32_t step_b = static_cast<uint32_t>(std::round(s_b * steps_per_unit));
            const uint32_t seg_steps = (step_b > step_a) ? (step_b - step_a) : 0;

            if (seg_steps > 0) {
                const float v_avg = std::max(0.001f, (s_b - s_a) / dt_seg);
                const float step_dt = 1.0f / (v_avg * steps_per_unit);
                const uint32_t ticks = std::max(1U, static_cast<uint32_t>(std::round(step_dt * static_cast<float>(timer_freq_hz))));
                out_table.push_back({seg_steps, ticks});
                accumulated_steps += seg_steps;
            }
        }

        if (accumulated_steps != total_steps && !out_table.empty()) {
            const int32_t diff = static_cast<int32_t>(total_steps) - static_cast<int32_t>(accumulated_steps);
            auto max_it = std::max_element(out_table.begin(), out_table.end(),
                [](const StepInterval& a, const StepInterval& b) {
                    return a.step_count < b.step_count;
                });
            if (max_it != out_table.end()) {
                const int32_t new_count = static_cast<int32_t>(max_it->step_count) + diff;
                if (new_count > 0) {
                    max_it->step_count = static_cast<uint32_t>(new_count);
                }
            }
        }
    }
}

} // namespace motion_core

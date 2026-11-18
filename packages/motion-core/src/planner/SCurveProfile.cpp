#include "planner/SCurveProfile.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace motion_core {

void SCurveProfile::calculate(float distance, float v_start, float v_target,
                              float v_max, float a_max, float j_max) {
    m_distance = std::max(0.0f, distance);
    m_v_start = std::max(0.0f, v_start);
    m_v_target = std::max(0.0f, v_target);
    m_v_max = std::max(0.001f, v_max);
    m_a_max = std::max(0.001f, a_max);
    m_j_max = std::max(0.001f, j_max);

    // If zero or near-zero distance, profile is zero duration
    if (m_distance <= 1e-6f) {
        m_total_duration = 0.0f;
        m_peak_velocity = 0.0f;
        m_peak_accel = 0.0f;
        m_phases.fill(PhaseInfo{});
        return;
    }

    // Clamp boundary velocities to v_max
    m_v_start = std::min(m_v_start, m_v_max);
    m_v_target = std::min(m_v_target, m_v_max);

    // Jerk time needed to reach a_max
    const float Tj_accel = m_a_max / m_j_max;
    const float Tj_decel = m_a_max / m_j_max;

    // Helper lambda to compute acceleration ramp distance and times given target v_peak
    auto evalAccel = [&](float v_pk, float& t1, float& t2, float& t3, float& a_pk) -> float {
        const float dv = v_pk - m_v_start;
        if (dv <= 0.0f) {
            t1 = t2 = t3 = 0.0f;
            a_pk = 0.0f;
            return 0.0f;
        }
        const float dv_jerk = m_a_max * Tj_accel; // = a_max^2 / j_max
        if (dv < dv_jerk) {
            t1 = t3 = std::sqrt(dv / m_j_max);
            t2 = 0.0f;
            a_pk = m_j_max * t1;
        } else {
            t1 = t3 = Tj_accel;
            t2 = (dv - dv_jerk) / m_a_max;
            a_pk = m_a_max;
        }
        const float T_a = 2.0f * t1 + t2;
        return 0.5f * (m_v_start + v_pk) * T_a;
    };

    // Helper lambda to compute deceleration ramp distance and times given target v_peak
    auto evalDecel = [&](float v_pk, float& t5, float& t6, float& t7, float& a_pk) -> float {
        const float dv = v_pk - m_v_target;
        if (dv <= 0.0f) {
            t5 = t6 = t7 = 0.0f;
            a_pk = 0.0f;
            return 0.0f;
        }
        const float dv_jerk = m_a_max * Tj_decel;
        if (dv < dv_jerk) {
            t5 = t7 = std::sqrt(dv / m_j_max);
            t6 = 0.0f;
            a_pk = m_j_max * t5;
        } else {
            t5 = t7 = Tj_decel;
            t6 = (dv - dv_jerk) / m_a_max;
            a_pk = m_a_max;
        }
        const float T_d = 2.0f * t5 + t6;
        return 0.5f * (v_pk + m_v_target) * T_d;
    };

    float T1 = 0.0f, T2 = 0.0f, T3 = 0.0f, T4 = 0.0f, T5 = 0.0f, T6 = 0.0f, T7 = 0.0f;
    float a_accel_peak = 0.0f, a_decel_peak = 0.0f;

    // First evaluate if full v_max can be reached
    const float dist_accel_max = evalAccel(m_v_max, T1, T2, T3, a_accel_peak);
    const float dist_decel_max = evalDecel(m_v_max, T5, T6, T7, a_decel_peak);
    const float dist_needed_for_vmax = dist_accel_max + dist_decel_max;

    if (m_distance >= dist_needed_for_vmax) {
        // Full cruise (Phase 4 exists)
        m_peak_velocity = m_v_max;
        m_peak_accel = std::max(a_accel_peak, a_decel_peak);
        T4 = (m_distance - dist_needed_for_vmax) / m_v_max;
    } else {
        // Cruise collapses (T4 = 0), short-move collapse
        T4 = 0.0f;
        float low = std::max(m_v_start, m_v_target);
        float high = m_v_max;
        for (int i = 0; i < 35; ++i) {
            float mid = 0.5f * (low + high);
            float d1, d2, d3, d5, d6, d7, a1, a2;
            float stot = evalAccel(mid, d1, d2, d3, a1) + evalDecel(mid, d5, d6, d7, a2);
            if (stot < m_distance) {
                low = mid;
            } else {
                high = mid;
            }
        }
        m_peak_velocity = 0.5f * (low + high);
        evalAccel(m_peak_velocity, T1, T2, T3, a_accel_peak);
        evalDecel(m_peak_velocity, T5, T6, T7, a_decel_peak);
        m_peak_accel = std::max(a_accel_peak, a_decel_peak);
    }

    computePhases(T1, T2, T3, T4, T5, T6, T7, m_j_max, m_j_max);
}

void SCurveProfile::computePhases(float T1, float T2, float T3, float T4, float T5, float T6, float T7,
                                  float J_accel, float J_decel) {
    const std::array<float, 7> T = {T1, T2, T3, T4, T5, T6, T7};
    const std::array<float, 7> J = {J_accel, 0.0f, -J_accel, 0.0f, -J_decel, 0.0f, J_decel};

    float t_curr = 0.0f;
    float s_curr = 0.0f;
    float v_curr = m_v_start;
    float a_curr = 0.0f;

    for (size_t i = 0; i < 7; ++i) {
        PhaseInfo& p = m_phases[i];
        p.duration = T[i];
        p.start_time = t_curr;
        p.end_time = t_curr + T[i];
        p.start_pos = s_curr;
        p.start_vel = v_curr;
        p.start_accel = a_curr;
        p.jerk = J[i];

        const float dt = T[i];
        if (dt > 1e-9f) {
            p.end_accel = a_curr + p.jerk * dt;
            p.end_vel = v_curr + a_curr * dt + 0.5f * p.jerk * dt * dt;
            p.end_pos = s_curr + v_curr * dt + 0.5f * a_curr * dt * dt + (1.0f / 6.0f) * p.jerk * dt * dt * dt;
        } else {
            p.end_accel = a_curr;
            p.end_vel = v_curr;
            p.end_pos = s_curr;
        }

        t_curr = p.end_time;
        s_curr = p.end_pos;
        v_curr = p.end_vel;
        a_curr = p.end_accel;
    }

    m_total_duration = t_curr;
    m_phases[6].end_pos = m_distance;
}

float SCurveProfile::getVelocity(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f) {
        return m_v_start;
    }
    if (t >= m_total_duration) {
        return m_v_target;
    }

    for (const auto& p : m_phases) {
        if (t <= p.end_time || &p == &m_phases[6]) {
            const float tau = t - p.start_time;
            return p.start_vel + p.start_accel * tau + 0.5f * p.jerk * tau * tau;
        }
    }
    return m_v_target;
}

float SCurveProfile::getPosition(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f) {
        return 0.0f;
    }
    if (t >= m_total_duration) {
        return m_distance;
    }

    for (const auto& p : m_phases) {
        if (t <= p.end_time || &p == &m_phases[6]) {
            const float tau = t - p.start_time;
            return p.start_pos + p.start_vel * tau + 0.5f * p.start_accel * tau * tau + (1.0f / 6.0f) * p.jerk * tau * tau * tau;
        }
    }
    return m_distance;
}

float SCurveProfile::getAcceleration(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f || t >= m_total_duration) {
        return 0.0f;
    }

    for (const auto& p : m_phases) {
        if (t <= p.end_time || &p == &m_phases[6]) {
            const float tau = t - p.start_time;
            return p.start_accel + p.jerk * tau;
        }
    }
    return 0.0f;
}

float SCurveProfile::getJerk(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f || t >= m_total_duration) {
        return 0.0f;
    }

    for (const auto& p : m_phases) {
        if (t <= p.end_time || &p == &m_phases[6]) {
            return p.jerk;
        }
    }
    return 0.0f;
}

uint8_t SCurveProfile::getPhase(float t) const {
    if (m_total_duration <= 1e-6f || t <= 0.0f) {
        return 1;
    }
    if (t >= m_total_duration) {
        return 7;
    }

    for (size_t i = 0; i < 7; ++i) {
        if (t <= m_phases[i].end_time) {
            return static_cast<uint8_t>(i + 1);
        }
    }
    return 7;
}

void SCurveProfile::generateStepTable(float steps_per_unit, uint32_t timer_freq_hz,
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

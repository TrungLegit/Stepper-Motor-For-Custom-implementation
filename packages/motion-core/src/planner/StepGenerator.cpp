#include "planner/StepGenerator.hpp"
#include <cstring>
#include <algorithm>

namespace motion_core {

StepGenerator::StepGenerator() {
    m_axis_steps.fill(0);
    m_direction.fill(Direction::POSITIVE);
    m_dda_accum.fill(0);
    m_current_steps.fill(0);
}

StepGenerator::~StepGenerator() {
    stop();
    if (m_timer_hal && m_timer_handle != INVALID_TIMER_HANDLE) {
        m_timer_hal->cancelTimer(m_timer_handle);
        m_timer_handle = INVALID_TIMER_HANDLE;
    }
}

void StepGenerator::init(IDriverHAL* driver_hal, ITimerHAL* timer_hal, uint32_t timer_freq_hz) {
    m_driver_hal = driver_hal;
    m_timer_hal = timer_hal;
    m_timer_freq_hz = (timer_freq_hz > 0) ? timer_freq_hz : 100000;

    if (m_timer_hal) {
        const uint32_t period_us = std::max(1U, 1000000U / m_timer_freq_hz);
        if (m_timer_handle != INVALID_TIMER_HANDLE) {
            m_timer_hal->cancelTimer(m_timer_handle);
        }
        m_timer_handle = m_timer_hal->createPeriodicTimer(period_us, [this]() {
            stepTimerTick();
        });
    }
}

bool StepGenerator::loadMove(uint8_t lead_axis,
                            const std::array<uint32_t, MAX_AXES>& axis_steps,
                            const std::array<Direction, MAX_AXES>& directions,
                            const StepInterval* table,
                            size_t table_size) {
    if (!table || table_size == 0 || table_size > MAX_SEGMENTS || lead_axis >= MAX_AXES) {
        return false;
    }

    m_lead_axis = lead_axis;
    m_lead_steps = axis_steps[lead_axis];
    if (m_lead_steps == 0) {
        return false;
    }

    m_axis_steps = axis_steps;
    m_direction = directions;
    m_dda_accum.fill(0);

    // Apply DIR and ENABLE signals to hardware driver
    if (m_driver_hal) {
        for (uint8_t i = 0; i < MAX_AXES; ++i) {
            if (m_axis_steps[i] > 0) {
                m_driver_hal->setDirPin(i, m_direction[i]);
                m_driver_hal->setEnablePin(i, true);
            }
        }
    }

    // Copy step table into fixed internal array
    for (size_t i = 0; i < table_size; ++i) {
        m_table[i] = table[i];
    }
    m_total_segments = table_size;
    m_current_segment_idx = 0;
    m_segment_steps_remaining = m_table[0].step_count;
    m_current_interval_ticks = m_table[0].isr_ticks_per_step;
    m_pulse_active = false;

    return true;
}

void StepGenerator::start() {
    if (m_total_segments > 0 && m_lead_steps > 0) {
        m_is_running = true;
    }
}

void StepGenerator::stop() {
    m_is_running = false;
    m_pulse_active = false;
    if (m_driver_hal) {
        for (uint8_t i = 0; i < MAX_AXES; ++i) {
            m_driver_hal->setStepPin(i, PinState::LOW);
        }
    }
}

void StepGenerator::pause() {
    m_is_running = false;
}

void StepGenerator::resume() {
    if (m_total_segments > 0 && m_current_segment_idx < m_total_segments) {
        m_is_running = true;
    }
}

void StepGenerator::stepTimerTick() {
    // Return immediately if no motion is running
    if (!m_is_running) {
        return;
    }

    // 1. Reset active step pulse to LOW from previous tick
    if (m_pulse_active) {
        if (m_driver_hal) {
            for (uint8_t i = 0; i < MAX_AXES; ++i) {
                m_driver_hal->setStepPin(i, PinState::LOW);
            }
        }
        m_pulse_active = false;
    }

    // 2. Decrement interval tick countdown
    if (--m_current_interval_ticks > 0) {
        return;
    }

    // 3. Step trigger: Fire STEP pulse on lead axis
    if (m_driver_hal) {
        m_driver_hal->setStepPin(m_lead_axis, PinState::HIGH);
    }
    m_current_steps[m_lead_axis] += (m_direction[m_lead_axis] == Direction::POSITIVE) ? 1 : -1;
    if (m_on_step_cb) {
        m_on_step_cb(m_lead_axis, m_current_steps[m_lead_axis]);
    }

    // 4. Multi-axis Bresenham / DDA interpolation for secondary axes
    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        if (i == m_lead_axis || m_axis_steps[i] == 0) {
            continue;
        }

        m_dda_accum[i] += static_cast<int32_t>(m_axis_steps[i]);
        if (m_dda_accum[i] >= static_cast<int32_t>(m_lead_steps)) {
            m_dda_accum[i] -= static_cast<int32_t>(m_lead_steps);
            if (m_driver_hal) {
                m_driver_hal->setStepPin(i, PinState::HIGH);
            }
            m_current_steps[i] += (m_direction[i] == Direction::POSITIVE) ? 1 : -1;
            if (m_on_step_cb) {
                m_on_step_cb(i, m_current_steps[i]);
            }
        }
    }
    m_pulse_active = true;

    // 5. Advance segment step counter
    if (--m_segment_steps_remaining == 0) {
        // Move to next segment
        if (++m_current_segment_idx >= m_total_segments) {
            // Move completely finished
            m_is_running = false;
            m_total_segments = 0;
            if (m_on_move_complete) {
                m_on_move_complete();
            }
            return;
        }

        m_segment_steps_remaining = m_table[m_current_segment_idx].step_count;
        m_current_interval_ticks = m_table[m_current_segment_idx].isr_ticks_per_step;
    } else {
        // Reload ticks for next step in current segment
        m_current_interval_ticks = m_table[m_current_segment_idx].isr_ticks_per_step;
    }
}

} // namespace motion_core

#include "CaptureEngine.hpp"
#include <cstdio>
#include <algorithm>

namespace scanner {

CaptureEngine::CaptureEngine(MotionPlanner& planner, ScannerKinematics& kinematics)
    : m_planner(planner), m_kinematics(kinematics) {}

CaptureEngine::~CaptureEngine() {
    abort();
}

void CaptureEngine::init(IGpioHAL* gpio_hal, ITimerHAL* timer_hal) {
    m_gpio_hal = gpio_hal;
    m_timer_hal = timer_hal;

    if (m_gpio_hal) {
        m_gpio_hal->pinMode(m_cfg.shutter_gpio_pin, PinMode::OUTPUT);
        m_gpio_hal->digitalWrite(m_cfg.shutter_gpio_pin, PinState::LOW);

        if (m_cfg.ack_gpio_pin > 0) {
            m_gpio_hal->pinMode(m_cfg.ack_gpio_pin, PinMode::INPUT_PULLUP);
        }
    }
}

bool CaptureEngine::startScan(std::vector<SphericalPose> path) {
    if (path.empty()) {
        return false;
    }

    m_poses = std::move(path);
    m_logged_poses.clear();
    m_logged_poses.reserve(m_poses.size());

    m_current_pose_idx = 0;
    m_retry_count = 0;
    m_pause_requested = false;
    m_ack_received = false;
    m_ack_success = false;

    if (!dispatchMoveToPose(m_poses[0])) {
        transitionTo(CaptureState::FAILED, 0);
        return false;
    }

    transitionTo(CaptureState::MOVE_TO_POSE, 0);
    return true;
}

void CaptureEngine::pause() {
    if (isScanning()) {
        m_pause_requested = true;
    }
}

void CaptureEngine::resume() {
    if (m_state == CaptureState::PAUSED) {
        m_pause_requested = false;
        if (m_current_pose_idx < m_poses.size()) {
            dispatchMoveToPose(m_poses[m_current_pose_idx]);
            transitionTo(CaptureState::MOVE_TO_POSE, 0);
        } else {
            transitionTo(CaptureState::SCAN_COMPLETE, 0);
            if (m_on_scan_finished) {
                m_on_scan_finished(true);
            }
        }
    }
}

void CaptureEngine::abort() {
    m_planner.emergencyStop();
    if (m_gpio_hal && m_shutter_pin_active) {
        m_gpio_hal->digitalWrite(m_cfg.shutter_gpio_pin, PinState::LOW);
        m_shutter_pin_active = false;
    }
    transitionTo(CaptureState::ABORTED, 0);
}

void CaptureEngine::notifyAck(bool success) {
    m_ack_received = true;
    m_ack_success = success;
}

void CaptureEngine::transitionTo(CaptureState next_state, uint64_t current_time_us) {
    m_state = next_state;
    m_state_start_us = current_time_us;

    if (m_on_progress) {
        m_on_progress(m_state, m_current_pose_idx, static_cast<uint32_t>(m_poses.size()));
    }
}

bool CaptureEngine::dispatchMoveToPose(const SphericalPose& pose) {
    std::array<float, MAX_AXES> targets{};
    targets[0] = static_cast<float>(pose.azimuth_deg);
    targets[1] = static_cast<float>(pose.elevation_deg);
    targets[2] = static_cast<float>(pose.focus_z_mm);
    targets[3] = 0.0f;

    return m_planner.moveTo(targets, m_cfg.travel_feedrate_mm_s, ProfileType::SCURVE);
}

void CaptureEngine::update(uint64_t current_time_us) {
    // 1. Shutter pulse pulse-width timer
    if (m_shutter_pin_active) {
        if (current_time_us - m_shutter_start_us >= m_cfg.shutter_pulse_us) {
            if (m_gpio_hal) {
                m_gpio_hal->digitalWrite(m_cfg.shutter_gpio_pin, PinState::LOW);
            }
            m_shutter_pin_active = false;
        }
    }

    // 2. Acquisition state machine
    switch (m_state) {
        case CaptureState::MOVE_TO_POSE:
            if (!m_planner.isMoving()) {
                // Slew complete; enter vibration damping delay
                transitionTo(CaptureState::SETTLING_DELAY, current_time_us);
            }
            break;

        case CaptureState::SETTLING_DELAY:
            if (current_time_us - m_state_start_us >= static_cast<uint64_t>(m_cfg.settle_delay_ms) * 1000ULL) {
                // Mechanical vibrations have damped out; trigger camera shutter
                transitionTo(CaptureState::TRIGGER_CAMERA, current_time_us);
            }
            break;

        case CaptureState::TRIGGER_CAMERA:
            if (m_gpio_hal) {
                m_gpio_hal->digitalWrite(m_cfg.shutter_gpio_pin, PinState::HIGH);
                m_shutter_pin_active = true;
                m_shutter_start_us = current_time_us;
            }

            m_ack_received = false;
            m_ack_success = false;

            if (m_software_hook && m_current_pose_idx < m_poses.size()) {
                m_software_hook(m_current_pose_idx, m_poses[m_current_pose_idx], [this](bool ok) {
                    notifyAck(ok);
                });
            }

            transitionTo(CaptureState::WAIT_ACK, current_time_us);
            break;

        case CaptureState::WAIT_ACK:
            // Check hardware GPIO ACK pin if configured
            if (!m_ack_received && m_cfg.ack_gpio_pin > 0 && m_gpio_hal) {
                if (m_gpio_hal->digitalRead(m_cfg.ack_gpio_pin) == PinState::LOW) {
                    notifyAck(true);
                }
            }

            // Auto-acknowledge if enabled and no external software hook
            if (!m_ack_received && m_cfg.auto_ack && !m_software_hook) {
                if (current_time_us - m_state_start_us >= (static_cast<uint64_t>(m_cfg.shutter_pulse_us) + 10000ULL)) {
                    notifyAck(true);
                }
            }

            if (m_ack_received) {
                if (m_ack_success) {
                    m_retry_count = 0;
                    transitionTo(CaptureState::LOG_POSE, current_time_us);
                } else {
                    if (m_retry_count < m_cfg.max_retries) {
                        ++m_retry_count;
                        transitionTo(CaptureState::SETTLING_DELAY, current_time_us);
                    } else {
                        transitionTo(CaptureState::FAILED, current_time_us);
                        if (m_on_scan_finished) {
                            m_on_scan_finished(false);
                        }
                    }
                }
            } else if (current_time_us - m_state_start_us >= static_cast<uint64_t>(m_cfg.ack_timeout_ms) * 1000ULL) {
                // Timeout elapsed without ACK
                if (m_retry_count < m_cfg.max_retries) {
                    ++m_retry_count;
                    transitionTo(CaptureState::SETTLING_DELAY, current_time_us);
                } else {
                    transitionTo(CaptureState::FAILED, current_time_us);
                    if (m_on_scan_finished) {
                        m_on_scan_finished(false);
                    }
                }
            }
            break;

        case CaptureState::LOG_POSE:
            if (m_current_pose_idx < m_poses.size()) {
                char img_name[64];
                std::snprintf(img_name, sizeof(img_name), "frame_%04u.jpg", m_current_pose_idx);
                auto record = m_kinematics.createPoseRecord(m_current_pose_idx, img_name, m_poses[m_current_pose_idx]);
                m_logged_poses.push_back(record);

                if (m_on_pose_captured) {
                    m_on_pose_captured(record);
                }
            }
            transitionTo(CaptureState::NEXT_POSE, current_time_us);
            break;

        case CaptureState::NEXT_POSE:
            if (m_pause_requested) {
                transitionTo(CaptureState::PAUSED, current_time_us);
                return;
            }

            ++m_current_pose_idx;
            if (m_current_pose_idx < m_poses.size()) {
                dispatchMoveToPose(m_poses[m_current_pose_idx]);
                transitionTo(CaptureState::MOVE_TO_POSE, current_time_us);
            } else {
                transitionTo(CaptureState::SCAN_COMPLETE, current_time_us);
                if (m_on_scan_finished) {
                    m_on_scan_finished(true);
                }
            }
            break;

        default:
            break;
    }
}

} // namespace scanner

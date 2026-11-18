#pragma once

#include "ScannerKinematics.hpp"
#include "planner/MotionPlanner.hpp"
#include "hal/IGpioHAL.hpp"
#include "hal/ITimerHAL.hpp"
#include "protocol/MeshLabSchema.hpp"
#include <vector>
#include <functional>
#include <cstdint>
#include <string>

namespace scanner {

/**
 * @brief Discrete operational states of the 3D capture engine.
 */
enum class CaptureState : uint8_t {
    IDLE = 0,
    MOVE_TO_POSE,      // Axis motors slewing to next viewpoint
    SETTLING_DELAY,    // Non-blocking timer damping mechanical vibration
    TRIGGER_CAMERA,    // Pulsing optocoupler shutter and/or software hook
    WAIT_ACK,          // Awaiting camera sensor capture acknowledgment
    LOG_POSE,          // Archiving frame telemetry and 6-DoF extrinsics
    NEXT_POSE,         // Incrementing viewpoint index
    PAUSED,            // Scan paused safely between viewpoints
    SCAN_COMPLETE,     // Full scan trajectory completed
    ABORTED,           // Emergency stopped or user aborted
    FAILED             // Camera ACK timeout or motor error
};

/**
 * @brief Timing and hardware configuration for image acquisition.
 */
struct CaptureConfig {
    uint32_t settle_delay_ms{200};        // Post-move settling delay (100-500ms)
    uint32_t shutter_pulse_us{50000};     // Shutter GPIO pulse width in microseconds (50ms)
    uint32_t ack_timeout_ms{5000};        // Camera ACK timeout before retry/abort
    uint8_t  max_retries{2};              // Retry attempts per viewpoint
    uint8_t  shutter_gpio_pin{12};        // Optocoupler trigger GPIO pin
    uint8_t  ack_gpio_pin{0};             // Hardware flash/ready input pin (0 = none)
    bool     auto_ack{true};              // Auto-acknowledge if no hardware ACK pin
    float    travel_feedrate_mm_s{0.0f};  // Coordinated axis feedrate (0 = default max)
};

/**
 * @brief High-precision non-blocking 3D scanner capture orchestrator.
 *
 * Drives the cyclical workflow:
 * Slewing -> Vibration Settle -> Shutter Trigger -> Camera ACK -> Telemetry Logging -> Next Viewpoint
 */
class CaptureEngine {
public:
    using SoftwareTriggerHook = std::function<void(uint32_t frame_id,
                                                   const SphericalPose& pose,
                                                   std::function<void(bool ok)> ack_fn)>;
    using PoseCapturedCallback = std::function<void(const protocol::ScanPoseRecord& record)>;
    using ScanFinishedCallback = std::function<void(bool success)>;
    using ProgressCallback = std::function<void(CaptureState state, uint32_t current_idx, uint32_t total_poses)>;

    CaptureEngine(MotionPlanner& planner, ScannerKinematics& kinematics);
    ~CaptureEngine();

    void init(IGpioHAL* gpio_hal, ITimerHAL* timer_hal = nullptr);
    void setConfig(const CaptureConfig& config) { m_cfg = config; }
    const CaptureConfig& getConfig() const noexcept { return m_cfg; }

    /**
     * @brief Load a scan trajectory and start automated acquisition.
     */
    bool startScan(std::vector<SphericalPose> path);

    /**
     * @brief Pause scan at next safe viewpoint boundary.
     */
    void pause();

    /**
     * @brief Resume a paused scan.
     */
    void resume();

    /**
     * @brief Immediately abort active scan and halt all motion.
     */
    void abort();

    /**
     * @brief Deliver an asynchronous capture confirmation (e.g. from HTTP/PTP client).
     */
    void notifyAck(bool success);

    /**
     * @brief Non-blocking state machine update tick.
     * @param current_time_us Current system time in microseconds
     */
    void update(uint64_t current_time_us);

    CaptureState getState() const noexcept { return m_state; }
    uint32_t getCurrentIndex() const noexcept { return m_current_pose_idx; }
    uint32_t getTotalPoses() const noexcept { return static_cast<uint32_t>(m_poses.size()); }
    bool isScanning() const noexcept {
        return m_state != CaptureState::IDLE &&
               m_state != CaptureState::SCAN_COMPLETE &&
               m_state != CaptureState::ABORTED &&
               m_state != CaptureState::FAILED;
    }

    const std::vector<protocol::ScanPoseRecord>& getLoggedPoses() const noexcept {
        return m_logged_poses;
    }

    void setSoftwareTriggerHook(SoftwareTriggerHook hook) {
        m_software_hook = std::move(hook);
    }

    void setOnPoseCaptured(PoseCapturedCallback cb) {
        m_on_pose_captured = std::move(cb);
    }

    void setOnScanFinished(ScanFinishedCallback cb) {
        m_on_scan_finished = std::move(cb);
    }

    void setOnProgress(ProgressCallback cb) {
        m_on_progress = std::move(cb);
    }

private:
    MotionPlanner& m_planner;
    ScannerKinematics& m_kinematics;
    IGpioHAL* m_gpio_hal{nullptr};
    ITimerHAL* m_timer_hal{nullptr};

    CaptureConfig m_cfg;
    std::vector<SphericalPose> m_poses;
    std::vector<protocol::ScanPoseRecord> m_logged_poses;

    CaptureState m_state{CaptureState::IDLE};
    uint32_t m_current_pose_idx{0};
    uint8_t m_retry_count{0};
    bool m_pause_requested{false};
    bool m_ack_received{false};
    bool m_ack_success{false};

    uint64_t m_state_start_us{0};
    uint64_t m_shutter_start_us{0};
    bool m_shutter_pin_active{false};

    SoftwareTriggerHook m_software_hook;
    PoseCapturedCallback m_on_pose_captured;
    ScanFinishedCallback m_on_scan_finished;
    ProgressCallback m_on_progress;

    void transitionTo(CaptureState next_state, uint64_t current_time_us);
    bool dispatchMoveToPose(const SphericalPose& pose);
};

} // namespace scanner

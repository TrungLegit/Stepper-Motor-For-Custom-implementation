#pragma once

/**
 * @file motion_core.hpp
 * @brief Single-include convenience header for stepper-motion-core SDK.
 * 
 * Provides complete access to:
 * - Hardware Abstraction Layer (HAL) interfaces
 * - S-Curve & Trapezoidal Motion Planning Engine
 * - JSON-RPC 2.0 Protocol, Safety E-STOP, Telemetry & MeshLab 3D Schemas
 */

// Hardware Abstraction Layer (HAL) Interfaces
#include "hal/HalTypes.hpp"
#include "hal/IGpioHAL.hpp"
#include "hal/ITimerHAL.hpp"
#include "hal/IDriverHAL.hpp"
#include "hal/IEndstopHAL.hpp"
#include "hal/IEncoderHAL.hpp"
#include "hal/TMC2209Registers.hpp"

// Motion Planning & Kinematics Engine
#include "planner/Axis.hpp"
#include "planner/RingBuffer.hpp"
#include "planner/VelocityProfile.hpp"
#include "planner/SCurveProfile.hpp"
#include "planner/TrapezoidalProfile.hpp"
#include "planner/StepGenerator.hpp"
#include "planner/BacklashCompensator.hpp"
#include "planner/HomingEngine.hpp"
#include "planner/MotionPlanner.hpp"

// Host Protocol, Safety Controller & Telemetry
#include "protocol/ErrorCodes.hpp"
#include "protocol/Json.hpp"
#include "protocol/PacketBuffer.hpp"
#include "protocol/CommandParser.hpp"
#include "protocol/MeshLabSchema.hpp"
#include "protocol/TelemetryEmitter.hpp"
#include "protocol/JsonRpcDispatcher.hpp"

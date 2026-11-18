#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

namespace motion_core {
namespace tmc2209 {

// TMC2209 UART Register Addresses
constexpr uint8_t REG_GCONF        = 0x00;
constexpr uint8_t REG_GSTAT        = 0x01;
constexpr uint8_t REG_IFCNT        = 0x02;
constexpr uint8_t REG_SLAVECONF    = 0x03;
constexpr uint8_t REG_OTP_PROG     = 0x04;
constexpr uint8_t REG_OTP_READ     = 0x05;
constexpr uint8_t REG_IOIN         = 0x06;
constexpr uint8_t REG_FACTORY_CONF = 0x07;
constexpr uint8_t REG_IHOLD_IRUN   = 0x10;
constexpr uint8_t REG_TPOWERDOWN   = 0x11;
constexpr uint8_t REG_TSTEP        = 0x12;
constexpr uint8_t REG_TPWMTHRS     = 0x13;
constexpr uint8_t REG_TCOOLTHRS    = 0x14;
constexpr uint8_t REG_THIGH        = 0x15;
constexpr uint8_t REG_SGTHRS       = 0x40;
constexpr uint8_t REG_SG_RESULT    = 0x41;
constexpr uint8_t REG_COOLCONF     = 0x42;
constexpr uint8_t REG_MSCNT        = 0x6A;
constexpr uint8_t REG_MSCURACT     = 0x6B;
constexpr uint8_t REG_CHOPCONF     = 0x6C;
constexpr uint8_t REG_DRV_STATUS   = 0x6F;
constexpr uint8_t REG_PWMCONF      = 0x70;
constexpr uint8_t REG_PWM_SCALE    = 0x71;
constexpr uint8_t REG_PWM_AUTO     = 0x72;

// GCONF bits
constexpr uint32_t GCONF_I_SCALE_ANALOG  = (1u << 0);
constexpr uint32_t GCONF_INTERNAL_RSENSE = (1u << 1);
constexpr uint32_t GCONF_EN_SPREADCYCLE  = (1u << 2);
constexpr uint32_t GCONF_SHAFT           = (1u << 3);
constexpr uint32_t GCONF_INDEX_OTPW      = (1u << 4);
constexpr uint32_t GCONF_INDEX_STEP      = (1u << 5);
constexpr uint32_t GCONF_PDN_DISABLE     = (1u << 6);
constexpr uint32_t GCONF_MSTEP_REG_SELECT= (1u << 7);
constexpr uint32_t GCONF_MULTISTEP_FILT  = (1u << 8);
constexpr uint32_t GCONF_TEST_MODE       = (1u << 9);

// DRV_STATUS bits
constexpr uint32_t DRV_STATUS_OTPW       = (1u << 0);
constexpr uint32_t DRV_STATUS_OT         = (1u << 1);
constexpr uint32_t DRV_STATUS_S2GA       = (1u << 2);
constexpr uint32_t DRV_STATUS_S2GB       = (1u << 3);
constexpr uint32_t DRV_STATUS_OLA        = (1u << 4);
constexpr uint32_t DRV_STATUS_OLB        = (1u << 5);
constexpr uint32_t DRV_STATUS_STST       = (1u << 31); // Standstill

// TMC2209 UART Sync bytes
constexpr uint8_t SYNC_BYTE = 0x05;
constexpr uint8_t MASTER_ADDR = 0xFF;

/**
 * @brief Calculate TMC2209 CRC8 checksum (polynomial: x^8 + x^2 + x + 1 = 0x07)
 */
inline uint8_t calculateCRC8(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        uint8_t current_byte = data[i];
        for (int j = 0; j < 8; ++j) {
            if ((crc >> 7) ^ (current_byte >> 7)) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
            current_byte = static_cast<uint8_t>(current_byte << 1);
        }
    }
    return crc;
}

/**
 * @brief Converts microstep resolution to TMC2209 MRES register field (0-8)
 */
inline uint8_t microstepsToMres(uint16_t microsteps) {
    switch (microsteps) {
        case 256: return 0;
        case 128: return 1;
        case 64:  return 2;
        case 32:  return 3;
        case 16:  return 4;
        case 8:   return 5;
        case 4:   return 6;
        case 2:   return 7;
        case 1:   return 8;
        default:  return 4; // default 1/16
    }
}

/**
 * @brief Converts MRES register field to microsteps
 */
inline uint16_t mresToMicrosteps(uint8_t mres) {
    switch (mres & 0x0F) {
        case 0: return 256;
        case 1: return 128;
        case 2: return 64;
        case 3: return 32;
        case 4: return 16;
        case 5: return 8;
        case 6: return 4;
        case 7: return 2;
        case 8: return 1;
        default: return 16;
    }
}

/**
 * @brief Calculates current scale CS (0-31) from RMS current in Amperes
 * Formula: I_rms = (CS + 1) / 32 * (V_fs / (R_sense + 0.02)) * (1 / sqrt(2))
 * V_fs = 0.325V (typical for vsense = 0)
 */
inline uint8_t currentToCS(float current_rms_amps, float r_sense = 0.11f) {
    constexpr float V_FS = 0.325f;
    constexpr float SQRT_2 = 1.41421356f;
    float cs_float = (current_rms_amps * 32.0f * SQRT_2 * (r_sense + 0.02f) / V_FS) - 1.0f;
    int cs_int = static_cast<int>(std::round(cs_float));
    return static_cast<uint8_t>(std::clamp(cs_int, 0, 31));
}

/**
 * @brief Calculates RMS current in Amperes from current scale CS (0-31)
 */
inline float csToCurrent(uint8_t cs, float r_sense = 0.11f) {
    constexpr float V_FS = 0.325f;
    constexpr float SQRT_2 = 1.41421356f;
    return (static_cast<float>(cs & 0x1F) + 1.0f) / 32.0f * (V_FS / (r_sense + 0.02f)) / SQRT_2;
}

/**
 * @brief Encodes IHOLD_IRUN register value
 */
inline uint32_t encodeIHoldIRun(uint8_t ihold, uint8_t irun, uint8_t iholddelay = 10) {
    return (static_cast<uint32_t>(ihold & 0x1F)) |
           (static_cast<uint32_t>(irun & 0x1F) << 8) |
           (static_cast<uint32_t>(iholddelay & 0x0F) << 16);
}

} // namespace tmc2209
} // namespace motion_core

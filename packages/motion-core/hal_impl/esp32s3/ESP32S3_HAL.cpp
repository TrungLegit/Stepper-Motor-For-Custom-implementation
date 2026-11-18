#include "ESP32S3_HAL.hpp"

#if defined(ESP_PLATFORM)

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "ESP32S3_HAL";

namespace motion_core {
namespace esp32s3 {

ESP32S3_HAL::ESP32S3_HAL(const AxisPins pins[MAX_AXES], const UartPins& uart_cfg)
    : uart_pins_(uart_cfg) {
    for (uint8_t a = 0; a < MAX_AXES; ++a) {
        axis_pins_[a] = pins[a];
        debouncers_[a].configure(EndstopType::SWITCH_NC, 5000, PullMode::PULL_UP);
    }
}

ESP32S3_HAL::~ESP32S3_HAL() {
    if (initialized_) {
        uart_driver_delete(static_cast<uart_port_t>(uart_pins_.uart_num));
    }
}

bool ESP32S3_HAL::init() {
    std::lock_guard<std::mutex> lock(mutex_);

    // 1. Configure Stepper Pins (Step, Dir, Enable)
    for (uint8_t a = 0; a < MAX_AXES; ++a) {
        const auto& p = axis_pins_[a];
        if (p.step_pin >= 0) {
            gpio_reset_pin(static_cast<gpio_num_t>(p.step_pin));
            gpio_set_direction(static_cast<gpio_num_t>(p.step_pin), GPIO_MODE_OUTPUT);
            gpio_set_level(static_cast<gpio_num_t>(p.step_pin), 0);
        }
        if (p.dir_pin >= 0) {
            gpio_reset_pin(static_cast<gpio_num_t>(p.dir_pin));
            gpio_set_direction(static_cast<gpio_num_t>(p.dir_pin), GPIO_MODE_OUTPUT);
            gpio_set_level(static_cast<gpio_num_t>(p.dir_pin), 0);
        }
        if (p.enable_pin >= 0) {
            gpio_reset_pin(static_cast<gpio_num_t>(p.enable_pin));
            gpio_set_direction(static_cast<gpio_num_t>(p.enable_pin), GPIO_MODE_OUTPUT);
            // TMC2209 enable is active LOW; disable (HIGH) on startup for safety
            gpio_set_level(static_cast<gpio_num_t>(p.enable_pin), 1);
        }
        if (p.endstop_pin >= 0) {
            configureEndstop(a, debouncers_[a].getType(), debouncers_[a].getDebounceUs(), debouncers_[a].getPullMode());
        }
    }

    // 2. Configure TMC2209 UART
    if (!initUart()) {
        ESP_LOGE(TAG, "Failed to initialize TMC2209 UART on port %d", uart_pins_.uart_num);
        return false;
    }

    initialized_ = true;
    return true;
}

bool ESP32S3_HAL::initUart() {
    uart_config_t uart_config = {
        .baud_rate = static_cast<int>(uart_pins_.baud_rate),
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_port_t port = static_cast<uart_port_t>(uart_pins_.uart_num);
    esp_err_t err = uart_param_config(port, &uart_config);
    if (err != ESP_OK) return false;

    err = uart_set_pin(port, uart_pins_.tx_pin, uart_pins_.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return false;

    // Buffer size 256 bytes for single-wire / half-duplex UART
    err = uart_driver_install(port, 256, 256, 0, nullptr, 0);
    return (err == ESP_OK);
}

void ESP32S3_HAL::pinMode(uint8_t pin, PinMode mode) {
    gpio_mode_t gmode = GPIO_MODE_INPUT;
    gpio_pullup_t pullup = GPIO_PULLUP_DISABLE;
    gpio_pulldown_t pulldown = GPIO_PULLDOWN_DISABLE;

    switch (mode) {
        case PinMode::INPUT:
            gmode = GPIO_MODE_INPUT;
            break;
        case PinMode::OUTPUT:
            gmode = GPIO_MODE_OUTPUT;
            break;
        case PinMode::INPUT_PULLUP:
            gmode = GPIO_MODE_INPUT;
            pullup = GPIO_PULLUP_ENABLE;
            break;
        case PinMode::INPUT_PULLDOWN:
            gmode = GPIO_MODE_INPUT;
            pulldown = GPIO_PULLDOWN_ENABLE;
            break;
        case PinMode::OUTPUT_OPEN_DRAIN:
            gmode = GPIO_MODE_OUTPUT_OD;
            break;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = gmode,
        .pull_up_en = pullup,
        .pull_down_en = pulldown,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void ESP32S3_HAL::digitalWrite(uint8_t pin, PinState state) {
    gpio_set_level(static_cast<gpio_num_t>(pin), (state == PinState::HIGH) ? 1 : 0);
}

PinState ESP32S3_HAL::digitalRead(uint8_t pin) {
    return (gpio_get_level(static_cast<gpio_num_t>(pin)) != 0) ? PinState::HIGH : PinState::LOW;
}

void ESP32S3_HAL::togglePin(uint8_t pin) {
    PinState cur = digitalRead(pin);
    digitalWrite(pin, (cur == PinState::HIGH) ? PinState::LOW : PinState::HIGH);
}

void ESP32S3_HAL::attachInterrupt(uint8_t pin, GpioInterruptCallback /*cb*/, TriggerEdge /*edge*/) {
    // Basic wrapper for ESP32 GPIO ISR attachment
    gpio_install_isr_service(0);
}

void ESP32S3_HAL::detachInterrupt(uint8_t pin) {
    gpio_isr_handler_remove(static_cast<gpio_num_t>(pin));
}

void ESP32S3_HAL::setStepPin(uint8_t axis, PinState state) {
    if (axis < MAX_AXES && axis_pins_[axis].step_pin >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(axis_pins_[axis].step_pin), (state == PinState::HIGH) ? 1 : 0);
    }
}

void ESP32S3_HAL::setDirPin(uint8_t axis, Direction dir) {
    if (axis < MAX_AXES && axis_pins_[axis].dir_pin >= 0) {
        int lvl = (dir == Direction::POSITIVE) ? 1 : 0;
        if (driver_configs_[axis].invert_dir) {
            lvl = 1 - lvl;
        }
        gpio_set_level(static_cast<gpio_num_t>(axis_pins_[axis].dir_pin), lvl);
    }
}

void ESP32S3_HAL::setEnablePin(uint8_t axis, bool enabled) {
    if (axis < MAX_AXES && axis_pins_[axis].enable_pin >= 0) {
        // TMC2209 is active LOW: 0 = enabled, 1 = disabled
        gpio_set_level(static_cast<gpio_num_t>(axis_pins_[axis].enable_pin), enabled ? 0 : 1);
    }
}

void ESP32S3_HAL::step(uint8_t axis) {
    if (axis < MAX_AXES && axis_pins_[axis].step_pin >= 0) {
        gpio_num_t pin = static_cast<gpio_num_t>(axis_pins_[axis].step_pin);
        gpio_set_level(pin, 1);
        esp_rom_delay_us(1); // minimal step pulse width
        gpio_set_level(pin, 0);
    }
}

bool ESP32S3_HAL::configureDriver(uint8_t axis, const DriverConfig& cfg) {
    if (axis >= MAX_AXES) return false;
    driver_configs_[axis] = cfg;
    bool ok = true;
    ok &= setMicrosteps(axis, cfg.microsteps);
    ok &= setRunCurrent(axis, cfg.run_current_amps);
    ok &= setHoldCurrent(axis, cfg.hold_current_amps);
    ok &= setStealthChop(axis, cfg.stealthchop_enabled);
    ok &= setStallGuardThreshold(axis, cfg.stallguard_threshold);
    return ok;
}

bool ESP32S3_HAL::setMicrosteps(uint8_t axis, uint16_t microsteps) {
    if (axis >= MAX_AXES) return false;
    uint32_t chopconf = 0;
    if (!readRegister(axis, tmc2209::REG_CHOPCONF, chopconf)) return false;

    uint8_t mres = tmc2209::microstepsToMres(microsteps);
    chopconf = (chopconf & ~(0x0Fu << 24)) | (static_cast<uint32_t>(mres & 0x0F) << 24);
    if (driver_configs_[axis].interpolate) {
        chopconf |= (1u << 28); // INTPOL
    } else {
        chopconf &= ~(1u << 28);
    }
    return writeRegister(axis, tmc2209::REG_CHOPCONF, chopconf);
}

bool ESP32S3_HAL::setRunCurrent(uint8_t axis, float current_amps) {
    if (axis >= MAX_AXES) return false;
    uint8_t cs = tmc2209::currentToCS(current_amps, driver_configs_[axis].r_sense);
    uint32_t val = 0;
    readRegister(axis, tmc2209::REG_IHOLD_IRUN, val);
    val = (val & ~(0x1Fu << 8)) | (static_cast<uint32_t>(cs & 0x1F) << 8);
    return writeRegister(axis, tmc2209::REG_IHOLD_IRUN, val);
}

bool ESP32S3_HAL::setHoldCurrent(uint8_t axis, float current_amps) {
    if (axis >= MAX_AXES) return false;
    uint8_t cs = tmc2209::currentToCS(current_amps, driver_configs_[axis].r_sense);
    uint32_t val = 0;
    readRegister(axis, tmc2209::REG_IHOLD_IRUN, val);
    val = (val & ~0x1Fu) | static_cast<uint32_t>(cs & 0x1F);
    return writeRegister(axis, tmc2209::REG_IHOLD_IRUN, val);
}

bool ESP32S3_HAL::setStealthChop(uint8_t axis, bool enable) {
    if (axis >= MAX_AXES) return false;
    uint32_t gconf = 0;
    readRegister(axis, tmc2209::REG_GCONF, gconf);
    if (enable) {
        gconf &= ~tmc2209::GCONF_EN_SPREADCYCLE;
    } else {
        gconf |= tmc2209::GCONF_EN_SPREADCYCLE;
    }
    return writeRegister(axis, tmc2209::REG_GCONF, gconf);
}

bool ESP32S3_HAL::setStallGuardThreshold(uint8_t axis, uint8_t threshold) {
    return writeRegister(axis, tmc2209::REG_SGTHRS, threshold);
}

bool ESP32S3_HAL::readStallGuard(uint8_t axis, uint16_t& sg_val) {
    uint32_t val = 0;
    if (readRegister(axis, tmc2209::REG_SG_RESULT, val)) {
        sg_val = static_cast<uint16_t>(val & 0x3FF); // 10-bit result
        return true;
    }
    return false;
}

bool ESP32S3_HAL::writeRegister(uint8_t axis, uint8_t reg_addr, uint32_t value) {
    if (axis >= MAX_AXES) return false;
    return tmcWrite(axis_pins_[axis].tmc_slave_addr, reg_addr, value);
}

bool ESP32S3_HAL::readRegister(uint8_t axis, uint8_t reg_addr, uint32_t& value) {
    if (axis >= MAX_AXES) return false;
    return tmcRead(axis_pins_[axis].tmc_slave_addr, reg_addr, value);
}

bool ESP32S3_HAL::getDriverStatus(uint8_t axis, DriverStatus& status) {
    uint32_t val = 0;
    if (!readRegister(axis, tmc2209::REG_DRV_STATUS, val)) return false;
    status.otpw = (val & tmc2209::DRV_STATUS_OTPW) != 0;
    status.ot = (val & tmc2209::DRV_STATUS_OT) != 0;
    status.s2ga = (val & tmc2209::DRV_STATUS_S2GA) != 0;
    status.s2gb = (val & tmc2209::DRV_STATUS_S2GB) != 0;
    status.ola = (val & tmc2209::DRV_STATUS_OLA) != 0;
    status.olb = (val & tmc2209::DRV_STATUS_OLB) != 0;
    status.standstill = (val & tmc2209::DRV_STATUS_STST) != 0;
    status.driver_error = status.ot || status.s2ga || status.s2gb;
    return true;
}

bool ESP32S3_HAL::tmcWrite(uint8_t slave_addr, uint8_t reg, uint32_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t buf[8];
    buf[0] = tmc2209::SYNC_BYTE;
    buf[1] = slave_addr;
    buf[2] = reg | 0x80; // Write flag
    buf[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    buf[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buf[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[6] = static_cast<uint8_t>(value & 0xFF);
    buf[7] = tmc2209::calculateCRC8(buf, 7);

    uart_port_t port = static_cast<uart_port_t>(uart_pins_.uart_num);
    uart_flush(port);
    int written = uart_write_bytes(port, buf, 8);
    uart_wait_tx_done(port, 10 / portTICK_PERIOD_MS);
    return (written == 8);
}

bool ESP32S3_HAL::tmcRead(uint8_t slave_addr, uint8_t reg, uint32_t& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t req[4];
    req[0] = tmc2209::SYNC_BYTE;
    req[1] = slave_addr;
    req[2] = reg & 0x7F; // Read flag
    req[3] = tmc2209::calculateCRC8(req, 3);

    uart_port_t port = static_cast<uart_port_t>(uart_pins_.uart_num);
    uart_flush(port);
    uart_write_bytes(port, req, 4);
    uart_wait_tx_done(port, 10 / portTICK_PERIOD_MS);

    // Read echo + response (or just response depending on single-wire mode)
    uint8_t rx_buf[12];
    int len = uart_read_bytes(port, rx_buf, sizeof(rx_buf), 20 / portTICK_PERIOD_MS);
    if (len < 8) return false;

    // Scan for 0x05 sync byte and 0xFF master address response
    for (int i = 0; i <= len - 8; ++i) {
        if (rx_buf[i] == tmc2209::SYNC_BYTE && rx_buf[i + 1] == tmc2209::MASTER_ADDR && rx_buf[i + 2] == (reg & 0x7F)) {
            uint8_t expected_crc = tmc2209::calculateCRC8(&rx_buf[i], 7);
            if (expected_crc == rx_buf[i + 7]) {
                value = (static_cast<uint32_t>(rx_buf[i + 3]) << 24) |
                        (static_cast<uint32_t>(rx_buf[i + 4]) << 16) |
                        (static_cast<uint32_t>(rx_buf[i + 5]) << 8) |
                        static_cast<uint32_t>(rx_buf[i + 6]);
                return true;
            }
        }
    }
    return false;
}

bool ESP32S3_HAL::configureEndstop(uint8_t axis, EndstopType type, uint32_t debounce_us, PullMode pull) {
    if (axis >= MAX_AXES) return false;
    debouncers_[axis].configure(type, debounce_us, pull);
    int8_t pin = axis_pins_[axis].endstop_pin;
    if (pin >= 0) {
        PinMode mode = PinMode::INPUT;
        if (pull == PullMode::PULL_UP) mode = PinMode::INPUT_PULLUP;
        else if (pull == PullMode::PULL_DOWN) mode = PinMode::INPUT_PULLDOWN;
        pinMode(static_cast<uint8_t>(pin), mode);
    }
    return true;
}

bool ESP32S3_HAL::configureEndstop(uint8_t axis, const EndstopConfig& config) {
    return configureEndstop(axis, config.type, config.debounce_us, config.pull);
}

bool ESP32S3_HAL::isTriggered(uint8_t axis) const {
    if (axis >= MAX_AXES) return false;
    return debouncers_[axis].isTriggered();
}

PinState ESP32S3_HAL::rawPinState(uint8_t axis) const {
    if (axis >= MAX_AXES || axis_pins_[axis].endstop_pin < 0) return PinState::LOW;
    return (gpio_get_level(static_cast<gpio_num_t>(axis_pins_[axis].endstop_pin)) != 0) ? PinState::HIGH : PinState::LOW;
}

void ESP32S3_HAL::attachTriggerCallback(uint8_t axis, EndstopCallback cb) {
    if (axis < MAX_AXES) {
        endstop_callbacks_[axis] = cb;
    }
}

void ESP32S3_HAL::detachTriggerCallback(uint8_t axis) {
    if (axis < MAX_AXES) {
        endstop_callbacks_[axis] = nullptr;
    }
}

void ESP32S3_HAL::setDebounceUs(uint8_t axis, uint32_t debounce_us) {
    if (axis < MAX_AXES) {
        debouncers_[axis].setDebounceUs(debounce_us);
    }
}

void ESP32S3_HAL::update(uint64_t current_time_us) {
    for (uint8_t a = 0; a < MAX_AXES; ++a) {
        if (axis_pins_[a].endstop_pin >= 0) {
            PinState raw = rawPinState(a);
            if (debouncers_[a].updateRaw(raw, current_time_us)) {
                if (endstop_callbacks_[a]) {
                    endstop_callbacks_[a](a, debouncers_[a].isTriggered());
                }
            }
        }
    }
}

} // namespace esp32s3
} // namespace motion_core

#else

// Host/Stub fallback implementation for building unit tests on PC
namespace motion_core {
namespace esp32s3 {

ESP32S3_HAL::ESP32S3_HAL(const AxisPins pins[MAX_AXES], const UartPins& uart_cfg)
    : uart_pins_(uart_cfg) {
    for (uint8_t a = 0; a < MAX_AXES; ++a) {
        axis_pins_[a] = pins[a];
        debouncers_[a].configure(EndstopType::SWITCH_NC, 5000, PullMode::PULL_UP);
    }
}

ESP32S3_HAL::~ESP32S3_HAL() = default;

bool ESP32S3_HAL::init() {
    initialized_ = true;
    return true;
}

void ESP32S3_HAL::pinMode(uint8_t /*pin*/, PinMode /*mode*/) {}
void ESP32S3_HAL::digitalWrite(uint8_t /*pin*/, PinState /*state*/) {}
PinState ESP32S3_HAL::digitalRead(uint8_t /*pin*/) { return PinState::LOW; }
void ESP32S3_HAL::togglePin(uint8_t /*pin*/) {}
void ESP32S3_HAL::attachInterrupt(uint8_t /*pin*/, GpioInterruptCallback /*cb*/, TriggerEdge /*edge*/) {}
void ESP32S3_HAL::detachInterrupt(uint8_t /*pin*/) {}

void ESP32S3_HAL::setStepPin(uint8_t /*axis*/, PinState /*state*/) {}
void ESP32S3_HAL::setDirPin(uint8_t /*axis*/, Direction /*dir*/) {}
void ESP32S3_HAL::setEnablePin(uint8_t /*axis*/, bool /*enabled*/) {}
void ESP32S3_HAL::step(uint8_t /*axis*/) {}

bool ESP32S3_HAL::configureDriver(uint8_t axis, const DriverConfig& cfg) {
    if (axis >= MAX_AXES) return false;
    driver_configs_[axis] = cfg;
    return true;
}

bool ESP32S3_HAL::setMicrosteps(uint8_t /*axis*/, uint16_t /*microsteps*/) { return true; }
bool ESP32S3_HAL::setRunCurrent(uint8_t /*axis*/, float /*current_amps*/) { return true; }
bool ESP32S3_HAL::setHoldCurrent(uint8_t /*axis*/, float /*current_amps*/) { return true; }
bool ESP32S3_HAL::setStealthChop(uint8_t /*axis*/, bool /*enable*/) { return true; }
bool ESP32S3_HAL::setStallGuardThreshold(uint8_t /*axis*/, uint8_t /*threshold*/) { return true; }
bool ESP32S3_HAL::readStallGuard(uint8_t /*axis*/, uint16_t& sg_val) { sg_val = 100; return true; }

bool ESP32S3_HAL::writeRegister(uint8_t /*axis*/, uint8_t /*reg_addr*/, uint32_t /*value*/) { return true; }
bool ESP32S3_HAL::readRegister(uint8_t /*axis*/, uint8_t /*reg_addr*/, uint32_t& value) { value = 0; return true; }
bool ESP32S3_HAL::getDriverStatus(uint8_t /*axis*/, DriverStatus& status) {
    status = DriverStatus{};
    return true;
}

bool ESP32S3_HAL::configureEndstop(uint8_t axis, EndstopType type, uint32_t debounce_us, PullMode pull) {
    if (axis >= MAX_AXES) return false;
    debouncers_[axis].configure(type, debounce_us, pull);
    return true;
}

bool ESP32S3_HAL::configureEndstop(uint8_t axis, const EndstopConfig& config) {
    return configureEndstop(axis, config.type, config.debounce_us, config.pull);
}

bool ESP32S3_HAL::isTriggered(uint8_t axis) const {
    if (axis >= MAX_AXES) return false;
    return debouncers_[axis].isTriggered();
}

PinState ESP32S3_HAL::rawPinState(uint8_t /*axis*/) const {
    return PinState::LOW;
}

void ESP32S3_HAL::attachTriggerCallback(uint8_t axis, EndstopCallback cb) {
    if (axis < MAX_AXES) {
        endstop_callbacks_[axis] = cb;
    }
}

void ESP32S3_HAL::detachTriggerCallback(uint8_t axis) {
    if (axis < MAX_AXES) {
        endstop_callbacks_[axis] = nullptr;
    }
}

void ESP32S3_HAL::setDebounceUs(uint8_t axis, uint32_t debounce_us) {
    if (axis < MAX_AXES) {
        debouncers_[axis].setDebounceUs(debounce_us);
    }
}

void ESP32S3_HAL::update(uint64_t current_time_us) {
    for (uint8_t a = 0; a < MAX_AXES; ++a) {
        debouncers_[a].updateTime(current_time_us);
    }
}

} // namespace esp32s3
} // namespace motion_core

#endif

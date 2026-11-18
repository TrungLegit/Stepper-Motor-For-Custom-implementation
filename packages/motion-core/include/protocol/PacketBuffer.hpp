#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <string_view>
#include <optional>

namespace motion_core::protocol {

/**
 * @brief Thread-safe/ISR-friendly packet buffer that buffers incoming byte streams
 *        and extracts newline-delimited JSON-RPC packets.
 *        Includes fast out-of-band inspection for emergency stop (E-STOP) preemption.
 */
class PacketBuffer {
public:
    static constexpr size_t DEFAULT_MAX_CAPACITY = 4096;

    explicit PacketBuffer(size_t max_capacity = DEFAULT_MAX_CAPACITY);

    /**
     * @brief Append raw data chunks from transport layer (UART, USB CDC, or WebSocket).
     * @return true if data was appended, false if truncated/buffer overflow.
     */
    bool push(const char* data, size_t length);
    bool push(std::string_view chunk);

    /**
     * @brief Checks if a full newline-delimited line is ready in the buffer.
     */
    bool hasLine() const noexcept;

    /**
     * @brief Extracts and pops the oldest complete line from the buffer.
     *        Strips trailing '\r' and '\n'.
     * @return The extracted string if available, std::nullopt otherwise.
     */
    std::optional<std::string> popLine();

    /**
     * @brief Fast O(N) inspection for emergency stop command ("system.estop")
     *        in the raw buffer without waiting for newline or JSON parsing.
     *        Allows immediate hardware E-STOP preemption.
     */
    bool peekEmergencyStop() const noexcept;

    /**
     * @brief Clear the internal buffer.
     */
    void clear() noexcept;

    /**
     * @brief Current buffered byte count.
     */
    size_t size() const noexcept { return m_buffer.size(); }

    /**
     * @brief Maximum capacity allowed.
     */
    size_t capacity() const noexcept { return m_max_capacity; }

    /**
     * @brief Check if buffer is full.
     */
    bool isFull() const noexcept { return m_buffer.size() >= m_max_capacity; }

    /**
     * @brief Total number of bytes dropped due to capacity overflow.
     */
    size_t droppedBytes() const noexcept { return m_dropped_bytes; }

private:
    std::string m_buffer;
    size_t m_max_capacity;
    size_t m_dropped_bytes{0};
};

} // namespace motion_core::protocol

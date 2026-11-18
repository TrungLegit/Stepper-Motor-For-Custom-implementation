#include "protocol/PacketBuffer.hpp"
#include <algorithm>

namespace motion_core::protocol {

PacketBuffer::PacketBuffer(size_t max_capacity)
    : m_max_capacity(max_capacity > 64 ? max_capacity : 64) {
    m_buffer.reserve(std::min<size_t>(m_max_capacity, 512));
}

bool PacketBuffer::push(const char* data, size_t length) {
    if (!data || length == 0) return true;

    size_t available = m_max_capacity > m_buffer.size() ? m_max_capacity - m_buffer.size() : 0;
    if (length > available) {
        // Append what fits, drop the rest to protect memory
        size_t to_copy = available;
        m_buffer.append(data, to_copy);
        m_dropped_bytes += (length - to_copy);
        return false;
    }

    m_buffer.append(data, length);
    return true;
}

bool PacketBuffer::push(std::string_view chunk) {
    return push(chunk.data(), chunk.size());
}

bool PacketBuffer::hasLine() const noexcept {
    return m_buffer.find('\n') != std::string::npos;
}

std::optional<std::string> PacketBuffer::popLine() {
    size_t newline_pos = m_buffer.find('\n');
    if (newline_pos == std::string::npos) {
        // If buffer is completely full and has no newline, discard malformed packet to prevent deadlock
        if (m_buffer.size() >= m_max_capacity) {
            m_dropped_bytes += m_buffer.size();
            m_buffer.clear();
        }
        return std::nullopt;
    }

    // Extract line up to newline
    std::string line = m_buffer.substr(0, newline_pos);

    // Erase processed line + '\n' from buffer
    m_buffer.erase(0, newline_pos + 1);

    // Strip carriage return '\r' if present
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }

    return line;
}

bool PacketBuffer::peekEmergencyStop() const noexcept {
    // Fast substring check for "system.estop"
    return m_buffer.find("system.estop") != std::string::npos;
}

void PacketBuffer::clear() noexcept {
    m_buffer.clear();
}

} // namespace motion_core::protocol

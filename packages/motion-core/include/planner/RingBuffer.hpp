#pragma once

#include <cstddef>
#include <array>
#include <atomic>
#include <optional>
#include <type_traits>

namespace motion_core {

/**
 * @brief Lock-free, zero-allocation Single-Producer Single-Consumer (SPSC) Ring Buffer.
 * 
 * Safe for concurrent access between main thread and timer ISR tick.
 * Invariant: Capacity MUST be a power of two for fast bitwise masking.
 */
template <typename T, size_t Capacity = 32>
class RingBuffer {
    static_assert(Capacity > 0, "Capacity must be greater than zero");
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

public:
    RingBuffer() noexcept : m_head(0), m_tail(0) {}

    ~RingBuffer() = default;

    // Non-copyable for concurrency safety
    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

    // Movable
    RingBuffer(RingBuffer&& other) noexcept {
        clear();
        T val;
        while (other.pop(val)) {
            push(std::move(val));
        }
    }

    /**
     * @brief Push an element into the buffer (copy).
     * @return true if pushed, false if buffer is full.
     */
    bool push(const T& item) noexcept {
        const size_t current_head = m_head.load(std::memory_order_relaxed);
        const size_t current_tail = m_tail.load(std::memory_order_acquire);

        if ((current_head - current_tail) >= Capacity) {
            return false; // Buffer is full
        }

        m_buffer[current_head & MASK] = item;
        m_head.store(current_head + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Push an element into the buffer (move).
     * @return true if pushed, false if buffer is full.
     */
    bool push(T&& item) noexcept {
        const size_t current_head = m_head.load(std::memory_order_relaxed);
        const size_t current_tail = m_tail.load(std::memory_order_acquire);

        if ((current_head - current_tail) >= Capacity) {
            return false; // Buffer is full
        }

        m_buffer[current_head & MASK] = std::move(item);
        m_head.store(current_head + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop an element from the buffer.
     * @param out Reference to store the popped item.
     * @return true if popped, false if buffer is empty.
     */
    bool pop(T& out) noexcept {
        const size_t current_tail = m_tail.load(std::memory_order_relaxed);
        const size_t current_head = m_head.load(std::memory_order_acquire);

        if (current_tail == current_head) {
            return false; // Buffer is empty
        }

        out = std::move(m_buffer[current_tail & MASK]);
        m_tail.store(current_tail + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop an element returning std::optional.
     */
    std::optional<T> pop() noexcept {
        T item;
        if (pop(item)) {
            return item;
        }
        return std::nullopt;
    }

    /**
     * @brief Peek at the front element without removing it.
     * @return Pointer to front item, or nullptr if empty.
     */
    const T* peek() const noexcept {
        const size_t current_tail = m_tail.load(std::memory_order_relaxed);
        const size_t current_head = m_head.load(std::memory_order_acquire);

        if (current_tail == current_head) {
            return nullptr;
        }
        return &m_buffer[current_tail & MASK];
    }

    /**
     * @brief Check if the buffer is empty.
     */
    bool empty() const noexcept {
        return m_head.load(std::memory_order_acquire) == m_tail.load(std::memory_order_relaxed);
    }

    /**
     * @brief Check if the buffer is full.
     */
    bool full() const noexcept {
        return (m_head.load(std::memory_order_relaxed) - m_tail.load(std::memory_order_acquire)) >= Capacity;
    }

    /**
     * @brief Current number of elements in the buffer.
     */
    size_t size() const noexcept {
        const size_t head = m_head.load(std::memory_order_acquire);
        const size_t tail = m_tail.load(std::memory_order_relaxed);
        return (head >= tail) ? (head - tail) : 0;
    }

    /**
     * @brief Maximum capacity of the buffer.
     */
    constexpr size_t capacity() const noexcept {
        return Capacity;
    }

    /**
     * @brief Reset buffer to empty state.
     */
    void clear() noexcept {
        m_head.store(0, std::memory_order_relaxed);
        m_tail.store(0, std::memory_order_relaxed);
    }

private:
    static constexpr size_t MASK = Capacity - 1;
    std::array<T, Capacity> m_buffer;
    std::atomic<size_t> m_head{0};
    std::atomic<size_t> m_tail{0};
};

} // namespace motion_core

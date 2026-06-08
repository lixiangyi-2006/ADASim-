**
 * @file Ringbuffer.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 数据流引擎实现
 * @date 2026-06
 * @details 环形缓冲区实现数据存储
 */
#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <vector>
#include <cstddef>

template<typename T>
class RingBuffer
{
public:
    explicit RingBuffer(size_t capacity)
        : buffer(capacity), cap(capacity), head(0), count(0) {}

    void push(const T& item) {
        buffer[head] = item;
        head = (head + 1) % cap;
        if (count < cap) ++count;
    }

    T& operator[](size_t index) {
        size_t oldest = (head - count + cap) % cap;
        size_t pos = (oldest + index) % cap;
        return buffer[pos];
    }

    const T& operator[](size_t index) const {
        size_t oldest = (head - count + cap) % cap;
        size_t pos = (oldest + index) % cap;
        return buffer[pos];
    }

    size_t size() const { return count; }
    size_t capacity() const { return cap; }
    void clear() { head = 0; count = 0; }

private:
    std::vector<T> buffer;
    size_t head;
    size_t count;
    size_t cap;
};

#endif

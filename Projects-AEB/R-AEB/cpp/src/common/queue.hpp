#pragma once

#include <mutex>
#include <condition_variable>
#include <deque>
#include <cstddef>

template <typename T>
class BoundedBlockingQueue {
public:
    explicit BoundedBlockingQueue(std::size_t capacity)
        : capacity_(capacity) {}

    // 生产者：阻塞直到有空间或队列已关闭（关闭后丢弃新任务）
    void push(T&& item) {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_push_.wait(lk, [&] { return queue_.size() < capacity_ || closed_; });
        if (closed_) return;
        queue_.emplace_back(std::move(item));
        cv_pop_.notify_one();
    }

    // 消费者：阻塞直到有数据或队列关闭；返回 false 表示队列已关闭且为空
    bool pop(T& out) {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_pop_.wait(lk, [&] { return !queue_.empty() || closed_; });
        if (queue_.empty()) return false;
        out = std::move(queue_.front());
        queue_.pop_front();
        cv_push_.notify_one();
        return true;
    }

    // 关闭队列：唤醒所有等待中的 push/pop，之后 push 会直接返回，pop 在队列清空后返回 false
    void close() {
        std::lock_guard<std::mutex> lk(mtx_);
        closed_ = true;
        cv_pop_.notify_all();
        cv_push_.notify_all();
    }

private:
    std::mutex mtx_;
    std::condition_variable cv_push_;
    std::condition_variable cv_pop_;
    std::deque<T> queue_;
    std::size_t capacity_;
    bool closed_{false};
};

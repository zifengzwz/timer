#ifndef TIMER_LOCKED_QUEUE_HPP
#define TIMER_LOCKED_QUEUE_HPP

#include <condition_variable>
#include <deque>
#include <mutex>

namespace zifeng {
template <class T>
class locked_queue {
 public:
  locked_queue() = default;
  ~locked_queue() = default;

  void enqueue(const T& t) {
    std::lock_guard<std::mutex> lock(m_);
    q_.emplace_back(t);
    cv_.notify_one();
  }

  void enqueue(T&& t) {
    std::lock_guard<std::mutex> lock(m_);
    q_.emplace_back(std::move(t));
    cv_.notify_one();
  }

  bool dequeue(T& t) {
    std::unique_lock<std::mutex> lock(m_);
    if (cv_.wait_for(lock, std::chrono::seconds(1),
                     [this] { return !q_.empty(); })) {
      t = q_.front();
      q_.pop_front();
      return true;
    }
    return false;
  }

 private:
  std::deque<T> q_;
  std::mutex m_;
  std::condition_variable cv_;
};

}  // namespace zifeng

#endif
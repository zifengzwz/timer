#ifndef TIMER_THREAD_POOL_HPP
#define TIMER_THREAD_POOL_HPP

#include <atomic>
#include <exception>
#include <functional>
#include <thread>
#include <vector>

#include "locked_queue.hpp"

namespace zifeng {
using timer_func = std::function<void()>;
using timer_queue = locked_queue<timer_func>;

class thread_pool {
 public:
  thread_pool(size_t n) {
    if (n == 0 || n > (1 << 10)) {
      throw std::invalid_argument(
          "thread_pool init failed, invalid thread number");
    }

    for (size_t i = 0; i < n; ++i) {
      threads_.emplace_back([this] { worker_loop(); });
    }
  }

  ~thread_pool() {
    for (auto& t : threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
  }

  // no copying
  thread_pool(const thread_pool&) = delete;
  thread_pool& operator=(const thread_pool&) = delete;

  void enqueue(const timer_func& f) { q_.enqueue(f); }

  void stop() { stop_ = true; }

 private:
  void worker_loop() {
    while (!stop_) {
      timer_func f;
      if (q_.dequeue(f)) {
        f();
      }
    }
  }

  timer_queue q_;
  std::vector<std::thread> threads_;
  std::atomic_bool stop_{false};
};

}  // namespace zifeng

#endif
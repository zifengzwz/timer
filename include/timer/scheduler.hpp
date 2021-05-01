#ifndef TIMER_SCHEDULER_HPP
#define TIMER_SCHEDULER_HPP

#include <cstdint>
#include <memory>
#include <unordered_map>

#include "thread_pool.hpp"

namespace zifeng {
struct timer {
  timer* prev{nullptr};
  timer* next{nullptr};
  // timer id
  int64_t id{0};
  // expires time
  int64_t expires{0};
  // > 0 circle timer
  int64_t interval{0};
  // function
  timer_func func;

  timer() = default;
  timer(int64_t idx, int64_t e, int64_t i, const timer_func& f)
      : id(idx), expires(e), interval(i), func(f) {}
};

using timer_ptr = std::shared_ptr<timer>;

// insert tail
inline void timer_emplace_back(timer* head, timer* n) {
  if (!head || !n) return;
  auto tail = head->prev;
  tail->next = n;
  n->prev = tail;
  n->next = head;
  head->prev = n;
}

inline void timer_erase(timer* n) {
  if (!n) return;
  auto prev = n->prev;
  auto next = n->next;
  if (!prev || !next) return;
  prev->next = next;
  next->prev = prev;
}

class scheduler {
 public:
  // wheel slot: 8 + 4 * 6
  static constexpr int64_t wheel_size1 = 1LL << 8;
  static constexpr int64_t wheel_size2 = 1LL << 6;

  // wheel range
  static constexpr int64_t first_range = (1LL << 8) - 1;
  static constexpr int64_t second_range = (1LL << 14) - 1;
  static constexpr int64_t third_range = (1LL << 20) - 1;
  static constexpr int64_t fourth_range = (1LL << 26) - 1;
  static constexpr int64_t fifth_range = (1LL << 32) - 1;

  // 100ms, 10ms, 1ms
  // for high, the range is [0, 2^32ms), about 0～49.7day
  // for medium, the range is [0, 10 * 2^32ms), about 0~1.36year
  // for low, the range is [0, 100 * 2^32ms), aboue 0~13.6year
  enum class precision : int64_t { low = 100, medium = 10, high = 1 };

  // time wheel, 0~4
  enum {
    first_wheel = 0,
    second_wheel,
    third_wheel,
    fourth_wheel,
    fifth_wheel
  };

  static int64_t now_milli() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  }

  scheduler(std::weak_ptr<thread_pool> tp, precision p = precision::high)
      : now_(now_milli()),
        precision_(static_cast<int64_t>(p)),
        thread_pool_(tp) {
    // init time wheel
    for (int64_t i = 0; i <= fifth_wheel; ++i) {
      std::vector<timer_ptr> tmp;
      for (int64_t j = 0; j < (i == 0 ? wheel_size1 : wheel_size2); ++j) {
        auto it = std::make_shared<timer>();
        it->prev = it.get();
        it->next = it.get();
        tmp.emplace_back(std::move(it));
      }
      time_wheel_.emplace_back(std::move(tmp));
    }

    // start loop thread
    loop_thread_ = std::thread([this] { worker_loop(); });
  }

  ~scheduler() {
    if (loop_thread_.joinable()) {
      loop_thread_.join();
    }
  }

  // no copying
  scheduler(const scheduler&) = delete;
  scheduler& operator=(const scheduler) = delete;

  // timeout：ms
  int64_t add(int64_t timeout, const timer_func& f) {
    return add(timeout, 0, f);
  }

  // interval: ms,  > 0, cycle timer
  int64_t add(int64_t timeout, int64_t interval, const timer_func& f) {
    auto adjust = [this](int64_t& time) {
      if (time <= 0) {
        time = 0;
      } else {
        int64_t tmp = time / precision_;
        if (tmp > fifth_range) tmp = fifth_range;
        time = tmp * precision_;
      }
    };

    adjust(timeout);
    adjust(interval);
    int64_t timerid = 0;

    {
      std::lock_guard<std::mutex> lock(loop_mutex_);
      timerid = generate_id();
      auto ptr = std::make_shared<timer>(timerid, now_ + timeout, interval, f);
      timer_map_.emplace(timerid, ptr);
      add(ptr.get());
    }

    return timerid;
  }

  void erase(int64_t id) {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    auto it = timer_map_.find(id);
    if (it == timer_map_.end()) return;
    timer_erase(it->second.get());
    timer_map_.erase(it);
  }

  void stop() {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    stop_ = true;
  }

 private:
  int64_t generate_id() {
    ++id_;
    if (id_ == 0) ++id_;
    while (timer_map_.find(id_) != timer_map_.end()) ++id_;
    return id_;
  }

  int64_t get_slot(int64_t time, int64_t wheel_level) {
    time /= precision_;
    if (wheel_level == 0) {
      return time & 255;
    } else if (wheel_level > first_wheel && wheel_level <= fifth_wheel) {
      return time >> (8 + (wheel_level - 1) * 6) & 63;
    }
    return 0;
  }

  void add(timer* ptr) {
    int64_t duration = ptr->expires - now_;
    duration /= precision_;
    int64_t level = 0, slot = 0;

    // 0 ~255
    if (duration < first_range) {
      slot = get_slot(ptr->expires, first_wheel);
      level = first_wheel;
    }
    // 255 ~ 2^14 - 1
    else if (duration < second_range) {
      slot = get_slot(ptr->expires, second_wheel);
      level = second_wheel;
    }
    // 2^14 ~ 2^20 -1
    else if (duration < third_range) {
      slot = get_slot(ptr->expires, third_wheel);
      level = third_wheel;
    }
    // 2^20 ~ 2^26 -1
    else if (duration < fourth_range) {
      slot = get_slot(ptr->expires, fourth_wheel);
      level = fourth_wheel;
    }
    // 2^26 ~ 2^32 -1
    else if (duration < fifth_range) {
      slot = get_slot(ptr->expires, fifth_wheel);
      level = fifth_wheel;
    } else {
      return;
    }

    timer* head = time_wheel_[level][slot].get();

    // insert timer
    timer_emplace_back(head, ptr);
  }

  void transfer(int64_t level, int64_t slot) {
    auto head = time_wheel_[level][slot].get();
    if (head->next == head) return;

    auto node = head->next;
    while (node != head) {
      auto tmp = node;
      node = node->next;
      add(tmp);
    }

    head->next = head;
    head->prev = head;
  }

  void forearch(int64_t level, int64_t slot) {
    auto head = time_wheel_[level][slot].get();
    if (head->next == head) return;

    auto list = head->next;

    while (list != head) {
      auto tmp = list;
      list = list->next;

      // exec timer function
      if (tmp->func) {
        if (auto tp = thread_pool_.lock()) {
          tp->enqueue(tmp->func);
        } else {
          throw std::runtime_error("no thread runs the timer function");
        }
      }

      if (tmp->interval > 0) {
        tmp->expires += tmp->interval;
        add(tmp);
      } else {
        timer_map_.erase(tmp->id);
      }
    }

    head->next = head;
    head->prev = head;
  }

  void worker_loop() {
    while (true) {
      auto present = std::chrono::steady_clock::now();
      auto next_tick = present + std::chrono::milliseconds(precision_);
      int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                        present.time_since_epoch())
                        .count();
      int64_t slot = 0, firt_slot = 0;

      {
        std::lock_guard<std::mutex> lock(loop_mutex_);
        if (stop_) break;

        while (now - now_ >= 0) {
          slot = get_slot(now_, first_wheel);
          firt_slot = slot;
          if (slot == 0) {
            slot = get_slot(now_, second_wheel);
            transfer(second_wheel, slot);
            if (slot == 0) {
              slot = get_slot(now_, third_wheel);
              transfer(third_wheel, slot);
              if (slot == 0) {
                slot = get_slot(now_, fourth_wheel);
                transfer(fourth_wheel, slot);
                if (slot == 0) {
                  slot = get_slot(now_, fifth_wheel);
                  transfer(fifth_wheel, slot);
                }
              }
            }
          }

          forearch(first_wheel, firt_slot);
          now_ += precision_;
        }
      }

      std::this_thread::sleep_until(next_tick);
    }
  }

  int64_t id_{0};
  int64_t now_{0};
  int64_t precision_{0};
  std::weak_ptr<thread_pool> thread_pool_;
  std::unordered_map<int64_t, timer_ptr> timer_map_;
  std::vector<std::vector<timer_ptr>> time_wheel_;

  std::thread loop_thread_;
  std::mutex loop_mutex_;
  bool stop_{false};
};

}  // namespace zifeng

#endif
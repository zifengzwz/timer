#include <iomanip>
#include <iostream>
#include <sstream>

#include "timer.hpp"

using namespace zifeng;

static std::string now_time_string() {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto milli =
      std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  time_t t = milli / 1000;
  std::stringstream s;
  s << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S") << '.'
    << milli % 1000;
  return s.str();
}

static void cycle_timer_function() {
  std::cout << now_time_string() << ", cycle timer: 1s" << std::endl;
}

int main(int argc, char* argv[]) {
  try {
    // create thread pool
    auto tp = std::make_shared<thread_pool>(1);
    // timer scheduler, default precision: 1ms
    scheduler sch(tp);

    std::cout << now_time_string() << " start timer" << std::endl;

    // timer once
    int64_t start = scheduler::now_milli();
    sch.add(100, [start] {
      std::cout << now_time_string() << " timer once: 100ms, fact: "
                << scheduler::now_milli() - start << "ms" << std::endl;
    });

    start = scheduler::now_milli();
    sch.add(1000, [start] {
      std::cout << now_time_string()
                << " timer once: 1s, fact: " << scheduler::now_milli() - start
                << "ms" << std::endl;
    });

    start = scheduler::now_milli();
    sch.add(5000, [start] {
      std::cout << now_time_string()
                << " timer once: 5s, fact: " << scheduler::now_milli() - start
                << "ms" << std::endl;
    });

    // cycle timer, execute the callback function immediately，and will be
    // executed every 1 second.
    int64_t timerid = sch.add(0, 1000, &cycle_timer_function);

    std::this_thread::sleep_for(std::chrono::seconds(4));

    // erase timer
    sch.erase(timerid);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    sch.stop();
    tp->stop();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  } catch (...) {
    std::cerr << "unkonwn error" << std::endl;
  }

  return 0;
}

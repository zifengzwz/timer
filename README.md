## Introduction

- The timer is implemented by a 5-level time wheel. 
- The timer has three precisions, 1ms, 10ms and 100ms, and the default is 1ms. 
- Support C++11 or later, include header files only. 
- Use the thread pool to execute the callback function asynchronously. 

## Build

```bash
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .
$ ./example
```
## Example

```cpp
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
```


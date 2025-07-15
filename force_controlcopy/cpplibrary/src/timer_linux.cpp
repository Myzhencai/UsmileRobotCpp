#include "RobotUtilities/timer_linux.h"

#include <iostream>
#include <thread>

namespace RUT {
Timer::Timer() {
  _t1 = Clock::now();
  _t2 = Clock::now();
  _next_loop_start_t = Clock::now();
  _loop_duration_s = std::chrono::duration<double>{-1.};
}

Timer::~Timer() {}

TimePoint Timer::now() {
  return Clock::now();
}

TimePoint Timer::tic() {
  _t1 = Clock::now();
  return _t1;
}

void Timer::tic(const TimePoint& time_point) {
  _t1 = time_point;
}

double Timer::toc_ms() {
  _t2 = Clock::now();
  return double(std::chrono::duration_cast<std::chrono::nanoseconds>(_t2 - _t1)
                    .count()) /
         1e6;  // milli second
}

bool Timer::set_loop_rate_hz(double hz) {
  if (hz <= 0) {
    std::cerr << "[Timer] Error: rate must be a positive number." << std::endl;
    return false;
  }
  _loop_duration_s = std::chrono::duration<double>{1. / hz};
  return true;
}

bool Timer::start_timed_loop() {
  _next_loop_start_t = Clock::now();
  return true;
}

bool Timer::sleep_till_next() {
  _next_loop_start_t +=
      std::chrono::duration_cast<std::chrono::nanoseconds>(_loop_duration_s);
  std::this_thread::sleep_until(_next_loop_start_t);
  return true;
}

double Timer::check_for_overrun_ms(bool accumulative) {
  _expect_next_loop_start_t =
      _next_loop_start_t +
      std::chrono::duration_cast<std::chrono::nanoseconds>(_loop_duration_s);
  double overrun = double(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              Clock::now() - _expect_next_loop_start_t)
                              .count()) /
                   1e6;
  if ((overrun > 0) && !accumulative) {
    _next_loop_start_t = Clock::now();
  }
  return overrun;
}

Profiler::Profiler() {
  _running = false;
}
Profiler::~Profiler() {}

void Profiler::start() {
  if (_running) {
    std::cerr << "[Profiler] Error: profiler is already running." << std::endl;
    return;
  }
  _t0 = Clock::now();
  _running = true;
}

void Profiler::stop(std::string name) {
  if (!_running) {
    std::cerr << "[Profiler] Error: profiler is not running." << std::endl;
    return;
  }
  double duration = double(std::chrono::duration_cast<std::chrono::nanoseconds>(
                               Clock::now() - _t0)
                               .count()) /
                    1e6;  // milli second
  if (name.empty()) {
    name = "default";
  }
  if (_logs.find(name) != _logs.end()) {
    // name already exists
    _logs[name] += duration;
  } else {
    _logs[name] = duration;
  }
  _running = false;
}

void Profiler::clear() {
  _logs.clear();
}

void Profiler::show() {
  if (_logs.empty()) {
    std::cout << "[Profiler] No logs to show." << std::endl;
    return;
  }
  std::cout << "==================== Profiler ===================="
            << std::endl;
  for (const auto& log : _logs) {
    std::cout << log.first << ": " << log.second << " ms" << std::endl;
  }
  std::cout << "=================================================" << std::endl;
}

}  // namespace RUT

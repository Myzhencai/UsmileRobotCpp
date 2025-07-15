#ifndef _TIMER_LINUX_H_
#define _TIMER_LINUX_H_

#include <chrono>
#include <cstdlib>
#include <string>
#include <unordered_map>

namespace RUT {

typedef std::chrono::steady_clock Clock;
typedef Clock::time_point TimePoint;

class Timer {
 public:
  Timer();
  ~Timer();

  /*
   *  return epoch time
   */
  TimePoint now();

  /*
   *  duration measurement
   */

  /// @brief Start the timer now. Return the TimePoint of the current time.
  TimePoint tic();
  /// @brief Start the timer from a given time point, as if the timer was tic'ed at that time.
  /// This is used to synchronize multiple timers.
  void tic(const TimePoint& time_point);
  double toc_ms();  // return ms

  // timed loop
  bool set_loop_rate_hz(double hz);
  bool start_timed_loop();
  bool sleep_till_next();
  // won't sleep, only check if the loop rate is not met
  double check_for_overrun_ms(bool accumulative);

 private:
  TimePoint _t1;
  TimePoint _t2;
  TimePoint _next_loop_start_t;
  TimePoint _expect_next_loop_start_t;
  std::chrono::duration<double> _loop_duration_s;
};

class Profiler {
 public:
  Profiler();
  ~Profiler();

  void start();
  void stop(std::string name = "");
  void clear();
  void show();

 private:
  bool _running{false};
  TimePoint _t0;
  std::unordered_map<std::string, double> _logs;
};

}  // namespace RUT

#endif  // _TIMER_LINUX_H_

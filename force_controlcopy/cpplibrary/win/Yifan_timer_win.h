#ifndef _YIFAN_TIMER_WIN_H_
#define _YIFAN_TIMER_WIN_H_

#include <windows.h>

namespace YIFAN {
class Timer {
 public:
  Timer();
  ~Timer();

  void tic();
  double toc();  // return ms

  double _dff;  // machine frequency
 private:
  __int64 _time1;
  __int64 _time2;
  LARGE_INTEGER _large_interger;
};

}  // namespace YIFAN

#endif  // _YIFAN_TIMER_WIN_H_
#include "Yifan_timer_win.h"

using namespace YIFAN;

Timer::Timer() {
  // get machine frequency
  QueryPerformanceFrequency(&_large_interger);
  _dff = _large_interger.QuadPart;

  QueryPerformanceCounter(&_large_interger);
  _time1 = _large_interger.QuadPart;
}

Timer::~Timer() {}

void Timer::tic() {
  QueryPerformanceCounter(&_large_interger);
  _time1 = _large_interger.QuadPart;
}

double Timer::toc() {
  QueryPerformanceCounter(&_large_interger);
  _time2 = _large_interger.QuadPart;

  return (_time2 - _time1) * 1000 / _dff;
}
#ifndef SRC_STOPWATCH_H
#define SRC_STOPWATCH_H


#include <chrono>
#include <ratio>
#include <iostream>
#include <string>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::high_resolution_clock::time_point;

class StopWatch {
private:
  TimePoint time_point_start;
  TimePoint time_point_end;
  bool is_running;
  bool is_reset;

public:
  StopWatch();

  void Reset();

  void Start();

  double ElapsedMilliSeconds();

  double ElapsedMicroSeconds();

  void Stop();

  void PrintMs(std::string str_before);
};


#endif //SRC_STOPWATCH_H

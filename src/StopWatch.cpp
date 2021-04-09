#include "StopWatch.h"

StopWatch::StopWatch() {
  is_running = false;
  is_reset = true;
}

double StopWatch::ElapsedMilliSeconds() {
  if (is_running)
    time_point_end = Clock::now();
  std::chrono::duration<double, std::milli> elapsed_duration =
      time_point_end.time_since_epoch() -
          time_point_start.time_since_epoch();
  return elapsed_duration.count();

}

double StopWatch::ElapsedMicroSeconds() {
  if (is_running)
    time_point_end = Clock::now();
  std::chrono::duration<double, std::micro> elapsed_duration =
      time_point_end.time_since_epoch() -
          time_point_start.time_since_epoch();
  return elapsed_duration.count();
}

void StopWatch::Reset() {
  time_point_start = time_point_end;
  is_reset = true;
  is_running = false;
}

void StopWatch::Stop() {
  time_point_end = Clock::now();
  is_running = false;
}

void StopWatch::Start() {
  if (is_reset) {
    time_point_start = Clock::now();
    is_running = true;
  }
  is_reset = false;
  is_running = true;
}

void StopWatch::PrintMs(std::string str_before) {
  std::cout << str_before << ElapsedMilliSeconds() << " ms." << std::endl;
}

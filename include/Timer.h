#ifndef TIMER_H
#define TIMER_H

#include <chrono>

using namespace std::chrono;

namespace argosServer {

  class Timer {
  public:
    void start() {
      epoch = high_resolution_clock::now();
    }

    high_resolution_clock::duration elapsed() const {
      return high_resolution_clock::now() - epoch;
    }

    hours::rep getHours() const {
      return duration_cast<hours>(elapsed()).count();
    }

    minutes::rep getMinutes() const {
      return duration_cast<minutes>(elapsed()).count();
    }

    seconds::rep getSeconds() const {
      return duration_cast<seconds>(elapsed()).count();
    }

    milliseconds::rep getMilliseconds() const {
      return duration_cast<milliseconds>(elapsed()).count();
    }

    microseconds::rep getMicroseconds() const {
      return duration_cast<microseconds>(elapsed()).count();
    }

  private:
    high_resolution_clock::time_point epoch;
  };

}

#endif

#ifndef TIMER_H_
#define TIMER_H_

#include <glog/logging.h>
#include <chrono>
#include <fstream>
#include <map>
#include <numeric>
#include <string>

class Timer {
 public:
  struct TimerRecord {
    TimerRecord() = default;
    TimerRecord(const std::string& name, double time_usage) {
      func_name_ = name;
      time_usage_in_ms_.emplace_back(time_usage);
    }
    std::string func_name_;
    std::vector<double> time_usage_in_ms_;
  };

  template <class F>
  void Evaluate(F&& func, const std::string& func_name) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count() *
        1000;
    if (records_.find(func_name) != records_.end()) {
      records_[func_name].time_usage_in_ms_.emplace_back(time_used);
    } else {
      records_.insert({func_name, TimerRecord(func_name, time_used)});
    }
  }

  void PrintAll();

  /// clean the records
  void Clear() { records_.clear(); }

 private:
  std::map<std::string, TimerRecord> records_;
};

#endif